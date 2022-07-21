//
//  AuxSeparator.h
//  AuxDataSeparator
//
//  Created by Stone PEN on 19/7/22.
//

#ifndef AuxSeparator_h
#define AuxSeparator_h

#include <string>
#include <filesystem>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "oipshared.h"
#include "CRC.h"

#define SYNC_BYTES          "\x1A\xCF\xFC\x1D"
#define SYNC_BYTES_LEN      4
#define AOS_FRAME_BYTES     1024
#define AOS_HEADER_OFF      4
#define AOS_HEADER_BYTES    6
#define AOS_VCID_OFF        5
#define AOS_VCID_BYTES      1
#define AOS_VCID_MASK       0x3F
#define AOS_VCID_EMPTY      0x3F
#define AOS_VCDUSEQ_OFF     6
#define AOS_VCDUSEQ_BYTES   3
#define AOS_VCDUINJ_OFF     10
#define AOS_VCDUINJ_BYTES   4
#define AOS_VCDUINJ_INVAL   0xAAAAAAAA
#define AOS_VCDUINJ_VALID   0x00000000
#define AOS_DATA_OFF        14
#define AOS_DATA_BYTES      880
#define AOS_EMPTY_DATA      0x55AA
// CRC: assume CRC-16/CCITT-FALSE
// X^16 + X^12 + X^5 + 1, INIT=0xFF, assume: POLY=0x1021, XOROUT=0, REFIN=FALSE, REFOUT=FALSE
#define AOS_CRC_OFF         894
#define AOS_CRC_BITS        16
#define AOS_CRC_INIT        0xFF
#define AOS_LDPC_OFF        896
#define AOS_LDPC_BYTES      128

struct AosFileInfo {
    char station[16];
    char satellite[16];
    short year;
    short month; // 1-based
    short day;   // 1-based
    short hour;  // 1-based
    short minute;// 1-based
    short second;// 1-based
};

struct AosFrameInfo {
    uint8_t vcid;
    uint16_t crc;
    uint32_t vcduSeq;
    uint32_t vcduInj;
    const char * data;
    char ldpc[AOS_LDPC_BYTES];
};

class AuxSeparator
{
public:
    AuxSeparator(const std::string & aosFile, size_t offset = 0) :
    mAosFile(aosFile), mAOS(0), mMapAOS(nullptr), mMapSize(0), mMapOffset(offset)
    {
        int ps = getpagesize();
        if (offset % ps != 0) {
            mMapOffset = offset / ps * ps;
            OLOG("WARNING: offset not aligned with system memory page size, adjusted to %ld (0x%lX).", mMapOffset, mMapOffset);
        }
        
        auto filePath = std::filesystem::path(aosFile);
        if (!ParseFileInfoFromName(filePath.filename().string().c_str(), mAFI)) {
            auto pd = filePath.parent_path();
            if (!ParseFileInfoFromName(pd.filename().string().c_str(), mAFI)) {
                throw std::invalid_argument("unrecognized AOS file name pattern");
            }
        }
        DumpAosFileInfo(mAFI);
    }
    
    ~AuxSeparator()
    {
        if (mMapAOS) munmap(mMapAOS, mMapSize);
        if (mAOS > 0) close(mAOS);
    }
    
public:
    void Separate(const char * outputDir = nullptr) {
        auto od = std::filesystem::current_path().string();
        if (outputDir != nullptr) {
            od = outputDir;
        }
        
        SeparateAosFile(mAosFile, od);
    }
    
protected:
    void SeparateAosFile(const std::string & aosFile, const std::string & outDir) {
        mAOS = open(aosFile.c_str(), O_RDONLY);
        if (mAOS < 0) throw errno_error("open AOS file failed");
        
        struct stat st = { 0 };
        if (fstat(mAOS, &st)) throw errno_error("query file stat failed.");
        mMapSize = (size_t)st.st_size - mMapOffset;
        if ((mMapAOS = mmap(nullptr, mMapSize, PROT_READ, MAP_NOCACHE | MAP_FILE | MAP_SHARED, mAOS, mMapOffset)) == MAP_FAILED) {
            throw errno_error("mmap AOS file failed.");
        }
        
        size_t remain = mMapSize;
        unsigned char * sb = (uint8_t *)SYNC_BYTES;
        OLOG("sync byte: %02X%02X%02X%02X (%d bytes).", sb[0], sb[1], sb[2], sb[3], SYNC_BYTES_LEN);
        AosFrameInfo afi = { 0 };
        for (char * p = (char *)mMapAOS;;) {
            char * frame = NextFrame(p, remain);
            if (!frame) {
                OLOG("No further SYNC-BYTES found in remaining %s bytes of AOS file content.",
                     comma_sep(remain).sep());
                break;
            }
            
            if (!ValidateAosFrame(frame, AOS_FRAME_BYTES, afi)) {
                remain -= frame - p + SYNC_BYTES_LEN;
                p = frame + SYNC_BYTES_LEN;
                OLOG("Invalid frame found, ignored.");
                continue;
            }
            
            OLOG("Found valid frame at byte offset: %06lX", frame - (char *)mMapAOS);
            DumpAosFrameInfo(afi);
            break;
        }
    }
    
    static char * NextFrame(char * p, size_t sz) {
        if (sz < AOS_FRAME_BYTES) return nullptr;
        return (char *)memmem(p, sz, SYNC_BYTES, SYNC_BYTES_LEN);
    }
    
    static bool ValidateAosFrame(char * frame, int len, AosFrameInfo & afi) {
        uint8_t vcid = *((unsigned char *)frame + AOS_VCID_OFF) & AOS_VCID_MASK;
        uint32_t vcduSeq = *(uint32_t *)(frame + AOS_VCDUSEQ_OFF - 1) & 0x00FFFFFF;
        vcduSeq = ntohl(vcduSeq);
        uint32_t vcduInj = *(uint32_t *)(frame + AOS_VCDUINJ_OFF);
        char * data = frame + AOS_DATA_OFF;
        uint16_t crc = *(uint16_t *)(frame + AOS_CRC_OFF);
        crc = ntohs(crc);
        
        if (vcduInj != AOS_VCDUINJ_INVAL && vcduInj != AOS_VCDUINJ_VALID) return false;
        
        afi.vcid = vcid;
        afi.crc = crc;
        afi.vcduSeq = vcduSeq;
        afi.vcduInj = vcduInj;
        afi.data = data;
        //if (vcduInj == AOS_VCDUINJ_INVAL) return true;
        
        // validate CRC
        uint16_t calcedCRC = CRC::Calculate(frame + AOS_HEADER_OFF,
                                            AOS_HEADER_BYTES + AOS_VCDUINJ_BYTES + AOS_DATA_BYTES,
                                            CRC::CRC_16_CCITTFALSE());
        
        OLOG("CRC in frame: %04X, Calculated: %04X.", crc, calcedCRC);
        
        memcpy(afi.ldpc, frame + AOS_LDPC_OFF, AOS_LDPC_BYTES);
        return true;
    }
    
    static bool ParseFileInfoFromName(const char * name, AosFileInfo & afi) {
        int cmos = 0;
        char year[5];
        char month[3];
        char day[3];
        char hour[3];
        char min[3];
        char sec[3];
        if (sscanf(name,
                   "%15[A-Za-z0-9]%*[_-]%15[A-Za-z0-9-]_%4[0-9]%2[0-9]%2[0-9]_%2[0-9]%2[0-9]%2[0-9]_%d",
                   afi.station,
                   afi.satellite,
                   year, month, day,
                   hour, min, sec,
                   &cmos) != 9) {
            memset(&afi, 0, sizeof(AosFileInfo));
            return false;
        }
        
        afi.year  = atoi(year);
        afi.month = atoi(month);
        afi.day   = atoi(day);
        afi.hour  = atoi(hour);
        afi.minute= atoi(min);
        afi.second= atoi(sec);
        
        return true;
    }
    
    static void DumpAosFrameInfo(const AosFrameInfo & afi) {
        OLOG("VCID: %02X", afi.vcid);
        OLOG("VCDU SEQ: %04X", afi.vcduSeq);
        OLOG("CRC: %04X", afi.crc);
    }
    static void DumpAosFileInfo(const AosFileInfo & afi) {
        OLOG("STATION: %s", afi.station);
        OLOG("SATELLITE: %s", afi.satellite);
        OLOG("DATA DATE: %04d%02d%02d",
            afi.year, afi.month, afi.day);
        OLOG("DATA TIME: %02d%02d%02d",
            afi.hour, afi.minute, afi.second);
    }
    
private:
    std::string mAosFile;
    AosFileInfo mAFI;
    
    int mAOS;
    void * mMapAOS;
    size_t mMapSize;
    size_t mMapOffset;
};

#endif /* AuxSeparator_h */
