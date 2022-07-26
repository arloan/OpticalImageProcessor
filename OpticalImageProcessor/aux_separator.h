//
//  AuxSeparator.h
//  AuxDataSeparator
//
//  Created by Stone PEN on 19/7/22.
//

#ifndef AuxSeparator_h
#define AuxSeparator_h

#include <string>
#include <deque>
#include <filesystem>
#include <future>
#include <mutex>

#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "oipshared.h"
#include "CRC.h"

#define REPORT_PER_COUNT    5000

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

#define AOS_FRAME_INVALID   -1
#define AOS_FRAME_EMPTY     0
#define AOS_FRAME_VALID     1


#define IMTR_SIG            "\x49\x54\xCE\x1F"
#define IMTR_SIG_BYTES      4
#define IMTR_FRAME_BYTES    882
#define IMTR_SEQ_OFF        4
#define IMTR_SEQ_BYTES      4
#define IMTR_CHID_OFF       8
#define IMTR_CHID_BYTES     1
#define IMTR_CHID_CMOS1     0x11
#define IMTR_CHID_CMOS2     0x22
#define IMTR_DTMARK_OFF     9
#define IMTR_DTMARK_BYTES   1
#define IMTR_DTMARK_IMG     0x22
#define IMTR_IMGDATA_OFF    10
#define IMTR_IMGDATA_BYTES  866
#define IMTR_CRC_OFF        876
#define IMTR_CRC_BITS       16
#define IMTR_ENDSIG         "\x2E\xE9\xC8\xFD"
#define IMTR_ENDSIG_OFF     878
#define IMTR_ENDSIG_BYTES   4

#define IMGSIG_SIG          "\xEB\x90\xE1\x4D"
#define IMGSIG_SIG_BYTES    4
#define IMGSIG_CAMERA_OFF   4
#define IMGSIG_CAMERA_BYTES 1
#define IMGSIG_CAM_ZRATIO(x) (((uint8_t)x) & 0x3F)
#define IMGSIG_ZRTO_NONE    0
#define IMGSIG_ZRTO_M4P4    0x11
#define IMGSIG_ZRTO_M4P8    0x12
#define IMGSIG_ZRTO_M4P16   0x13
#define IMGSIG_ZRTO_M8P8    0x22
#define IMGSIG_ZRTO_M8P4    0x21
#define IMGSIG_ZRTO_M8P16   0x23
#define IMGSIG_ZRTO_M16P16  0x33
#define IMGSIG_ZRTO_M16P4   0x31
#define IMGSIG_ZRTO_M16P8   0x32

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
    const uint8_t * data;
    const uint8_t * ldpc;
};

struct ImtrFrameInfo {
    uint8_t chid;
    uint16_t crc;
    uint32_t seq;
    const uint8_t * data;
};

class AuxSeparator
{
public:
    AuxSeparator(const std::string & aosFile, size_t offset = 0) :
    mAosFile(aosFile), mAOS(0), mMapAOS(nullptr), mMapSize(0), mMapOffset(offset)
    {
        InitializeCriticalSection(&mAosFrameLock);
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
        CleanAosMMap();
    }
    
public:
    void Separate(const char * outputDir = nullptr) {
        auto od = std::filesystem::current_path().string();
        if (outputDir != nullptr) {
            od = outputDir;
        }
        
        OLOG("Launching AOS file separation ...");
        auto t1 = std::thread(&AuxSeparator::SeparateAosFile, this, mAosFile, od);
        OLOG("Launching AOS frame parsering ...");
        auto t2 = std::thread(&AuxSeparator::DataTransFrameParser, this);
        
        t2.join();
        t1.join();
        OLOG("Done.");
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
        
        int invalid = 0;
        int empty = 0;
        int valid = 0;
        size_t remain = mMapSize;
        unsigned char * sb = (uint8_t *)SYNC_BYTES;
        OLOG("sync bytes: %02X%02X%02X%02X (%d bytes).", sb[0], sb[1], sb[2], sb[3], SYNC_BYTES_LEN);
        AosFrameInfo afi = { 0 };
        CriticalSectionLocker csl(mAosFrameLock);
        stop_watch sw;
        
        for (uint8_t * p = (uint8_t *)mMapAOS;;) {
            uint8_t * frame = NextAosFrame(p, remain);
            if (!frame) {
                OLOG("No further SYNC-BYTES found in remaining %s bytes of AOS file content.",
                     comma_sep(remain).sep());
                
                _ims_lock(CriticalSectionLocker, csl) {
                    mAosFrameData.push_back(nullptr);
                }
                break;
            }
            
            int fvr = ValidateAosFrame(frame, AOS_FRAME_BYTES, afi);
            if (fvr != AOS_FRAME_VALID) {
                if ((invalid + empty) % REPORT_PER_COUNT == 0) {
                    OLOG("%08d invalid or empty AOS frames found & ignored.", invalid + empty);
                }
                if (fvr == AOS_FRAME_INVALID) invalid++;
                if (fvr == AOS_FRAME_EMPTY) empty++;
                remain -= frame - p + SYNC_BYTES_LEN;
                p = frame + SYNC_BYTES_LEN;
                continue;
            }
            
            size_t mapOff = frame - (uint8_t *)mMapAOS;
            size_t fileOff = mapOff + mMapOffset;
            if (valid % REPORT_PER_COUNT == 0) {
                OLOG("Found valid AOS frame [#%08d] at byte offset of mmap: %lX (%s), of file: %lX (%s).",
                     valid, mapOff,
                     comma_sep(mapOff).sep(),
                     fileOff,
                     comma_sep(fileOff).sep());
                //DumpAosFrameInfo(afi);
            }
            valid++;
            remain -= frame - p + AOS_FRAME_BYTES;
            p = frame + AOS_FRAME_BYTES;
            _ims_lock(CriticalSectionLocker, csl) {
                mAosFrameData.push_back(afi.data);
            }
        }
        auto es = sw.tick().ellapsed;
        OLOG("%s bytes processed for AOS filemap in %s seconds (%s MBps).",
             comma_sep(mMapSize).sep(),
             comma_sep(es).sep(),
             comma_sep(mMapSize/es/(1024.0*1024.0)).sep());
    }
    
    int DataTransFrameParser() {
        uint8_t imtrFrame[IMTR_FRAME_BYTES];
        bool imtrFrameDirty = false;

        uint8_t imtrCache[IMTR_FRAME_BYTES * 2];
        int cacheBytes = 0;
        
        uint32_t lastImtrSeq = 0; // 1-based
        uint32_t count = 0;
        long total = 0;

        scoped_ptr<FILE, FileDtor> imdt = nullptr;
        CriticalSectionLocker csl(mAosFrameLock);
        stop_watch sw;
        
        for (;;) {
            const uint8_t * aosData = nullptr;
            if (!imtrFrameDirty) {
                if (cacheBytes < IMTR_FRAME_BYTES) {
                    for (; mAosFrameData.size() == 0;) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    _ims_lock(CriticalSectionLocker, csl) {
                        aosData = mAosFrameData.front();
                        mAosFrameData.pop_front();
                    }
                    if (aosData == nullptr) {
                        OLOG("No more AOS frame data, end of job.");
                        break;
                    }
                    memcpy(imtrCache + cacheBytes, aosData, AOS_DATA_BYTES);
                    cacheBytes += AOS_DATA_BYTES;
                    total++;
                    continue;
                }
                
                memcpy(imtrFrame, imtrCache, IMTR_FRAME_BYTES);
                imtrFrameDirty = true;
                cacheBytes -= IMTR_FRAME_BYTES;
                if (cacheBytes > 0) memmove(imtrCache, imtrCache + IMTR_FRAME_BYTES, cacheBytes);
                continue;
            }
            ImtrFrameInfo ifi;
            if (ValidateImtrFrame(imtrFrame, ifi)) {
                if (lastImtrSeq == 0) {
                    mIMDTFileName = xs("%s_%s_%s_%04d%02d%02d_%02d%02d%02d.IMDT",
                                       mAFI.station,
                                       mAFI.satellite,
                                       ifi.chid == IMTR_CHID_CMOS1 ? "CMOS-1" : "CMOS-2",
                                       mAFI.year,
                                       mAFI.month,
                                       mAFI.day,
                                       mAFI.hour,
                                       mAFI.minute,
                                       mAFI.second).s;
                    imdt.attach(fopen(mIMDTFileName.c_str(), "wb"));
                    if (imdt.get() == nullptr) {
                        throw errno_error("create intermediate IMDT file failed:");
                    }
                }
                
                if (lastImtrSeq + 1 != ifi.seq) {
                    // TODO: how to handle this situation?
                    OLOG("WARNING: missing or invalid image transfer frame(s) #%08d-%08d", lastImtrSeq+1, ifi.seq-1);
                }
                
                lastImtrSeq = ifi.seq;
                if (fwrite(ifi.data, IMTR_IMGDATA_BYTES, 1, imdt) == 0) {
                    throw errno_error("write intermediate IMDT file failed:");
                }
                if (count++ % REPORT_PER_COUNT == 0) {
                    OLOG("%s frames parsed & written.", comma_sep(count).sep());
                }
            }
            
            imtrFrameDirty = false;
        } // for
        
        auto es = sw.tick().ellapsed;
        auto totalBytes = total * AOS_DATA_BYTES;
        OLOG("%s bytes of image trans data written in %s seconds (%s MBps).",
             comma_sep(totalBytes).sep(),
             comma_sep(es).sep(),
             comma_sep(totalBytes/es/(1024.0*1024.0)).sep());

        CleanAosMMap();
        return 0;
    }
    
    bool ValidateImtrFrame(uint8_t * frame, ImtrFrameInfo & ifi) {
        if (memcmp(frame, IMTR_SIG, IMTR_SIG_BYTES) != 0) {
            OLOG("WARNING: image trans frame head signature not match, ignored.");
            return false;
        }
        if (memcmp(frame + IMTR_ENDSIG_OFF, IMTR_ENDSIG, IMTR_ENDSIG_BYTES) != 0) {
            OLOG("WARNING: image trans frame tail signature not match, ignored.");
            return false;
        }
        
        uint32_t seq = *((uint32_t *)(frame + IMTR_SEQ_OFF));
        seq = ntohl(seq); // 0-based
        uint8_t cmos = frame[IMTR_CHID_OFF];
        uint8_t dtsig = frame[IMTR_DTMARK_OFF];
        if (dtsig != IMTR_DTMARK_IMG) {
            OLOG("WARNING: not an image data frame #%08d: %02X", seq, dtsig);
            return false;
        }
        
        uint16_t crc = *(uint16_t *)(frame + IMTR_CRC_OFF);
        crc = ntohs(crc);
        uint16_t calcedCRC = CRC::Calculate(frame, IMTR_CRC_OFF, CRC::CRC_16_CCITTFALSE());
        if (calcedCRC != crc) {
            OLOG("WARNING: bad CRC -> in frame: %04X, Calculated: %04X.", crc, calcedCRC);
            return false;
        }
        
        ifi.chid = cmos;
        ifi.crc = crc;
        ifi.seq = seq;
        ifi.data = frame + IMTR_IMGDATA_OFF;
        return true;
    }
    
    void CleanAosMMap() {
        if (mMapAOS) munmap(mMapAOS, mMapSize);
        if (mAOS > 0) close(mAOS);
        mMapAOS = nullptr;
        mAOS = 0;
    }
    
protected:
    static uint8_t * NextAosFrame(uint8_t * p, size_t sz) {
        if (sz < AOS_FRAME_BYTES) return nullptr;
        return (uint8_t *)memmem(p, sz, SYNC_BYTES, SYNC_BYTES_LEN);
    }
    
    static int ValidateAosFrame(uint8_t * frame, int len, AosFrameInfo & afi) {
        uint8_t vcid = *(frame + AOS_VCID_OFF) & AOS_VCID_MASK;
        uint32_t vcduSeq = *(uint32_t *)(frame + AOS_VCDUSEQ_OFF - 1) & 0xFFFFFF00; // big-endian
        vcduSeq = ntohl(vcduSeq);
        uint32_t vcduInj = *(uint32_t *)(frame + AOS_VCDUINJ_OFF);
        vcduInj = ntohl(vcduInj);
        uint8_t * data = frame + AOS_DATA_OFF;
        uint16_t crc = *(uint16_t *)(frame + AOS_CRC_OFF);
        crc = ntohs(crc);
        
        afi.vcid = vcid;
        afi.crc = crc;
        afi.vcduSeq = vcduSeq;
        afi.vcduInj = vcduInj;
        afi.data = data;
        afi.ldpc = frame + AOS_LDPC_OFF;
        
        if (vcduInj != AOS_VCDUINJ_INVAL && vcduInj != AOS_VCDUINJ_VALID) return AOS_FRAME_INVALID;
        if (vcduInj == AOS_VCDUINJ_INVAL && vcid == AOS_VCID_EMPTY) return AOS_FRAME_EMPTY;
        
        // validate CRC
        uint16_t calcedCRC = CRC::Calculate(frame + AOS_HEADER_OFF,
                                            AOS_HEADER_BYTES + AOS_VCDUINJ_BYTES + AOS_DATA_BYTES,
                                            CRC::CRC_16_CCITTFALSE());
        
        if (calcedCRC != crc) {
            OLOG("CRC in frame: %04X, Calculated: %04X.", crc, calcedCRC);
            return AOS_FRAME_INVALID;
        }
        
        // todo: validate LDPC
        return AOS_FRAME_VALID;
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
        OLOG("VCID: 0x%02X", afi.vcid);
        OLOG("VCDU SEQ: %u(0x%06X)", afi.vcduSeq, afi.vcduSeq);
        OLOG("VCDU INJ: 0x%08X", afi.vcduInj);
        OLOG("CRC: 0x%04X", afi.crc);
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
    std::deque<const uint8_t *> mAosFrameData;
    CRITICAL_SECTION mAosFrameLock;
    std::string mIMDTFileName;
    
    int mAOS;
    void * mMapAOS;
    size_t mMapSize;
    size_t mMapOffset;
};

#endif /* AuxSeparator_h */
