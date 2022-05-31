//
//  preproc.h
//  OpticalImageProcessor
//
//  Created by Qiu PENG on 30/5/22.
//

#ifndef preproc_h
#define preproc_h

#include <stdio.h>
#include <sys/stat.h>

#include "oipshared.h"
#include "toolbox.h"
BEGIN_NS(OIP)

struct RRCParam {
    double k;
    double b;
};

class PreProcessor {
public:
    PreProcessor(const std::string & panFile,
                 const std::string & mssFile,
                 const std::string & rrcFile4PAN,
                 const std::string & rrcFile4MSSB1,
                 const std::string & rrcFile4MSSB2,
                 const std::string & rrcFile4MSSB3,
                 const std::string & rrcFile4MSSB4) :
    mPanFile(panFile),
    mMssFile(mssFile),
    mRrcPanFile(rrcFile4PAN),
    mRrcMssB1File(rrcFile4MSSB1),
    mRrcMssB2File(rrcFile4MSSB2),
    mRrcMssB3File(rrcFile4MSSB3),
    mRrcMssB4File(rrcFile4MSSB4) {
        
#ifdef DEBUG
        printf("PAN: %s\n", mPanFile.c_str());
        printf("MSS: %s\n", mMssFile.c_str());
#endif

        CheckFilesAttributes();
        LoadRRCParamFiles();
    }
    
    // Relative radiation correction
    void DoRRC() {
        // 1. PAN image RRC
        scoped_ptr<char> pan = ReadPAN();
        
        OLOG("Begin inplace RRC for PAN data ... ");
        InplaceRRC((uint16_t *)pan.get(), PIXELS_PER_LINE, mLinesPAN, mRRCParamPAN);
        OLOG("RRC for PAN done.");
        
        // 2. MSS image RRC
        char * tmpBands[4] = { 0 };
        ReadMSS(tmpBands);
        
        scoped_ptr<char> mssBands[MSS_BANDS];
        const RRCParam * rrcParamMssBands[MSS_BANDS] = {
            mRRCParamMSSB1.get(),
            mRRCParamMSSB2.get(),
            mRRCParamMSSB3.get(),
            mRRCParamMSSB4.get()
        };
        for (int i = 0; i < MSS_BANDS; ++i) {
            mssBands[i].attach(tmpBands[i]);
            
            OLOG("Begin inplace RRC for MSS band %d ... ", i);
            InplaceRRC((uint16_t *)tmpBands[i], PIXELS_PER_LINE / MSS_BANDS, mLinesMSS, rrcParamMssBands[i]);
            OLOG("RRC done for MSS band %d.", i);
        }
    }
    
protected:
    void InplaceRRC(uint16_t * buff, int w, int h, const RRCParam * rrcParam) {
        for (off_t x = 0; x < w; ++x) {
            for (off_t y = 0; y < h; ++y) {
                off_t idx = w * y + x;
                uint16_t src = buff[idx];
                uint16_t dst = (uint16_t)(rrcParam[x].k * src + rrcParam[x].b);
                buff[idx] = dst;
            }
        }
    }
    
    char * ReadPAN() {
        OLOG("Reading PAN raw file content (time consuming) from `%s' ...", mPanFile.c_str());
        off_t size = 0;
        char * data = ReadFileContent(mPanFile.c_str(), size);
        if (size != mSizePAN) {
            throw std::runtime_error(xs("PAN file size(%ld) doesn't match with read byte count(%ld)", mSizePAN, size).s);
        }
        OLOG("ReadPAN(): %lld bytes read.", size);
        return data;
    }
    
    void ReadMSS(char * mss[MSS_BANDS]) {
        OLOG("Reading MSS raw file content from `%s' ...", mMssFile.c_str());
        off_t size = 0;
        scoped_ptr<char> mssMixed = ReadFileContent(mMssFile.c_str(), size);
        if (size != mSizeMSS) {
            throw std::runtime_error(xs("MSS file size(%ld) doesn't match with read byte count(%ld)", mSizeMSS, size).s);
        }
        OLOG("ReadMSS(): %lld bytes read.", size);
        
        scoped_ptr<char> mssBands[MSS_BANDS];
        for (int i = 0; i < MSS_BANDS; ++i) {
            mssBands[i].attach(new char[mSizeMSS]);
        }
        
        // split MSS 4 bands
        OLOG("ReadMSS(): splitting %d bands ...", MSS_BANDS);
        for (int i = 0; i < mLinesMSS; ++i) {
            int lineWidth = PIXELS_PER_LINE * BYTES_PER_PIXEL;
            int bandWidth = lineWidth / MSS_BANDS;
            for (int b = 0; b < MSS_BANDS; ++b) {
                memcpy(mssBands[b].get() + i * bandWidth, mssMixed.get() + i * lineWidth + b * bandWidth, bandWidth);
            }
        }
        
        for (int i = 0; i < MSS_BANDS; ++i) {
            mss[i] = mssBands[i].detach();
        }
        OLOG("ReadMSS(): splitting OK.");
    }
    
    void LoadRRCParamFiles() {
        int rrcLinesPAN = PIXELS_PER_LINE;
        int rrcLinesMSS = rrcLinesPAN / MSS_BANDS;
        mRRCParamPAN   = LoadRRCParamFile(mRrcPanFile  .c_str(), rrcLinesPAN);
        mRRCParamMSSB1 = LoadRRCParamFile(mRrcMssB1File.c_str(), rrcLinesMSS);
        mRRCParamMSSB2 = LoadRRCParamFile(mRrcMssB2File.c_str(), rrcLinesMSS);
        mRRCParamMSSB3 = LoadRRCParamFile(mRrcMssB3File.c_str(), rrcLinesMSS);
        mRRCParamMSSB4 = LoadRRCParamFile(mRrcMssB4File.c_str(), rrcLinesMSS);
#ifdef DEBUG
        printf("LoadRRCParamFiles(): OK.\n");
#endif
    }
    
    void CheckFilesAttributes() {
        struct stat st = { 0 };
        
        // PAN file
        OLOG("Checking PAN raw file attributes ...");
        if (stat(mPanFile.c_str(), &st)) throw errno_error(xs("stat() call for PAN file failed: %d", errno));
        mSizePAN = st.st_size;
        mLinesPAN = (int)(mSizePAN / (PIXELS_PER_LINE * BYTES_PER_PIXEL));
        
        // MSS file
        OLOG("Checking PAN raw file attributes ...");
        memset(&st, 0, sizeof(struct stat));
        if (stat(mMssFile.c_str(), &st)) throw errno_error(xs("stat() call for MSS file failed: %d", errno));
        mSizeMSS = st.st_size;
        mLinesMSS = (int)(mSizeMSS / (PIXELS_PER_LINE * BYTES_PER_PIXEL));
        
        if (mSizePAN != 4 * mSizeMSS)
            throw std::runtime_error("PAN file size does not match MSS file size: PAN file should be 4x as large as MSS file");
        if (mSizePAN % (PIXELS_PER_LINE * BYTES_PER_PIXEL) != 0)
            throw std::runtime_error(xs("PAN file size invalid: should be multiplies of %d", (PIXELS_PER_LINE * BYTES_PER_PIXEL)).s);

        OLOG("CheckFilesAttributes(): OK.");
    }
    
    static RRCParam * LoadRRCParamFile(const char * paramFilePath, int expectedLines) {
        OLOG("Loading RRC paramter from file `%s' ...", paramFilePath);
        scoped_ptr<FILE, FileDtor> f = fopen(paramFilePath, "rb");
        if (f.isNull()) throw errno_error(xs("open RRC Param file failed: %d", errno));
        
        const int bn = 1024;
        char buff[bn];
        
        // first line is `1'
        if (fgets(buff, bn, f) == NULL) throw errno_error(xs("LoadRRCParamFile([1]): read file content failed: %d", errno));
#ifdef DEBUG
        ToolBox::ChompChars(buff);
        assert(strcmp(buff, "1") == 0);
#endif
        
        // second line should be image lines
        if (fgets(buff, bn, f) == NULL) {
            throw errno_error(xs("LoadRRCParamFile([2]): read file content failed: %d", errno));
        }
        int lines = atoi(buff);
        if (lines != expectedLines) {
            throw std::runtime_error(xs("LoadRRCParamFile([2]): expected %d lines while %d found in file content", expectedLines, lines).s);
        }
        
        // third line should be `0'
        if (fgets(buff, bn, f) == NULL) throw errno_error(xs("LoadRRCParamFile([3]): read file content failed: %d", errno));
#ifdef DEBUG
        ToolBox::ChompChars(buff);
        assert(strcmp(buff, "0") == 0);
#endif
        
        // following lines are of RRC parameter of k & b
        int index = 0;
        double k = .0;
        double b = .0;
        scoped_ptr<RRCParam, array_dtor<RRCParam>> params = new RRCParam[expectedLines];
        
        for (; fgets(buff, bn, f); ++index) {
            if (sscanf(buff, " %lf , %lf", &k, &b) != 2) {
                throw std::runtime_error(xs("line #%d of RRC param file [%s] found invalid", index, paramFilePath).s);
            }
            params[index].k = k;
            params[index].b = b;
        }
        
        if (index != expectedLines) {
            throw std::runtime_error(xs("RRC Param file [%s] invalid: %d lines of param expected, %d lines parsed."
                                        , paramFilePath, expectedLines, index).s);
        }

        OLOG("LoadRRCParamFile(): loaded.");
        return params.detach();
    }
    

    static char * ReadFileContent(const char * filePath, off_t & size) {
        if (filePath == nullptr) throw std::invalid_argument("invalid or empty file path");
        
        scoped_ptr<FILE, FileDtor> f = fopen(filePath, "rb");
        if (f.isNull()) throw errno_error(xs("cannot open file [%s]: %d", filePath, errno));
        
        if (fseek(f, 0, SEEK_END)) throw errno_error(xs("ReadFileContent(): seek2end failed: %d", errno));
        off_t fsize = ftello(f);
        if (fseek(f, 0, SEEK_SET)) throw errno_error(xs("ReadFileContent(): rewind failed: %d", errno));
        
        scoped_ptr<char, new_dtor<char>> buff = new char[fsize];
        memset(buff, 0, fsize);
        const int unit = 4 * 1024 * 1024; // 4MB
        
        for (char * p = buff;;) {
            auto rn = fread(p, 1, unit, f);
            p += rn;
            if (rn == 0) {
                size = p - buff;
                break;
            }
        }
        
        return buff.detach();
    }
    
private:
    const std::string mPanFile;
    const std::string mMssFile;
    const std::string mRrcPanFile;
    const std::string mRrcMssB1File;
    const std::string mRrcMssB2File;
    const std::string mRrcMssB3File;
    const std::string mRrcMssB4File;
    
    scoped_ptr<RRCParam, array_dtor<RRCParam>> mRRCParamPAN;
    scoped_ptr<RRCParam, array_dtor<RRCParam>> mRRCParamMSSB1;
    scoped_ptr<RRCParam, array_dtor<RRCParam>> mRRCParamMSSB2;
    scoped_ptr<RRCParam, array_dtor<RRCParam>> mRRCParamMSSB3;
    scoped_ptr<RRCParam, array_dtor<RRCParam>> mRRCParamMSSB4;

    off_t mSizePAN;
    off_t mSizeMSS;
    int mLinesPAN;
    int mLinesMSS;
};

END_NS

#endif /* preproc_h */
