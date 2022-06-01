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
#include <algorithm>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <NumCpp/Polynomial/Poly1d.hpp>

#include "oipshared.h"
#include "toolbox.h"
BEGIN_NS(OIP)

struct RRCParam {
    double k;
    double b;
};
struct InterBandShift {
    double dx;
    double dy;
    double rs;
    int cx; // center-x: in pixel
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
    
    void LoadPAN() {
        OLOG("Reading PAN raw file content from `%s' ...", mPanFile.c_str());
        off_t size = 0;
        mImagePAN = (uint16_t *)ReadFileContent(mPanFile.c_str(), size);
        if (size != mSizePAN) {
            throw std::runtime_error(xs("PAN file size(%lld) doesn't match with read byte count(%lld)", mSizePAN, size).s);
        }
        OLOG("ReadPAN(): %lld bytes read.", size);
    }
    
    void LoadMSS() {
        OLOG("Reading MSS raw file content from `%s' ...", mMssFile.c_str());
        
        off_t size = 0;
        scoped_ptr<uint16_t> mssMixed = (uint16_t *)ReadFileContent(mMssFile.c_str(), size);
        if (size != mSizeMSS) {
            throw std::runtime_error(xs("MSS file size(%lld) doesn't match with read byte count(%lld)", mSizeMSS, size).s);
        }
        OLOG("ReadMSS(): %lld bytes read.", size);
        
        // split MSS 4 bands
        OLOG("ReadMSS(): splitting %d bands ...", MSS_BANDS);
        int lineBytes = PIXELS_PER_LINE * BYTES_PER_PIXEL;
        int bandBytesPerLine = lineBytes / MSS_BANDS;
        int bandPixelsPerLine = PIXELS_PER_LINE / MSS_BANDS; // MSS_BANDS: should always divisible by PIXELS_PER_LINE
        for (int i = 0; i < MSS_BANDS; ++i) {
            mImageBandMSS[i].attach(new uint16_t[mPixelsMSS / MSS_BANDS]); // obsolte: `+1' for in case of indivisible
        }
        for (int i = 0; i < mLinesMSS; ++i) {
            for (int b = 0; b < MSS_BANDS; ++b) {
                memcpy(mImageBandMSS[b].get() + i * PIXELS_PER_LINE,
                       mssMixed.get() + i * PIXELS_PER_LINE + b * bandPixelsPerLine,
                       bandBytesPerLine);
            }
        }
        
        OLOG("ReadMSS(): splitting OK.");
    }
    
    void UnloadPAN() {
        mImagePAN.attach(NULL);
    }
    void UnloadMSS() {
        for (int i = 0; i < MSS_BANDS; ++i) mImageBandMSS[i].attach(NULL);
    }
    void FreeAlignedMSS() {
        mAlignedMSS.attach(NULL);
    }
    
    void WriteAlignedMSS_RAW() {
        // TODO:
    }
    
    void WriteAlignedMSS_TIFF() {
        // TODO:
    }
    
    // Relative radiation correction
    void DoRRC() {
        if (mImagePAN.isNull()) throw std::logic_error("PAN raw image data not loaded, call `LoadPAN()' first");
        for (int b = 0; b < MSS_BANDS; ++b) {
            if (mImageBandMSS[b].isNull()) throw std::logic_error("MSS raw image data not loaded, call `LoadMSS()' first");
        }

        // 1. PAN image RRC
        OLOG("Begin inplace RRC for PAN data ... ");
        InplaceRRC(mImagePAN.get(), PIXELS_PER_LINE, mLinesPAN, mRRCParamPAN);
        OLOG("RRC for PAN done.");
        
        // 2. MSS image RRC
        const RRCParam * rrcParamMssBands[MSS_BANDS] = {
            mRRCParamMSSB1.get(),
            mRRCParamMSSB2.get(),
            mRRCParamMSSB3.get(),
            mRRCParamMSSB4.get()
        };
        for (int i = 0; i < MSS_BANDS; ++i) {
            OLOG("Begin inplace RRC for MSS band %d ... ", i);
            InplaceRRC(mImageBandMSS[i].get(), PIXELS_PER_LINE / MSS_BANDS, mLinesMSS, rrcParamMssBands[i]);
            OLOG("RRC done for MSS band %d.", i);
        }
    }
    
    void CalcInterBandCorrelation(int slices, bool autoUnloadPAN = true) {
        if (slices <= 0) {
            throw std::invalid_argument("CalcInterBandCorrelation: at lease 1 slice needed");
        }
        
        OLOG("Calculating inter-band correlation with %d slicing ...", slices);

        int baseRows = std::min(mLinesPAN, CORRELATION_LINES);
        cv::Mat baseImage(baseRows, PIXELS_PER_LINE / slices, CV_16U, mImagePAN.get() + PIXELS_PER_LINE * (mLinesPAN - baseRows) / 2);
        scoped_ptr<uint16_t> scaledMssBuffer = new uint16_t[(size_t)PIXELS_PER_LINE * mLinesPAN];
        scoped_ptr<InterBandShift> bandShifts[MSS_BANDS];
        for (int b = 0; b < MSS_BANDS; ++b) {
            OLOG("Calculating inter-band correlation of BAND%d ...", b);
            bandShifts[b] = CalcInterBandCorrelation(baseImage, scaledMssBuffer, b, slices);
            
            // x拟合为直线，y拟合为二次曲线
            OLOG("Doing polynomial fitting for BAND %d ...", b);
            std::vector<double> cxvals;
            std::vector<double> xvals;
            std::vector<double> yvals;
            for (int i = 0; i < slices; ++i) {
                const InterBandShift & ibs = bandShifts[b][i];
                cxvals.push_back((double)ibs.cx);
                xvals.push_back(ibs.dx);
                yvals.push_back(ibs.dy);
            }
            nc::NdArray<double> cxValues(cxvals);
            nc::NdArray<double> dXvalues(xvals);
            nc::NdArray<double> dYvalues(yvals);
            auto poly1dX = nc::polynomial::Poly1d<double>::fit(cxValues, dXvalues, 1);
            auto poly1dY = nc::polynomial::Poly1d<double>::fit(cxValues, dYvalues, 2);
            auto coeffsX = poly1dX.coefficients();
            auto coeffsY = poly1dY.coefficients();
            
            mDeltaXcoeffs[b][0] = coeffsX[0];
            mDeltaXcoeffs[b][1] = coeffsX[1];
            mDeltaYcoeffs[b][0] = coeffsY[0];
            mDeltaYcoeffs[b][1] = coeffsY[1];
            mDeltaYcoeffs[b][2] = coeffsY[2];
        }
        
        if (autoUnloadPAN) {
            OLOG("Unloading PAN raw image data ...");
            UnloadPAN();
            OLOG("Unloaded.");
        }
        
        OLOG("CalcInterBandCorrelation(): done.");
    }
    
    void DoInterBandAlignment() {
        OLOG("Doing inter-band alignment ...");
        mAlignedMSS = new uint16_t[(size_t)mLinesMSS * PIXELS_PER_LINE];
        int pixelPerBand = PIXELS_PER_LINE / MSS_BANDS;
        for (int x = 0; x < PIXELS_PER_LINE; ++x) {
            for (int y = 0; y < mLinesMSS; ++y) {
                int band = x / pixelPerBand;
                int xInBand = x % pixelPerBand;
                uint16_t * bandImage = mImageBandMSS[band];
                
                auto coeffX = mDeltaXcoeffs[band];
                auto coeffY = mDeltaYcoeffs[band];
                double deltaX = coeffX[1] * xInBand + coeffX[0];
                double deltaY = coeffY[2] * xInBand * xInBand + coeffY[1] * xInBand + coeffY[0];
                int mapx = (int)(xInBand + deltaX);
                int mapy = (int)(y + deltaY);
                
                size_t destIdx = y * PIXELS_PER_LINE + x;
                size_t srcIdx = mapy * pixelPerBand + mapx;
                mAlignedMSS[destIdx] = bandImage[srcIdx];
            }
        }
        OLOG("DoInterBandAlignment(): done.");
    }
    
protected:
    InterBandShift * CalcInterBandCorrelation(const cv::Mat& baseImage, uint16_t * scaledBandBuffer, int bandIndex, int slices) {
        uint16_t * bandImage = mImageBandMSS[bandIndex];
        scoped_ptr<uint16_t> scaled = UpscaleMssBand(bandImage, PIXELS_PER_LINE/MSS_BANDS, mLinesMSS);
        
        scoped_ptr<InterBandShift> bandShifts = new InterBandShift[slices];
        for (int i = 0; i < slices; ++i) {
            scoped_ptr<uint16_t> slice = CopyVerticalSplittedBufferSlice(scaled, PIXELS_PER_LINE, mLinesPAN, slices, i);
            cv::Mat comparingImage(baseImage.rows, baseImage.cols, CV_16U, slice.get() + PIXELS_PER_LINE * (mLinesPAN - baseImage.rows) / 2);
            double res = 0.0;
            cv::Point2d rv = cv::phaseCorrelate(baseImage, comparingImage, cv::noArray(), &res);
            InterBandShift & shift = bandShifts[i];
            shift.dx = rv.x;
            shift.dy = rv.y;
            shift.rs = res;
            shift.cx = i * baseImage.cols + baseImage.cols / 2;
        }
        return bandShifts.detach();
    }
    
    static uint16_t * CopyVerticalSplittedBufferSlice(const uint16_t * buffer, int w, int h, int slices, int sliceIndex) {
        int sliceW = w / slices;
        scoped_ptr<uint16_t> slice = new uint16_t[(size_t)sliceW * h];
        
        for (int i = 0; i < h; ++i) {
            memcpy(slice.get() + i * sliceW, buffer + i * w + sliceIndex * sliceW, sliceW * sizeof(uint16_t));
        }
        
        return slice.detach();
    }
    
    uint16_t * UpscaleMssBand(uint16_t * band, int w, int h, int scaleX = 4, int scaleY = 4) {
        scoped_ptr<uint16_t> scaled = new uint16_t[(size_t)w * scaleX * h * scaleY];
        
        cv::Mat bandImage(mLinesMSS, PIXELS_PER_LINE/MSS_BANDS, CV_16U, band);
        cv::Mat scaledImage(mLinesPAN, PIXELS_PER_LINE, CV_16U, scaled.get());
        cv::resize(bandImage, scaledImage, cv::Size(0, 0), scaleX, scaleY, cv::INTER_CUBIC);
        
        return scaled.detach();
    }
    
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
        mPixelsPAN = mSizePAN / BYTES_PER_PIXEL;
        mLinesPAN = (int)(mSizePAN / (PIXELS_PER_LINE * BYTES_PER_PIXEL));
        
        // MSS file
        OLOG("Checking PAN raw file attributes ...");
        memset(&st, 0, sizeof(struct stat));
        if (stat(mMssFile.c_str(), &st)) throw errno_error(xs("stat() call for MSS file failed: %d", errno));
        mSizeMSS = st.st_size;
        mPixelsMSS = mSizeMSS / BYTES_PER_PIXEL;
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

    off_t mSizePAN; // in Bytes
    off_t mSizeMSS; // in Bytes, all bands
    off_t mPixelsPAN;
    off_t mPixelsMSS; // all bands
    
    int mLinesPAN;
    int mLinesMSS;
    
    InterBandShift mBandShift[MSS_BANDS];
    scoped_ptr<uint16_t> mImagePAN;
    scoped_ptr<uint16_t> mImageBandMSS[MSS_BANDS];
    scoped_ptr<uint16_t> mAlignedMSS;
    
    double mDeltaXcoeffs[MSS_BANDS][2];
    double mDeltaYcoeffs[MSS_BANDS][3];
};

END_NS

#endif /* preproc_h */
