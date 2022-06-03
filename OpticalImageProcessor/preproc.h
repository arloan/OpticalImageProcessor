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
#include <filesystem>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
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
                 const std::string rrcFile4MSSBand[MSS_BANDS]) :
    mPanFile(panFile),
    mMssFile(mssFile),
    mRrcPanFile(rrcFile4PAN) {

#ifdef DEBUG
        printf("PAN: %s\n", mPanFile.c_str());
        printf("MSS: %s\n", mMssFile.c_str());
#endif
        for (int i = 0; i < MSS_BANDS; ++i) {
            mRrcMssBndFile[i] = rrcFile4MSSBand[i];
        }
        
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
        for (size_t i = 0; i < mLinesMSS; ++i) {
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
        OLOG("Writing aligned MSS image as RAW file ...");
        
        auto saveFilePath = BuildOutputFilePath(mMssFile, ".IBCOR");
        WriteBufferToFile((const char *)mAlignedMSS.get(), mLinesMSS * PIXELS_PER_LINE * BYTES_PER_PIXEL, saveFilePath);
        
        OLOG("Written to file [%s].", saveFilePath.c_str());
    }
    
    void WriteAlignedMSS_TIFF() {
        OLOG("Writing aligned MSS image as TIFF file ...");
        
        auto saveFilePath = BuildOutputFilePath(mMssFile, ".ALIGNED", ".TIFF");
        cv::Mat imageData((int)mLinesMSS, PIXELS_PER_LINE, CV_16U, mAlignedMSS.get());
        if (!cv::imwrite(saveFilePath, imageData)) {
            throw std::runtime_error("Writing/converting MSS image as TIFF failed");
        }
        
        OLOG("Written to file [%s].", saveFilePath.c_str());
    }
    
    // Relative radiation correction
    void DoRRC() {
        if (mImagePAN.isNull()) throw std::logic_error("PAN raw image data not loaded, call `LoadPAN()' first");
        for (int b = 0; b < MSS_BANDS; ++b) {
            if (mImageBandMSS[b].isNull()) throw std::logic_error("MSS raw image data not loaded, call `LoadMSS()' first");
        }

        // 1. PAN image RRC
        OLOG("Begin inplace RRC for PAN data ... ");
        InplaceRRC(mImagePAN.get(), PIXELS_PER_LINE, (int)mLinesPAN, mRRCParamPAN);
        OLOG("RRC for PAN done.");
        
        // 2. MSS image RRC
        for (int i = 0; i < MSS_BANDS; ++i) {
            OLOG("Begin inplace RRC for MSS band %d ... ", i);
            InplaceRRC(mImageBandMSS[i].get(), PIXELS_PER_LINE / MSS_BANDS, (int)mLinesMSS, mRRCParamMSS[i]);
            OLOG("RRC done for MSS band %d.", i);
        }
    }
    
    void CalcInterBandCorrelation(int slices, bool autoUnloadPAN = true) {
        if (slices < MINIMUM_IBCSLCS) {
            throw std::invalid_argument(xs("CalcInterBandCorrelation: at lease %d slice needed", MINIMUM_IBCSLCS).s);
        }
        
        OLOG("Calculating inter-band correlation with %d slicing ...", slices);
        for (int b = 0; b < MSS_BANDS; ++b) {
            mBandShift[b] = new InterBandShift[slices];
        }

        int baseRows = std::min((int)mLinesPAN, CORRELATION_LINES);
        int baseRowGap = ((int)mLinesPAN - baseRows) / 2;
        int baseSliceCols = PIXELS_PER_LINE / slices;
        nc::NdArray<uint16_t> baseImage16U(mImagePAN.get(), (int)mLinesPAN, PIXELS_PER_LINE, false);
        for (int i = 0; i < slices; ++i)
        {
            auto baseSlice16U = baseImage16U(nc::Slice(baseRowGap, baseRowGap + baseRows), nc::Slice(i * baseSliceCols, (i + 1) * baseSliceCols));
            auto baseSlice32F = baseSlice16U.astype<float>();
            cv::Mat baseMat32F(baseRows, baseSliceCols, CV_32FC1, baseSlice32F.data());
            
            int bandRows = baseRows / MSS_BANDS;
            int bandRowGap = baseRowGap / MSS_BANDS;
            int bandSliceCols = baseSliceCols / MSS_BANDS;
            
            for (int b = 0; b < MSS_BANDS; ++b) {
                OLOG("Calculating inter-band correlation of BAND%d ...", b);
                nc::NdArray<uint16_t> band16U(mImageBandMSS[b].get(), (int)mLinesMSS, PIXELS_PER_LINE / MSS_BANDS, false);
                auto bandSlice16U = band16U(nc::Slice(bandRowGap, bandRowGap + bandRows),
                                            nc::Slice(i * bandSliceCols, (i + 1) * bandSliceCols));
                auto bandSlice32F = bandSlice16U.astype<float>();
                cv::Mat scaledBandSlice32F;
                cv::resize(cv::Mat(bandRows, bandSliceCols, CV_32FC1, bandSlice32F.data())
                           , scaledBandSlice32F
                           , cv::Size(0, 0)
                           , (double)MSS_BANDS
                           , (double)MSS_BANDS
                           , cv::INTER_CUBIC);
                
                double res = 0.0;
                cv::Point2d rv = cv::phaseCorrelate(baseMat32F, scaledBandSlice32F, cv::noArray(), &res);
                InterBandShift & shift = mBandShift[b][i];
                shift.dx = rv.x;
                shift.dy = rv.y;
                shift.rs = res;
                shift.cx = i * baseSliceCols + baseSliceCols / 2;
            }
        }
        
        OLOG("Inter-band correlation finished, try polynomial fitting ...");
        
        for (int b = 0; b < MSS_BANDS; ++b) {
            // x拟合为直线，y拟合为二次曲线
            OLOG("Doing polynomial fitting for BAND %d ...", b);
            std::vector<double> cxvals;
            std::vector<double> xvals;
            std::vector<double> yvals;
            for (int i = 0; i < slices; ++i) {
                const InterBandShift & ibs = mBandShift[b][i];
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
        OLOG("Polynomial fitting done.");
        
        if (autoUnloadPAN) {
            OLOG("Unloading PAN raw image data ...");
            UnloadPAN();
            OLOG("Unloaded.");
        }
        OLOG("CalcInterBandCorrelation(): done.");
    }
    
    void DoInterBandAlignment(bool autoUnloadRawMSS = true) {
        OLOG("Doing inter-band alignment ...");
        mAlignedMSS = new uint16_t[mLinesMSS * PIXELS_PER_LINE];
        int pixelPerBand = PIXELS_PER_LINE / MSS_BANDS;
        for (int x = 0; x < PIXELS_PER_LINE; ++x) {
            for (size_t y = 0; y < mLinesMSS; ++y) {
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
        if (autoUnloadRawMSS) {
            OLOG("Unloading MSS (unaligned & band-split) raw image data ...");
            UnloadMSS();
            OLOG("Unloaded.");
        }
        OLOG("DoInterBandAlignment(): done.");
    }
    
protected:
    std::string BuildOutputFilePath(const std::string & templatePath
                                    , const std::string & stemExtension
                                    , const char * replaceExtension = NULL) {
        auto cd = std::filesystem::current_path();
        std::filesystem::path tmplPath = templatePath;
        auto outputFilePath = cd / tmplPath.stem();
        outputFilePath += stemExtension;
        outputFilePath += replaceExtension ? replaceExtension : tmplPath.extension();
        return outputFilePath.string();
    }
    
//    static uint16_t * CopyVerticalSplitBufferSlice(const uint16_t * buffer, int w, int h, int slices, int sliceIndex) {
//        int sliceW = w / slices;
//        scoped_ptr<uint16_t> slice = new uint16_t[(size_t)sliceW * h];
//        
//        for (size_t i = 0; i < h; ++i) {
//            memcpy(slice.get() + i * sliceW, buffer + i * w + sliceIndex * sliceW, sliceW * sizeof(uint16_t));
//        }
//        
//        return slice.detach();
//    }
    
    void InplaceRRC(uint16_t * buff, int w, int h, const RRCParam * rrcParam) {
        for (size_t x = 0; x < w; ++x) {
            for (size_t y = 0; y < h; ++y) {
                size_t idx = w * y + x;
                uint16_t src = buff[idx];
                uint16_t dst = (uint16_t)(rrcParam[x].k * src + rrcParam[x].b);
                buff[idx] = dst;
            }
        }
    }
    
    void LoadRRCParamFiles() {
        int rrcLinesPAN = PIXELS_PER_LINE;
        int rrcLinesMSS = rrcLinesPAN / MSS_BANDS;
        mRRCParamPAN   = LoadRRCParamFile(mRrcPanFile.c_str(), rrcLinesPAN);
        for (int i = 0; i < MSS_BANDS; ++i) {
            mRRCParamMSS[i] = LoadRRCParamFile(mRrcMssBndFile[i].c_str(), rrcLinesMSS);
        }
#ifdef DEBUG
        printf("LoadRRCParamFiles(): OK.\n");
#endif
    }
    
    void CheckFilesAttributes() {
        struct stat st = { 0 };
        
        // PAN file
        OLOG("Checking PAN raw file attributes ...");
        if (stat(mPanFile.c_str(), &st)) throw errno_error("stat() call for PAN file failed");
        mSizePAN = st.st_size;
        mPixelsPAN = mSizePAN / BYTES_PER_PIXEL;
        mLinesPAN = mSizePAN / (PIXELS_PER_LINE * BYTES_PER_PIXEL);
        
        // MSS file
        OLOG("Checking PAN raw file attributes ...");
        memset(&st, 0, sizeof(struct stat));
        if (stat(mMssFile.c_str(), &st)) throw errno_error("stat() call for MSS file failed");
        mSizeMSS = st.st_size;
        mPixelsMSS = mSizeMSS / BYTES_PER_PIXEL;
        mLinesMSS = mSizeMSS / (PIXELS_PER_LINE * BYTES_PER_PIXEL);
        
        if (mSizePAN != MSS_BANDS * mSizeMSS)
            throw std::runtime_error(xs("PAN file size does not match MSS file size: PAN file should be %dx as large as MSS file",
                                        MSS_BANDS).s);
        if (mSizePAN % (PIXELS_PER_LINE * BYTES_PER_PIXEL) != 0)
            throw std::runtime_error(xs("PAN file size invalid: should be multiplies of %d", (PIXELS_PER_LINE * BYTES_PER_PIXEL)).s);

        OLOG("CheckFilesAttributes(): OK.");
    }
    
    static RRCParam * LoadRRCParamFile(const char * paramFilePath, int expectedLines) {
        OLOG("Loading RRC paramter from file `%s' ...", paramFilePath);
        scoped_ptr<FILE, FileDtor> f = fopen(paramFilePath, "rb");
        if (f.isNull()) throw errno_error("open RRC Param file failed");
        
        const int bn = 1024;
        char buff[bn];
        
        // first line is `1'
        if (fgets(buff, bn, f) == NULL) throw errno_error("LoadRRCParamFile([1]): read file content failed");
#ifdef DEBUG
        ToolBox::ChompChars(buff);
        assert(strcmp(buff, "1") == 0);
#endif
        
        // second line should be image lines
        if (fgets(buff, bn, f) == NULL) {
            throw errno_error("LoadRRCParamFile([2]): read file content failed");
        }
        int lines = atoi(buff);
        if (lines != expectedLines) {
            throw std::runtime_error(xs("LoadRRCParamFile([2]): expected %d lines while %d found in file content", expectedLines, lines).s);
        }
        
        // third line should be `0'
        if (fgets(buff, bn, f) == NULL) throw errno_error("LoadRRCParamFile([3]): read file content failed");
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
        if (f.isNull()) throw std::invalid_argument(xs("cannot open file [%s]: %d", filePath, errno).s);
        
        if (fseek(f, 0, SEEK_END)) throw std::invalid_argument(xs("ReadFileContent(): seek2end failed: %d", errno).s);
        off_t fsize = ftello(f);
        if (fseek(f, 0, SEEK_SET)) throw std::invalid_argument(xs("ReadFileContent(): rewind failed: %d", errno).s);
        
        scoped_ptr<char, new_dtor<char>> buff = new char[fsize];
        memset(buff, 0, fsize);
        const int unit = 8 * 1024 * 1024; // 8MB
        
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
    
    static size_t WriteBufferToFile(const char * buff, size_t size, const std::string & saveFilePath) {
        scoped_ptr<FILE, FileDtor> f = fopen(saveFilePath.c_str(), "wb");
        if (f.isNull()) throw std::runtime_error(xs("open file [%s] failed: %", saveFilePath.c_str(), errno).s);
        
        const int unit = (int)std::min((size_t)8 * 1024 * 1024, size); // 8MB
        size_t written = 0;
        for (const char * p = buff; written < size; ) {
            auto wb = fwrite(p, 1, unit, f);
            if (wb == 0) throw std::runtime_error(xs("write file failed: %d, %lld bytes written so far", errno, written).s);
            written += wb;
            p += wb;
        }
        return written;
    }
    
private:
    const std::string mPanFile;
    const std::string mMssFile;
    const std::string mRrcPanFile;
    std::string mRrcMssBndFile[MSS_BANDS];
    
    scoped_ptr<RRCParam> mRRCParamPAN;
    scoped_ptr<RRCParam> mRRCParamMSS[MSS_BANDS];

    size_t mSizePAN; // in Bytes
    size_t mSizeMSS; // in Bytes, all bands
    size_t mPixelsPAN;
    size_t mPixelsMSS; // all bands
    
    size_t mLinesPAN;
    size_t mLinesMSS;
    
    scoped_ptr<InterBandShift> mBandShift[MSS_BANDS];
    scoped_ptr<uint16_t> mImagePAN;
    scoped_ptr<uint16_t> mImageBandMSS[MSS_BANDS];
    scoped_ptr<uint16_t> mAlignedMSS;
    
    double mDeltaXcoeffs[MSS_BANDS][2];
    double mDeltaYcoeffs[MSS_BANDS][3];
};

END_NS

#endif /* preproc_h */
