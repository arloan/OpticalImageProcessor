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
        stop_watch::rst();
        mImagePAN = (uint16_t *)ReadFileContent(mPanFile.c_str(), size);
        if (size != mSizePAN) {
            throw std::runtime_error(xs("PAN file size(%lld) doesn't match with read byte count(%lld)", mSizePAN, size).s);
        }
        auto es = stop_watch::tik().ellapsed;
        OLOG("ReadPAN(): %s bytes read in %s seconds (%s MBps).",
             comma_sep(size).sep(),
             comma_sep(es).sep(),
             comma_sep(size/es/1024.0/1024.0).sep());
    }
    
    void LoadMSS() {
        OLOG("Reading MSS raw file content from `%s' ...", mMssFile.c_str());
        
        off_t size = 0;
        stop_watch::rst();
        scoped_ptr<uint16_t> mssMixed = (uint16_t *)ReadFileContent(mMssFile.c_str(), size);
        if (size != mSizeMSS) {
            throw std::runtime_error(xs("MSS file size(%lld) doesn't match with read byte count(%lld)", mSizeMSS, size).s);
        }
        auto es = stop_watch::tik().ellapsed;
        OLOG("ReadMSS(): %s bytes read in %s seconds (%s MBps).",
             comma_sep(size).sep(),
             comma_sep(es).sep(),
             comma_sep(size/es/1024.0/1024.0).sep());
        
        // split MSS 4 bands
        OLOG("ReadMSS(): splitting %d bands ...", MSS_BANDS);
        int bandBytesPerLine = PIXELS_PER_LINE * BYTES_PER_PIXEL / MSS_BANDS;
        int bandPixelsPerLine = PIXELS_PER_LINE / MSS_BANDS; // MSS_BANDS: should always divisible by PIXELS_PER_LINE
        for (int i = 0; i < MSS_BANDS; ++i) {
            mImageBandMSS[i].attach(new uint16_t[mSizeMSS / MSS_BANDS]); // obsolte: `+1' for in case of indivisible
        }
        
        stop_watch::rst();
        for (size_t i = 0; i < mLinesMSS; ++i) {
            for (int b = 0; b < MSS_BANDS; ++b) {
                memcpy(mImageBandMSS[b].get() + i * bandPixelsPerLine,
                       mssMixed.get() + i * PIXELS_PER_LINE + b * bandPixelsPerLine,
                       bandBytesPerLine);
            }
        }
        es = stop_watch::tik().ellapsed;
        OLOG("ReadMSS(): split done in %s seconds (%s MBps).",
             comma_sep(es).sep(),
             comma_sep(size/es/1024.0/1024.0).sep());
    }
    
    void UnloadPAN() {
        mImagePAN.attach(NULL);
    }
    void UnloadMSS() {
        for (int i = 0; i < MSS_BANDS; ++i) mImageBandMSS[i].attach(NULL);
    }
    void FreeAlignedMSS() {
        //mAlignedMSS.attach(NULL);
        mAlignedMSS.release();
    }
    
    void WriteRRCedPAN() {
        OLOG("Writing RRC-ed PAN image as RAW file ...");
        
        auto saveFilePath = BuildOutputFilePath(mPanFile, ".RRC");
        stop_watch::rst();
        WriteBufferToFile((const char *)mImagePAN.get(), mSizePAN, saveFilePath);
        auto es = stop_watch::tik().ellapsed;
        
        OLOG("Written to file [%s].", saveFilePath.c_str());
        OLOG("Writing cost %s seconds (%s MBps).",
             comma_sep(es).sep(),
             comma_sep(mSizePAN/es/1024.0/1024.0).sep());
    }
    
    void WriteRRCedMSS() {
        OLOG("Writing RRC-ed PAN image as RAW file ...");
        
        for (int b = 0; b < MSS_BANDS; ++b) {
            auto saveFilePath = BuildOutputFilePath(mMssFile, xs(".RRCB%d", b));
            stop_watch::rst();
            WriteBufferToFile((const char *)mImageBandMSS[b].get(), mSizeMSS / MSS_BANDS, saveFilePath);
            auto es = stop_watch::tik().ellapsed;
            
            OLOG("Written to file [%s].", saveFilePath.c_str());
            OLOG("Writing cost %s seconds (%s MBps).",
                 comma_sep(es).sep(),
                 comma_sep(mSizeMSS/MSS_BANDS/es/1024.0/1024.0).sep());
        }
    }
    
    void WriteAlignedMSS_RAW() {
        OLOG("Writing aligned MSS image as RAW file ...");
        
        auto saveFilePath = BuildOutputFilePath(mMssFile, ".IBCOR");
        stop_watch::rst();
        //WriteBufferToFile((const char *)mAlignedMSS.get(), mSizeMSS, saveFilePath);
        WriteBufferToFile((const char *)mAlignedMSS.data, mSizeMSS, saveFilePath);
        auto es = stop_watch::tik().ellapsed;
        
        OLOG("Written to file [%s].", saveFilePath.c_str());
        OLOG("Writing cost %s seconds (%s MBps).",
             comma_sep(es).sep(),
             comma_sep(mSizeMSS/es/1024.0/1024.0).sep());
    }
    
    void WriteAlignedMSS_TIFF() {
        OLOG("Writing aligned MSS image as TIFF file ...");
        
        auto saveFilePath = BuildOutputFilePath(mMssFile, ".ALIGNED", ".TIFF");
        // cv::Mat imageData((int)mLinesMSS, PIXELS_PER_LINE, CV_16U, mAlignedMSS.get());
        // cv::Mat imageData(rows, PIXELS_PER_LINE, CV_16U, mAlignedMSS.get());
        cv::Mat & imageData = mAlignedMSS;
        
        stop_watch::rst();
        if (!cv::imwrite(saveFilePath, imageData)) {
            throw std::runtime_error("Writing/converting MSS image as TIFF failed");
        }
        auto es = stop_watch::tik().ellapsed;
        
        OLOG("Written to file [%s].", saveFilePath.c_str());
        OLOG("Writing cost %s seconds (%s MBps).",
             comma_sep(es).sep(),
             comma_sep(imageData.elemSize()*imageData.total()/es/1024.0/1024.0).sep());
    }
    
    // Relative radiation correction
    void DoRRC() {
        if (mImagePAN.isNull()) throw std::logic_error("PAN raw image data not loaded, call `LoadPAN()' first");
        for (int b = 0; b < MSS_BANDS; ++b) {
            if (mImageBandMSS[b].isNull()) throw std::logic_error("MSS raw image data not loaded, call `LoadMSS()' first");
        }

        // 1. PAN image RRC
        OLOG("Begin inplace RRC for PAN data ... ");
        stop_watch::rst();
        InplaceRRC(mImagePAN.get(), PIXELS_PER_LINE, (int)mLinesPAN, mRRCParamPAN);
        auto es = stop_watch::tik().ellapsed;
        OLOG("RRC for PAN done in %s seconds (%s MBps).",
             comma_sep(es).sep(),
             comma_sep(mSizePAN/es/1024.0/1024.0).sep());

        // 2. MSS image RRC
        for (int i = 0; i < MSS_BANDS; ++i) {
            OLOG("Begin inplace RRC for MSS band %d ... ", i);
            stop_watch::rst();
            InplaceRRC(mImageBandMSS[i].get(), PIXELS_PER_LINE / MSS_BANDS, (int)mLinesMSS, mRRCParamMSS[i]);
            es = stop_watch::tik().ellapsed;
            OLOG("RRC done for MSS band %d in %s seconds (%s MBps).",
                 i,
                 comma_sep(es).sep(),
                 comma_sep(mSizeMSS/MSS_BANDS/es/1024.0/1024.0).sep());
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

        double es = 0.0;
        int baseRows = std::min((int)mLinesPAN, CORRELATION_LINES);
        int baseRowGap = ((int)mLinesPAN - baseRows) / 2;
        int baseSliceCols = PIXELS_PER_LINE / slices;
        size_t sliceBytes = (size_t)baseRows * baseSliceCols * BYTES_PER_PIXEL;
        nc::NdArray<uint16_t> baseImage16U(mImagePAN.get(), (int)mLinesPAN, PIXELS_PER_LINE, false);
        for (int i = 0; i < slices; ++i)
        {
            OLOG("Extracting #%d slice from PAN image as base slice ...", i);
            stop_watch::rst();
            auto baseSlice16U = baseImage16U(nc::Slice(baseRowGap, baseRowGap + baseRows), nc::Slice(i * baseSliceCols, (i + 1) * baseSliceCols));
            es = stop_watch::tik().ellapsed;
            OLOG("Extraction done in %s seconds (%s MBps).",
                 comma_sep(es).sep(),
                 comma_sep(sliceBytes/es/1024.0/1024.0).sep());

            stop_watch::rst();
            auto baseSlice32F = baseSlice16U.astype<float>();
            es = stop_watch::tik().ellapsed;
            OLOG("Converting base slice from Uint16 to Float32 elements in %s seconds (%s MBps).",
                 comma_sep(es).sep(),
                 comma_sep(sliceBytes/es/1024.0/1024.0).sep());

            cv::Mat baseMat32F(baseRows, baseSliceCols, CV_32FC1, baseSlice32F.data());
            
            int bandRows = baseRows / MSS_BANDS;
            int bandRowGap = baseRowGap / MSS_BANDS;
            int bandSliceCols = baseSliceCols / MSS_BANDS;
            size_t bandSliceBytes = sliceBytes / MSS_BANDS / MSS_BANDS;
            for (int b = 0; b < MSS_BANDS; ++b) {
                OLOG("Calculating inter-band correlation of BAND%d ...", b);
                nc::NdArray<uint16_t> band16U(mImageBandMSS[b].get(), (int)mLinesMSS, PIXELS_PER_LINE / MSS_BANDS, false);
                
                OLOG("Extracting #%d slice from #%d band of MSS image ...", i, b);
                stop_watch::rst();
                auto bandSlice16U = band16U(nc::Slice(bandRowGap, bandRowGap + bandRows),
                                            nc::Slice(i * bandSliceCols, (i + 1) * bandSliceCols));
                es = stop_watch::tik().ellapsed;
                OLOG("Extraction done in %s seconds (%s MBps).",
                     comma_sep(es).sep(),
                     comma_sep(bandSliceBytes/es/1024.0/1024.0).sep());

                stop_watch::rst();
                auto bandSlice32F = bandSlice16U.astype<float>();
                es = stop_watch::tik().ellapsed;
                OLOG("Converting base slice from Uint16 to Float32 elements in %s seconds (%s MBps).",
                     comma_sep(es).sep(),
                     comma_sep(bandSliceBytes/es/1024.0/1024.0).sep());
                
                OLOG("Upscaling slice of MSS band image to the size of base slice image ...");
                cv::Mat scaledBandSlice32F;
                stop_watch::rst();
                cv::resize(cv::Mat(bandRows, bandSliceCols, CV_32FC1, bandSlice32F.data())
                           , scaledBandSlice32F
                           , cv::Size(0, 0)
                           , (double)MSS_BANDS
                           , (double)MSS_BANDS
                           , cv::INTER_CUBIC);
                es = stop_watch::tik().ellapsed;
                OLOG("Upscaling done in %s seconds (%s MBps).",
                     comma_sep(es).sep(),
                     comma_sep(bandSliceBytes/es/1024.0/1024.0).sep());

                OLOG("Calculating phase correlation of slice #%d for band #%d ...", i, b);
                double res = 0.0;
                stop_watch::rst();
                cv::Point2d rv = cv::phaseCorrelate(baseMat32F, scaledBandSlice32F, cv::noArray(), &res);
                es = stop_watch::tik().ellapsed;
                OLOG("Calculating done in %s seconds (%s MBps).",
                     comma_sep(es).sep(),
                     comma_sep(sliceBytes/es/1024.0/1024.0).sep());
                
                InterBandShift & shift = mBandShift[b][i];
                shift.dx = rv.x;
                shift.dy = rv.y;
                shift.rs = res;
                shift.cx = i * baseSliceCols + baseSliceCols / 2;
            }
        }
        
        OLOG("Inter-band correlation finished, result:");
        DumpInterBandShiftValues(slices);
        OLOG("Filter invalid correlation values, result:");
        FilterInterBandShiftValues(slices);
        DumpInterBandShiftValues(slices);

        OLOG("Try polynomial fitting ...");
        DoCorrelationPolynomialFitting(slices);
        OLOG("Polynomial fitting done.");
        
        OLOG("CalcInterBandCorrelation(): done.");
        if (autoUnloadPAN) {
            OLOG("Unloading PAN raw image data ...");
            UnloadPAN();
            OLOG("Unloaded.");
        }
    }
    
    // TODO: opencv::remap does not support image data larger than 32767*32767, need further handling
    // processing portion by portion of input image for now.
    void DoInterBandAlignment(int linePerSection, int lineOffset = 0,
                              int sectionOverlap = IBPA_DEFAULT_LINEOVERLAP,
                              bool autoUnloadRawMSS = true) {
        if (sectionOverlap > IBPA_MAX_LINEOVERLAP) {
            throw std::invalid_argument(xs("Overlap value %d exceeds maximum allowed value(%d)"
                                           , sectionOverlap, IBPA_MAX_LINEOVERLAP).s);
        }
        if (linePerSection > 32767) {
            throw std::invalid_argument("Row number exceeds OpenCV allowd value");
        }
        if (linePerSection < sectionOverlap * 2) {
            throw std::invalid_argument("Lines per section too small or section overlapped lines too large");
        }
        if (mLinesMSS - lineOffset < IBPA_MIN_PROCESSLINES) {
            throw std::invalid_argument("Too few image lines left to process");
        }
        
        OLOG("Doing inter-band alignment ...");
        
        size_t bytes = 0;
        size_t offset = lineOffset;
        int processedLines = 0;
        int sections = (int)((mLinesMSS - lineOffset) / (linePerSection - sectionOverlap)) + 1;
        mAlignedMSS.create((int)(mLinesMSS - lineOffset - sectionOverlap), PIXELS_PER_MSSBAND, CV_16UC4);
        stop_watch sw;
        
        for (int i = 0; ; ++i) {
            auto lines = std::min(mLinesMSS - offset, (size_t)linePerSection);
            if (mLinesMSS < offset || lines < IBPA_MIN_PROCESSLINES) break;
            
            OLOG("[SEC%d] %s lines for processing [offset=%s]."
                 , i+1
                 , comma_sep(lines).sep()
                 , comma_sep(offset).sep());
            
            
            OLOG("Doing inter-band alignment of section %d/%d ...", i+1, sections);
            cv::Mat sectionMat = DoInterBandAlignment(offset, (int)lines);
            OLOG("Copying to final image ...");
            memcpy(mAlignedMSS.ptr(processedLines),
                   sectionMat.ptr(sectionOverlap),
                   (size_t)(lines - sectionOverlap) * PIXELS_PER_MSSBAND * sectionMat.elemSize());
            OLOG("Copied.");
            
            processedLines += lines - sectionOverlap;
            bytes += (size_t)lines * PIXELS_PER_MSSBAND * BYTES_PER_PIXEL;
            offset += linePerSection - sectionOverlap;
        }
        
        auto es = sw.tick().ellapsed;
        OLOG("Alignment done in %s seconds (%s MBps).",
             comma_sep(es).sep(),
             comma_sep(bytes/es/1024.0/1024.0).sep());
        
        OLOG("Outputing aligned TIFF image ...");
        WriteAlignedMSS_TIFF();
        OLOG("Output done.");

        if (autoUnloadRawMSS) {
            OLOG("Unloading MSS (unaligned & band-split) raw image data ...");
            UnloadMSS();
            OLOG("Unloaded.");
        }
        OLOG("DoInterBandAlignment(): done.");
    }
    
protected:
    cv::Mat DoInterBandAlignment(size_t rowOffset, int rows) {
        cv::Mat alignedBands[MSS_BANDS];
        scoped_ptr<float> mapX = new float[PIXELS_PER_MSSBAND * rows];
        scoped_ptr<float> mapY = new float[PIXELS_PER_MSSBAND * rows];
        
        for (int b = 0; b < MSS_BANDS; ++b) {
            double * coeffX = mDeltaXcoeffs[b];
            double * coeffY = mDeltaYcoeffs[b];
            
            OLOG("[BAND#%d] creating mapX & mapY matrix ...", b);
            // (x, y) -> coordinates of MSS BAND sized image
            // polinomial coeffective values are of PAN size image
            // take (x', y') for coordinates of PAN, then x' = 4x, y' = 4y when BANDS=4
            // mapX(x,y) = mapX'(x',y')/4, mapY(x,y) = mapY'(x',y')/4
            // for (size_t y = 0; y < mLinesMSS; ++y) {
            for (size_t y = 0; y < rows; ++y) {
                for (int x = 0; x < PIXELS_PER_MSSBAND; ++x) {
                    auto yy = y * MSS_BANDS;
                    auto xx = x * MSS_BANDS;
                    mapX[y * PIXELS_PER_MSSBAND + x] = (float)((coeffX[1] * xx + coeffX[0] + xx)/MSS_BANDS);
                    mapY[y * PIXELS_PER_MSSBAND + x] = (float)((coeffY[2] * xx * xx + coeffY[1] * xx + coeffY[0] + yy)/MSS_BANDS);
                }
            }
            
            OLOG("[BAND#%d] remapping band image ...", b);
            cv::remap(cv::Mat(rows, PIXELS_PER_MSSBAND, CV_16U, mImageBandMSS[b].get() + rowOffset * PIXELS_PER_MSSBAND),
                      alignedBands[b],
                      cv::Mat(rows, PIXELS_PER_MSSBAND, CV_32FC1, mapX),
                      cv::Mat(rows, PIXELS_PER_MSSBAND, CV_32FC1, mapY),
                      cv::INTER_CUBIC, cv::BORDER_CONSTANT);

            OLOG("[BAND#%d] band remapping done.", b);
        }
        
        OLOG("Merging all image bands into a single multi-channel image ...");
        cv::Mat rMat;
        cv::merge(alignedBands, MSS_BANDS, rMat);
        OLOG("Merged.");
        
        //OLOG("Outputing section TIFF image ...");
        //WriteAlignedMSS_TIFF(rows, section);
        //OLOG("Output done.");
        return rMat;
    }
    
    void DumpInterBandShiftValues(int slices) {
        OLOG("|#SLC|Start|Center| End "
             "|   B1.x   |   B2.x   |   B3.x   |   B4.x   "
             "|   B1.y   |   B2.y   |   B3.y   |   B4.y   "
             "|   B1.r   |   B2.r   |   B3.r   |   B4.r   |");
        int sliceCols = PIXELS_PER_LINE / slices;
        for (int i = 0; i < slices; ++i) {
            OLOG("|%4d|%5d|%6d|%5d|%10.4f|%10.4f|%10.4f|%10.4f|%10.4f|%10.4f|%10.4f|%10.4f|%10.4f|%10.4f|%10.4f|%10.4f|"
                 , i, i * sliceCols, mBandShift[0][i].cx, (i + 1) * sliceCols
                 , mBandShift[0][i].dx, mBandShift[1][i].dx, mBandShift[2][i].dx, mBandShift[3][i].dx
                 , mBandShift[0][i].dy, mBandShift[1][i].dy, mBandShift[2][i].dy, mBandShift[3][i].dy
                 , mBandShift[0][i].rs, mBandShift[1][i].rs, mBandShift[2][i].rs, mBandShift[3][i].rs);
        }
    }
    
    void FilterInterBandShiftValues(int slices) {
        double nan = std::numeric_limits<double>::quiet_NaN();
        for (int b = 0; b < MSS_BANDS; ++b) {
            int fc = 0;
            InterBandShift * shifts = mBandShift[b];
            for (int i = 0; i < slices; ++i) {
                if (shifts[i].rs < IBCV_SHRESHHOLD) {
                    shifts[i].dx = nan;
                    shifts[i].dy = nan;
                } else {
                    fc++;
                }
            }
            if (fc < IBCV_MIN_COUNT) {
                xs s("Not enough valid correlation values for band#%d: %d valid values found, %d expected at least",
                     b, fc, IBCV_MIN_COUNT);
                OLOG("%s.", s.s);
                throw std::runtime_error(s.s);
            }
        }
    }
    
    void DoCorrelationPolynomialFitting(int slices) {
        for (int b = 0; b < MSS_BANDS; ++b) {
            // x拟合为直线，y拟合为二次曲线
            OLOG("Doing polynomial fitting for BAND %d ...", b);
            int vvi = 0;
            scoped_ptr<double> cxvals = new double[slices];
            scoped_ptr<double> xvals  = new double[slices];
            scoped_ptr<double> yvals  = new double[slices];
            for (int i = 0; i < slices; ++i) {
                const InterBandShift & ibs = mBandShift[b][i];
                if (ibs.rs >= IBCV_SHRESHHOLD) {
                    cxvals[vvi] = ibs.cx;
                    xvals [vvi] = ibs.dx;
                    yvals [vvi] = ibs.dy;
                    ++vvi;
                }
            }
            nc::NdArray<double> cxValues(cxvals, 1, vvi, false);
            nc::NdArray<double> dXvalues(xvals, vvi, 1, false);
            nc::NdArray<double> dYvalues(yvals, vvi, 1, false);
            auto poly1dX = nc::polynomial::Poly1d<double>::fit(cxValues, dXvalues, 1);
            auto poly1dY = nc::polynomial::Poly1d<double>::fit(cxValues, dYvalues, 2);
            auto coeffsX = poly1dX.coefficients();
            auto coeffsY = poly1dY.coefficients();
            
            mDeltaXcoeffs[b][0] = coeffsX[0];
            mDeltaXcoeffs[b][1] = coeffsX[1];
            mDeltaYcoeffs[b][0] = coeffsY[0];
            mDeltaYcoeffs[b][1] = coeffsY[1];
            mDeltaYcoeffs[b][2] = coeffsY[2];
            OLOG("\tdeltaX coeff: [1] %.15f, [0] %.9f",
                 coeffsX[1], coeffsX[0]);
            OLOG("\tdeltaY coeff: [2] %.15f, [1] %.15f, [0] %.9f",
                 coeffsY[2], coeffsY[1], coeffsY[0]);
        }
    }
    
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
    
    void InplaceRRC(uint16_t * buff, int w, int h, const RRCParam * rrcParam) {
        for (size_t y = 0; y < h; ++y) {
            for (size_t x = 0; x < w; ++x) {
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
        OLOG("LoadRRCParamFiles(): OK.\n");
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
        OLOG("Checking MSS raw file attributes ...");
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
        
        size_t unit = (int)std::min((size_t)8 * 1024 * 1024, size); // 8MB
        size_t written = 0;
        for (const char * p = buff; written < size; ) {
            auto wb = fwrite(p, 1, std::min(unit, size-written), f);
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
    //scoped_ptr<uint16_t> mAlignedMSS;
    cv::Mat mAlignedMSS;
    
    double mDeltaXcoeffs[MSS_BANDS][2];
    double mDeltaYcoeffs[MSS_BANDS][3];
};

END_NS

#endif /* preproc_h */
