//
//  stitcher.h
//  OpticalImageProcessor
//
//  Created by Stone PEN on 17/6/22.
//

#ifndef stitcher_h
#define stitcher_h

#include <opencv2/core/mat.hpp>

#include "oipshared.h"
#include "imageop.h"

BEGIN_NS(OIP)

class Stitcher
{
public:
    Stitcher(const std::string & pan1,
             const std::string & pan2,
             const std::string & rrc1,
             const std::string & rrc2,
             int sections = STT_DEF_SECTIONS,
             int linePerSection = STT_DEF_SECLINES,
             int overlapCols = STT_DEF_OVERLAPPX)
    : mFilePAN1(pan1), mFilePAN2(pan2)
    , mParamFileRRC1(rrc1), mParamFileRRC2(rrc2)
    , mSections(sections), mLinePerSection(linePerSection), mOverlapCols(overlapCols) {
        
        size_t s1 = IMO::FileSize(pan1);
        if ((size_t)sections * linePerSection * BYTES_PER_PIXEL > s1) {
            throw std::invalid_argument("PAN1 size too small for SECTION & LINE_PER_SECTION argument");
        }
        size_t s2 = IMO::FileSize(pan2);
        if ((size_t)sections * linePerSection * BYTES_PER_PIXEL > s2) {
            throw std::invalid_argument("PAN2 size too small for SECTION & LINE_PER_SECTION argument");
        }
        if (s1 != s2) {
            throw std::invalid_argument("PAN1 size doesn't match PAN2 size");
        }
        mLinesPAN = (int)(s1 / BYTES_PER_PANLINE);
        OLOG("PAN: %s lines total.", comma_sep(mLinesPAN).sep());
        
        mRrcFilePAN1 = mFilePAN1;
        mRrcFilePAN2 = mFilePAN2;
    }
    
    int PreStitch() {
        mPreSttFilePAN2 = IMO::BuildOutputFilePath(mFilePAN2, PRESTT_STEM_EXT);
        scoped_ptr<FILE, FileDtor> fPan2 = fopen(mRrcFilePAN2.c_str(), "rb");
        scoped_ptr<FILE, FileDtor> fPreStt2 = fopen(mPreSttFilePAN2.c_str(), "wb");
        
        cv::Mat1w buff(REMAP_SECTION_ROWS, PIXELS_PER_LINE);
        cv::Mat1d mapx(REMAP_SECTION_ROWS, PIXELS_PER_LINE, mDeltaX);
        cv::Mat1d mapy(REMAP_SECTION_ROWS, PIXELS_PER_LINE, mDeltaY);
        
        const size_t row_bytes = BYTES_PER_PANLINE;
        auto pick_src_image = [&](int row_offset, int rows) {
            fseek(fPan2, row_offset * row_bytes, SEEK_SET);
            if (fread(buff.data, row_bytes, rows, fPan2) < rows) {
                throw std::runtime_error("PreStitch(): not enough data read from RRC PAN2 raw file");
            }
            return buff;
        };
        auto prepare_mapx = [&](int,int) { return mapx; };
        auto prepare_mapy = [&](int,int) { return mapy; };
        auto write_remapped_data = [&](cv::Mat mapped) {
            if (fwrite(mapped.data, row_bytes, mapped.rows, fPreStt2) != mapped.rows) {
                throw std::runtime_error("PreStitch(): not enough data written to pre-stitched PAN2 raw file");
            }
        };
        
        int ucut = mDeltaY >= 0.0 ? 0 : (int)mDeltaY + 1;
        int bcut = mDeltaY >= 0.0 ? (int)mDeltaY + 1 : 0;
        // write empty data to upper&bottom cut zone to keep file size the same as PAN1
        char empty[BYTES_PER_PANLINE] = { 0 };
        for (int i = 0; i < ucut; ++i) {
            fwrite(empty, BYTES_PER_PANLINE, 1, fPreStt2);
        }
        int imageLines = IMO::SectionaryRemap(mLinesPAN, ucut, bcut,
                                              pick_src_image,
                                              prepare_mapx,
                                              prepare_mapy,
                                              write_remapped_data,
                                              cv::INTER_CUBIC);
        for (int i = 0; i < bcut; ++i) {
            fwrite(empty, BYTES_PER_PANLINE, 1, fPreStt2);
        }
        return imageLines;
    }
    
    void DoRRC() {
        mRrcFilePAN1 = IMO::BuildOutputFilePath(mFilePAN1, RRC_STEM_EXT);
        mRrcFilePAN2 = IMO::BuildOutputFilePath(mFilePAN2, RRC_STEM_EXT);
        IMO::DoRRC4RAW(mFilePAN1, PIXELS_PER_LINE, mParamFileRRC1, mRrcFilePAN1);
        IMO::DoRRC4RAW(mFilePAN2, PIXELS_PER_LINE, mParamFileRRC2, mRrcFilePAN2);
    }
    
    void CalcSttParameters(int edgeCols = 0) {
        int gapLines = (mLinesPAN - mSections * mLinePerSection) / (mSections + 1);
        int stepLines = gapLines + mLinePerSection;
        int sectionBytes = mLinePerSection * BYTES_PER_PANLINE;
        cv::Mat1w section1(mLinePerSection, PIXELS_PER_LINE);
        cv::Mat1w section2(mLinePerSection, PIXELS_PER_LINE);
        
        size_t rb = 0;
        mDeltaX = 0.0;
        mDeltaY= 0.0;
        mResponse = 0.0;
        int valid = 0;
        
        OLOG("Calculating stitching delta values ...");
        OLOG("| offset |  delta x |  delta y | response | r |");
        for (int i = 0; i < mSections; ++i) {
            int line_offset = gapLines + i * stepLines;
            size_t offset = (size_t)line_offset * BYTES_PER_PANLINE;
            IMO::ReadFileContent(mRrcFilePAN1, rb, offset, sectionBytes, (char *)section1.data);
            //OLOG("%ld bytes read from PAN1", rb);
            IMO::ReadFileContent(mRrcFilePAN2, rb, offset, sectionBytes, (char *)section2.data);
            //OLOG("%ld bytes read from PAN2", rb);

            /// cv::Mat::colRange(startIncluded, endExcluded), col is 0-based
            cv::Mat1f sliceF1 = section1.colRange(PIXELS_PER_LINE - mOverlapCols, PIXELS_PER_LINE - edgeCols);
            cv::Mat1f sliceF2 = section2.colRange(edgeCols, mOverlapCols);

            double resp = 0.0;
            cv::Point2d rv;
            rv = cv::phaseCorrelate(sliceF1.clone(), sliceF2.clone(), cv::noArray(), &resp);
            if (resp >= STT_DEF_PHCTHRHLD) {
                mDeltaX   += rv.x;
                mDeltaY   += rv.y;
                mResponse += resp;
                valid++;
            }
            OLOG("|%7d |%10.4f|%10.4f|%10.4f|%s|",
                 line_offset, rv.x, rv.y, resp,
                 resp >= STT_DEF_PHCTHRHLD ? " ✔︎ " : " ✘ ");
        }
        if (valid == 0) {
            throw std::runtime_error("No valid delta value found for stitching parameter calculating");
        }
        
        mDeltaX   /= valid;
        mDeltaY   /= valid;
        mResponse /= valid;
        OLOG("Total %d valid delta value pairs found, everage value:", valid);
        OLOG("    dx: %.5f, dy: %.5f, r: %.5f", mDeltaX, mDeltaY, mResponse);
    }
    
private:
    std::string mFilePAN1;
    std::string mFilePAN2;
    std::string mParamFileRRC1;
    std::string mParamFileRRC2;
    
    std::string mRrcFilePAN1;
    std::string mRrcFilePAN2;
    std::string mPreSttFilePAN2;

    double mDeltaX;
    double mDeltaY;
    double mResponse;

    int mSections;
    int mLinePerSection;
    int mOverlapCols; // in pixel
    int mLinesPAN;
};

END_NS

#endif /* stitcher_h */
