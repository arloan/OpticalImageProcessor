//
//  oipshared.h
//  OpticalImageProcessor
//
//  Created by Qiu PENG on 30/5/22.
//

#ifndef oipshared_h
#define oipshared_h

// C89: __STDC_VERSION__ is 199409L
// C99: __STDC_VERSION__ is 199901L
// C11: __STDC_VERSION__ is 201112L
// C17: __STDC_VERSION__ is 201710L
//C++ pre-C++98: __cplusplus is 1.
//C++98: __cplusplus is 199711L.
//C++98 + TR1: This reads as C++98 and there is no way to check that I know of.
//C++11: __cplusplus is 201103L.
//C++14: __cplusplus is 201402L.
//C++17: __cplusplus is 201703L.
//C++20: __cplusplus is 202002L.

#ifndef MAX_PATH
#define MAX_PATH 1024
#endif

#define BYTES_PER_PIXEL     2       // Little Endian
#define PIXELS_PER_LINE     12288
#define BYTES_PER_PANLINE   (PIXELS_PER_LINE * BYTES_PER_PIXEL)
#define MSS_BANDS           4
#define PIXELS_PER_MSSBAND  (PIXELS_PER_LINE/MSS_BANDS)
#define BYTES_PER_MSSBAND   (BYTES_PER_PIXEL * PIXELS_PER_MSSBAND)
#define CORRELATION_LINES   16000

#define IBCV_DEF_THRESHOLD  0.4 // valid Inter Band Correlation Value threshold
#define IBCV_MIN_COUNT      5   // minimum IBC value count before polynomial fitting
#define IBCV_DEF_SECTIONS   3
#define IBCV_DEF_SLICES     10
#define IBCV_MIN_SLICES     8

// Inter-Band sPectrum Alignment
#define IBPA_DEFAULT_LINEOFFSET     0
#define IBPA_DEFAULT_BATCHLINES     20000
#define IBPA_DEFAULT_LINEOVERLAP    520
#define IBPA_MAX_LINEOVERLAP        3000
#define IBPA_MIN_PROCESSLINES       1500

// STITCHING
#define STT_DEF_SECTIONS    10
#define STT_DEF_SECLINES    16000
#define STT_DEF_OVERLAPPX   200
#define STT_DEF_PHCTHRHLD   0.4 // phaseCorrelate response threshold
#define STT_DEF_EDGECOLS    0

#define STT_STEM_EXT        ".STT"
#define PRESTT_STEM_EXT     ".PRESTT"
#define RRC_STEM_EXT        ".RRC"
#define IBPA_STEM_EXT       ".ALIGNED"
#define TIFF_FILE_EXT       ".TIFF"
#define RAW_FILE_EXT        ".RAW"

#define BEGIN_NS(ns) namespace ns {
#define END_NS }
#define USING_NS(ns) using namespace ns;

/// https://github.com/arloan/libimsux
#include <libimsux/imsux.hxx>
IMSUX_USE_NS

struct FileDtor {
    inline void operator () (FILE * f) { fclose(f); }
};

#define OLOGNEL(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define OLOG(fmt, ...)  printf(fmt "\n", ##__VA_ARGS__)
#ifdef DEBUG
#define DLOG    OLOG
#else
#define DLOG
#endif

#endif /* oipshared_h */
