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
#define MSS_BANDS           4
#define PIXELS_PER_MSSSEC   (PIXELS_PER_LINE/MSS_SECTIONS)

#define AN_PAN  "-pan:"
#define AN_MSS  "-mss:"
#define AN_RRC  "-rrc:"
#define NPAN    (sizeof(AN_PAN)-1)
#define NMSS    (sizeof(AN_MSS)-1)
#define NRRC    (sizeof(AN_RRC)-1)

#define AN_TASK "-task:"
#define NTASK   (sizeof(AN_TASK)-1)

#define TASKKEY_PAN     "PAN"
#define TASKKEY_MSS     "MSS"
#define TASKKEY_RRCPAN  "RRC_PAN"
#define TASKKEY_RRCMS1  "RRC_MSB1"
#define TASKKEY_RRCMS2  "RRC_MSB2"
#define TASKKEY_RRCMS3  "RRC_MSB3"
#define TASKKEY_RRCMS4  "RRC_MSB4"

#define BEGIN_NS(ns) namespace ns {
#define END_NS }
#define USING_NS(ns) using namespace ns;

/// https://github.com/arloan/libimsux
#include <libimsux/imsux.hxx>
IMSUX_USE_NS

struct FileDtor {
    inline void operator () (FILE * f) { fclose(f); }
};

#define OLOG(fmt, ...)  printf(fmt "\n", ##__VA_ARGS__)
#ifdef DEBUG
#define DLOG    OLOG
#else
#define DLOG
#endif

#endif /* oipshared_h */
