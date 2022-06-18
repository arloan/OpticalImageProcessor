//
//  toolbox.h
//  OpticalImageProcessor
//
//  Created by Stone PEN on 30/5/22.
//

#ifndef toolbox_h
#define toolbox_h

#include <filesystem>
#include "oipshared.h"

BEGIN_NS(OIP)

class ToolBox {
public:
    static inline char * ChompChars(char * buff, size_t len = -1, const char * chomp = "\r\n") {
        auto n = strlen(chomp);
        if (len == -1) len = strlen(buff);
        
        for (char * p = buff + len - 1; p - buff >= 0; --p) {
            for (size_t i = 0; i < n; ++i) {
                if (p[0] == chomp[i]) {
                    *p = '\0';
                    break;
                }
            }
            if (p[0] != '\0') break;
        }
        return buff;
    }

    static inline char * FirstValidChars(char * buff, const char * excludes = "\t ") {
        auto n = strlen(excludes);
        for (; *buff; ++buff) {
            bool foundBadChar = false;
            for (size_t i = 0; i < n; ++i) {
                if (buff[0] == excludes[i]) {
                    foundBadChar = true;
                    break;
                }
            }
            if (!foundBadChar) break;
        }
        return buff;
    }

    static inline const char * FirstValidChars(const char * buff, const char * excludes = "\t ") {
        return (const char *)FirstValidChars((char *)buff, excludes);
    }
};

END_NS

#endif /* toolbox_h */
