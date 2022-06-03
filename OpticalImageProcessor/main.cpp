//
//  main.cpp
//  OpticalImageProcessor
//
//  Created by Qiu PENG on 30/5/22.
//

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

#include "preproc.h"

USING_NS(OIP)

struct InputParameters {
    std::string RawFilePAN;
    std::string RawFileMSS;
    
    // Relative radiation correction parameter file
    std::string RRCParaPAN;
    std::string RRCParaMSS[MSS_BANDS];
    
    int IBCOR_Slices;
};

InputParameters ips_ = { 0 };

void ParseTaskFile(const char * taskFile, InputParameters & ip) {
    scoped_ptr<FILE, FileDtor> f = fopen(taskFile, "rb");
    if (f.isNull()) throw errno_error("cannot open task file");
    
    char buff[2048] = { 0 };
    for (char * rp = NULL; (rp = fgets(buff, sizeof(buff), f)); ) {
        ToolBox::ChompChars(buff, strlen(buff));
        char * buf = ToolBox::FirstValidChars(buff);
        if (buf[0] == '#') continue;

        auto pos = strchr(buf, '=');
        if (pos == NULL) {
            fprintf(stderr, "ignored: unrecognized line: %s", buff);
            continue;
        }
        auto value = ToolBox::FirstValidChars(pos+1);
        *pos = '\0';
        auto key = ToolBox::ChompChars(buf, pos - buff, "\t ");
        
//#ifdef DEBUG
//        printf("key: `%s', value: `%s'\n", key, value);
//#endif
        
        if (       strcmp(key, TASKKEY_PAN   ) == 0) {
            ip.RawFilePAN = value;
        } else if (strcmp(key, TASKKEY_MSS   ) == 0) {
            ip.RawFileMSS = value;
        } else if (strcmp(key, TASKKEY_RRCPAN) == 0) {
            ip.RRCParaPAN = value;
        } else if (strcmp(key, TASKKEY_RRCMS1) == 0) {
            ip.RRCParaMSS[0] = value;
        } else if (strcmp(key, TASKKEY_RRCMS2) == 0) {
            ip.RRCParaMSS[1] = value;
        } else if (strcmp(key, TASKKEY_RRCMS3) == 0) {
            ip.RRCParaMSS[2] = value;
        } else if (strcmp(key, TASKKEY_RRCMS4) == 0) {
            ip.RRCParaMSS[3] = value;
        } else if (strcmp(key, TASKKEY_IBCSLCS)== 0) {
            ip.IBCOR_Slices = atoi(value);
        }
    }
    
    if (ip.IBCOR_Slices <= 0) ip.IBCOR_Slices = DEFAULT_IBCSLCS;
}

void ParseInputParametersFromCommandLineArguments(int argc, const char * argv[]) {
    /// task file: describe task parameters, including PAN,MSS,RRC parameter file, etc.
    if (argc != 2 || strncmp(argv[1], AN_TASK, NTASK) != 0) {
        throw std::invalid_argument(xs("USAGE: %s -task:/path/to/task/file", argv[0]).s);
    }
    
    char taskFile[MAX_PATH] = { 0 };
    strncpy(taskFile, argv[1] + NTASK, MAX_PATH);
    if (access(taskFile, F_OK) != 0) throw std::invalid_argument("specified task file not exists");
    
    ParseTaskFile(taskFile, ips_);
}

int main(int argc, const char * argv[]) {
    try {
        ParseInputParametersFromCommandLineArguments(argc, argv);
        
        PreProcessor pp(  ips_.RawFilePAN
                        , ips_.RawFileMSS
                        , ips_.RRCParaPAN
                        , ips_.RRCParaMSS);
        pp.LoadPAN();
        pp.LoadMSS();
        pp.DoRRC();
        pp.CalcInterBandCorrelation(ips_.IBCOR_Slices);
        pp.DoInterBandAlignment();
        // pp.WriteAlignedMSS_RAW();
        pp.WriteAlignedMSS_TIFF();
        
        return 0;
    } catch (std::invalid_argument & ex) {
        printf("USAGE ERROR: %s.\n", ex.what());
        return 255;
    } catch (std::exception & ex) {
        printf("FATAL ERROR: %s.\n", ex.what());
        return 254;
    } catch (...) {
        printf("UNKOWN FATAL ERROR OCCURED.\n");
        return 1;
    }
}
