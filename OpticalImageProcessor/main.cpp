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

typedef struct _InputParameters {
    char RawFilePAN[MAX_PATH];
    char RawFileMSS[MAX_PATH];
    
    // Relative radiation correction parameter file
    char RRCParaPAN[MAX_PATH];
    char RRCParaMS1[MAX_PATH];
    char RRCParaMS2[MAX_PATH];
    char RRCParaMS3[MAX_PATH];
    char RRCParaMS4[MAX_PATH];
} InputParameters;

InputParameters ips_ = { 0 };

void ParseTaskFile(const char * taskFile, InputParameters & ip) {
    scoped_ptr<FILE, FileDtor> f = fopen(taskFile, "rb");
    if (f.isNull()) throw errno_error(xs("cannot open task file: %d", errno));
    
    char buff[2048] = { 0 };
    for (char * rp = NULL; (rp = fgets(buff, sizeof(buff), f)); ) {
        ToolBox::ChompChars(buff, strlen(buff));

        auto pos = strchr(buff, '=');
        if (pos == NULL) {
            fprintf(stderr, "ignored: unrecognized line: %s", buff);
            continue;
        }
        auto value = ToolBox::FirstNonChars(pos+1);
        *pos = '\0';
        auto key = ToolBox::ChompChars(buff, pos - buff, "\t ");
        
//#ifdef DEBUG
//        printf("key: `%s', value: `%s'\n", key, value);
//#endif
        
        if (strcmp(key, TASKKEY_PAN) == 0) {
            strncpy(ip.RawFilePAN, value, MAX_PATH);
        } else if (strcmp(key, TASKKEY_MSS) == 0) {
            strncpy(ip.RawFileMSS, value, MAX_PATH);
        } else if (strcmp(key, TASKKEY_RRCPAN) == 0) {
            strncpy(ip.RRCParaPAN, value, MAX_PATH);
        } else if (strcmp(key, TASKKEY_RRCMS1) == 0) {
            strncpy(ip.RRCParaMS1, value, MAX_PATH);
        } else if (strcmp(key, TASKKEY_RRCMS2) == 0) {
            strncpy(ip.RRCParaMS2, value, MAX_PATH);
        } else if (strcmp(key, TASKKEY_RRCMS3) == 0) {
            strncpy(ip.RRCParaMS3, value, MAX_PATH);
        } else if (strcmp(key, TASKKEY_RRCMS4) == 0) {
            strncpy(ip.RRCParaMS4, value, MAX_PATH);
        }
    }
}

void ParseInputParametersFromCommandLineArguments(int argc, const char * argv[]) {
    /// obsolete: oip -pan:/path/to/the/PAN/file -mss:/path/to/the/MSS/file
    /// command line:
    /// oip -task:/path/to/the/task/file
    /// task file: describe task parameters, including PAN,MSS,RRC parameter file, etc.
    /*
    if (argc < 3) {
        throw std::invalid_argument("need more command line arguments");
    }
    for (int i = 1; i < argc; ++i) {
        if (strncmp(argv[i], AN_PAN, NPAN) == 0) {
            strncpy(ips_.RawFilePAN, argv[1]+NPAN, MAX_PATH);
        }
        if (strncmp(argv[i], AN_MSS, NMSS) == 0) {
            strncpy(ips_.RawFileMSS, argv[1]+NMSS, MAX_PATH);
        }
        if (strncmp(argv[i], AN_RRC, NRRC) == 0) {
            strncpy(ips_.RRCParFile, argv[1]+NRRC, MAX_PATH);
        }
    }
    
    if (ips_.RawFilePAN[0] == 0) throw std::invalid_argument("PAN file path not supplied");
    if (ips_.RawFileMSS[0] == 0) throw std::invalid_argument("MSS file path not supplied");
    if (ips_.RRCParFile[0] == 0) throw std::invalid_argument("RRC Parameter file path not supplied");

    if (access(ips_.RawFilePAN, F_OK) != 0) throw std::invalid_argument("specified PAN file not exists");
    if (access(ips_.RawFileMSS, F_OK) != 0) throw std::invalid_argument("specified MSS file not exists");
    if (access(ips_.RRCParFile, F_OK) != 0) throw std::invalid_argument("specified RRC Parameter file not exists");
    //*/
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
                        , ips_.RRCParaMS1
                        , ips_.RRCParaMS2
                        , ips_.RRCParaMS3
                        , ips_.RRCParaMS4
                        );
        pp.DoRRC();
        
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
