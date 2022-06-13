//
//  main.cpp
//  OpticalImageProcessor
//
//  Created by Qiu PENG on 30/5/22.
//

#include <stdio.h>
#include <stdint.h>
#include <cstring>
#include <stdexcept>
#include <CLI/CLI.hpp>

#include "preproc.h"

USING_NS(OIP)

struct InputParameters {
    std::string RawFilePAN;
    std::string RawFileMSS;
    
    // Relative radiation correction parameter file
    std::string RRCParaPAN;
    std::string RRCParaMSS[MSS_BANDS];
    
    int IBCOR_Slices;
    int IBPA_LineOffset;
    int IBPA_BatchLines;
    int IBPA_OverlapLines;
    
    InputParameters() :
        IBCOR_Slices(DEFAULT_IBCSLCS),
        IBPA_LineOffset(IBPA_DEFAULT_LINEOFFSET),
        IBPA_BatchLines(IBPA_DEFAULT_BATCHLINES),
        IBPA_OverlapLines(IBPA_DEFAULT_LINEOVERLAP)
    {}
};

InputParameters ips_;

int ParseInputParametersFromCommandLineArguments(int argc, const char * argv[]) {
    CLI::App app("Optical Satellite Image Pre-Processing/Processing Utility", "OpticalImageProcessor");
    app.set_version_flag("-v,--version", "1.0");
    
    auto existanceCheck = [](const std::string & v) {
        if (!std::filesystem::exists(v)) return "file not exists";
        return "";
    };
    
    app.add_option("--pan", ips_.RawFilePAN, "PAN raw image file path")->check(existanceCheck);
    app.add_option("--mss", ips_.RawFileMSS, "MSS raw image file path")->check(existanceCheck);
    app.add_option("--rrc-pan", ips_.RRCParaPAN, "Relative Radiometric Correction parameter file path for PAN image");
    app.add_option("--rrc-msb1",
                   ips_.RRCParaMSS[0],
                   "Relative Radiometric Correction parameter file path for MSS band #1 (1-based band NO.)"
                   )->check(existanceCheck);
    app.add_option("--rrc-msb2",
                   ips_.RRCParaMSS[1],
                   "Relative Radiometric Correction parameter file path for MSS band #2 (1-based band NO.)"
                   )->check(existanceCheck);
    app.add_option("--rrc-msb3",
                   ips_.RRCParaMSS[2],
                   "Relative Radiometric Correction parameter file path for MSS band #3 (1-based band NO.)"
                   )->check(existanceCheck);
    app.add_option("--rrc-msb4",
                   ips_.RRCParaMSS[3],
                   "Relative Radiometric Correction parameter file path for MSS band #4 (1-based band NO.)"
                   )->check(existanceCheck);
    app.add_option("--slices",
                   ips_.IBCOR_Slices,
                   xs("split slice count for inter-band correlation calculating, default is %d", DEFAULT_IBCSLCS));
    app.add_option("--line-offset",
                   ips_.IBPA_LineOffset,
                   "line offset for inter-band pixel alignment processing, default is 0");
    app.add_option("--lines-section",
                   ips_.IBPA_BatchLines,
                   xs("line-per-section for inter-band pixel alignment processing, default is %d", IBPA_DEFAULT_BATCHLINES));
    app.add_option("--overlap-lines",
                   ips_.IBPA_OverlapLines,
                   xs("overlapped lines for each sibling portion during inter-band pixel alignment processing, "
                      "default is %d", IBPA_DEFAULT_LINEOVERLAP));

    try {
        app.parse(argc, argv);
        return 0;
    } catch (const CLI::Success &e) {
        return app.exit(e) + 255;
    } catch (const CLI::ParseError &e) {
        return app.exit(e);
    }
}

int main(int argc, const char * argv[]) {
    try {
        int e = ParseInputParametersFromCommandLineArguments(argc, argv);
        if (e != 0) return e;
        
        PreProcessor pp(  ips_.RawFilePAN
                        , ips_.RawFileMSS
                        , ips_.RRCParaPAN
                        , ips_.RRCParaMSS);
        pp.LoadPAN();
        pp.LoadMSS();
        
        pp.DoRRC();
        //pp.WriteRRCedPAN(ips_.IBPA_LineOffset);
        //pp.WriteRRCedMSS();
        
        pp.CalcInterBandCorrelation(ips_.IBCOR_Slices);
        pp.DoInterBandAlignment(ips_.IBPA_BatchLines, ips_.IBPA_LineOffset, ips_.IBPA_OverlapLines);
        
        return 0;
    } catch (std::exception & ex) {
        printf("FATAL ERROR: %s.\n", ex.what());
        return 254;
    } catch (...) {
        printf("UNKOWN FATAL ERROR OCCURED.\n");
        return 1;
    }
}
