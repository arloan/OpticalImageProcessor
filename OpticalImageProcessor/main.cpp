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
#include "stitcher.h"

USING_NS(OIP)

struct InputParameters {
    std::string RawFilePAN;
    std::string RawFileMSS;
    
    // Relative radiation correction parameter file
    std::string RRCParaPAN;
    std::string RRCParaMSS[MSS_BANDS];
    
    int IBCOR_Slices;
    int IBCOR_Sections;
    int IBPA_LineOffset;
    int IBPA_BatchLines;
    int IBPA_OverlapLines;
    
    bool doRRC;
    bool outputRrcPanTiff;
    
    InputParameters() :
        IBCOR_Slices(IBCV_DEF_SLICES),
        IBCOR_Sections(IBCV_DEF_SECTIONS),
        IBPA_LineOffset(IBPA_DEFAULT_LINEOFFSET),
        IBPA_BatchLines(IBPA_DEFAULT_BATCHLINES),
        IBPA_OverlapLines(IBPA_DEFAULT_LINEOVERLAP),
        doRRC(true),
        outputRrcPanTiff(false)
    {}
};

struct StitchParams {
    std::string rawFilePAN1;
    std::string rawFilePAN2;
    std::string rrcParaPAN1;
    std::string rrcParaPAN2;

    int sections;
    int sectionLines;
    int overlapCols;
};

InputParameters ips_;
StitchParams stp_;

void PreStitch(const StitchParams & stp);
void DefaultAction(const InputParameters & ip);

int ParseInputParametersFromCommandLineArguments(int argc, const char * argv[]) {
    CLI::App app("Optical Satellite Image Pre-Processing/Processing Utility", "OpticalImageProcessor");
    app.set_version_flag("-v,--version", "1.1");
    app.require_subcommand(0, 1);
    
    auto existanceCheck = [](const std::string & v) {
        if (!std::filesystem::exists(v)) return "file not exists";
        return "";
    };
    
    // `prestitch` sub command arguments
    CLI::App & psa = * app.add_subcommand("prestitch", "Do preparation parameters calculting for CMOS stitching");
    psa.add_option("--pan1", stp_.rawFilePAN1, "PAN raw image file of CMOS1")->check(existanceCheck);
    psa.add_option("--pan2", stp_.rawFilePAN1, "PAN raw image file of CMOS2")->check(existanceCheck);
    psa.add_option("--rrc1", stp_.rrcParaPAN1,
                   "Relative Radiometric Correction parameter file for PAN1")->check(existanceCheck);
    psa.add_option("--rrc2", stp_.rrcParaPAN2,
                   "Relative Radiometric Correction parameter file for PAN2")->check(existanceCheck);
    psa.add_option("-s,--sections", stp_.sections,
                   "Section count for stitching parameter calculating")->default_val(STT_DEF_SECTIONS);
    psa.add_option("-l,--section-lines",
                   stp_.sectionLines,
                   "Data lines per section for stitching parameter calculating")->default_val(STT_DEF_SECLINES);
    psa.add_option("--stitch-overlap",
                   stp_.overlapCols,
                   "Overlapped columns of pixel for PAN image stitching")->default_val(STT_DEF_OVERLAPPX);
    psa.callback([](){
        PreStitch(stp_);
    });

    // default command arguments
    app.add_option("--pan", ips_.RawFilePAN, "PAN raw image file path")->check(existanceCheck);
    app.add_option("--mss", ips_.RawFileMSS, "MSS raw image file path")->check(existanceCheck);
    app.add_flag  ("--rrc,!--no-rrc", ips_.doRRC, "whether or not do Relative Radiometric Correction, default is to do it");
    app.add_flag  ("--write-rrcpan,!--no-rrcpan",
                   ips_.outputRrcPanTiff,
                   "whether or not write RRC-ed PAN data as tiff image file, default is not");
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
                   xs("split slice count for inter-band correlation calculating, default is %d", IBCV_DEF_SLICES));
    app.add_option("--ibc-sections",
                   ips_.IBCOR_Sections,
                   xs("split virtically section count for inter-band correlation calculating, default is %d",
                      IBCV_DEF_SECTIONS))->default_val(IBCV_DEF_SECTIONS);
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
    
    app.callback([](){
        DefaultAction(ips_);
    });

    try {
        app.parse(argc, argv);
        return 0;
    } catch (const CLI::Success &e) {
        return app.exit(e) + 255;
    } catch (const CLI::ParseError &e) {
        return app.exit(e);
    }
}

void PreStitch(const StitchParams & stp) {
    Stitcher stt(stp.rawFilePAN1,
                 stp.rawFilePAN2,
                 stp.rrcParaPAN1,
                 stp.rrcParaPAN2,
                 stp.sections,
                 stp.sectionLines,
                 stp.overlapCols);
    stt.PreStitch();
}

void DefaultAction(const InputParameters & ip) {
    if (ip.doRRC
        &&(ip.RRCParaPAN   .length() == 0
        || ip.RRCParaMSS[0].length() == 0
        || ip.RRCParaMSS[1].length() == 0
        || ip.RRCParaMSS[2].length() == 0
        || ip.RRCParaMSS[3].length() == 0)) {
        throw std::invalid_argument("RRC parameter file needed");
    }
    GDALAllRegister();
    PreProcessor pp(  ip.RawFilePAN
                    , ip.RawFileMSS
                    , ip.RRCParaPAN
                    , ip.RRCParaMSS);
    pp.LoadPAN();
    pp.LoadMSS();
    
    if (ip.doRRC) pp.DoRRC();
    if (ip.outputRrcPanTiff) pp.WriteRRCedPAN_TIFF(ip.IBPA_LineOffset);
    //pp.WriteRRCedMSS();
    
    pp.CalcInterBandCorrelation(ip.IBCOR_Slices, ip.IBCOR_Sections);
    pp.DoInterBandAlignment(ip.IBPA_BatchLines, ip.IBPA_LineOffset, ip.IBPA_OverlapLines);
}

int main(int argc, const char * argv[]) {
    try {
        return ParseInputParametersFromCommandLineArguments(argc, argv);
    } catch (std::exception & ex) {
        printf("FATAL ERROR: %s.\n", ex.what());
        return 254;
    } catch (...) {
        printf("UNKOWN FATAL ERROR OCCURED.\n");
        return 1;
    }
}
