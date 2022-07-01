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

    double IBCOR_Threshold;
    int IBCOR_Slices;
    int IBCOR_Sections;
    int IBPA_LineOffset;
    int IBPA_BatchLines;
    int IBPA_OverlapLines;
    
    bool doRRC4PAN;
    bool doRRC4MSS;
    bool outputRrcPanTiff;
    
    InputParameters() :
        IBCOR_Slices(IBCV_DEF_SLICES),
        IBCOR_Sections(IBCV_DEF_SECTIONS),
        IBPA_LineOffset(IBPA_DEFAULT_LINEOFFSET),
        IBPA_BatchLines(IBPA_DEFAULT_BATCHLINES),
        IBPA_OverlapLines(IBPA_DEFAULT_LINEOVERLAP),
        doRRC4PAN(false),
        doRRC4MSS(true),
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
    int edgeCols;
    
    bool doRRC;
    bool onlyParamCalc;
    
    StitchParams() :
        sections(STT_DEF_SECTIONS),
        sectionLines(STT_DEF_SECLINES),
        overlapCols(STT_DEF_OVERLAPPX),
        edgeCols(STT_DEF_EDGECOLS),
        doRRC(true),
        onlyParamCalc(false)
    {}
};

InputParameters ips_;
StitchParams stp_;

void PreStitch();
void DefaultAction();

int ParseInputParametersFromCommandLineArguments(int argc, const char * argv[]) {
    CLI::App app("Optical Satellite Image Pre-Processing/Processing Utility", "OpticalImageProcessor");
    app.set_version_flag("-v,--version", "1.1");
    app.require_subcommand(0, 1);
    
    // `prestitch` sub command arguments
    CLI::App & psa = * app.add_subcommand("prestitch",
                                          "Do preparation parameters calculating & PAN2 pixel correction for CMOS stitching");
    psa.add_option("--pan1", stp_.rawFilePAN1, "PAN raw image file of CMOS1")->required()->check(CLI::ExistingFile);
    psa.add_option("--pan2", stp_.rawFilePAN2, "PAN raw image file of CMOS2")->required()->check(CLI::ExistingFile);
    psa.add_option("--rrc1", stp_.rrcParaPAN1,
                   "Relative Radiometric Correction parameter file for PAN1")->check(CLI::ExistingFile);
    psa.add_option("--rrc2", stp_.rrcParaPAN2,
                   "Relative Radiometric Correction parameter file for PAN2")->check(CLI::ExistingFile);
    psa.add_option("-s,--sections", stp_.sections,
                   "Section count for stitching parameter calculating")->default_val(STT_DEF_SECTIONS);
    psa.add_option("-l,--section-lines",
                   stp_.sectionLines,
                   "Data lines per section for stitching parameter calculating")->default_val(STT_DEF_SECLINES);
    psa.add_option("--stitch-overlap",
                   stp_.overlapCols,
                   "Overlapped columns of pixel for PAN image stitching")->default_val(STT_DEF_OVERLAPPX);
    psa.add_option("-e,--edge-cols", stp_.edgeCols,
                   "Ignored edge cols (right edge of PAN1 & left edge of PAN2) "
                   "when calculating stitching parameter")->default_val(0)->check([](const std::string & v) {
        int iv = atoi(v.c_str());
        if (iv < 0 || iv > stp_.overlapCols / 2) {
            return "invalid edge cols";
        }
        return "";
    });
    
    psa.add_flag  ("-r,--rrc,!--no-rrc",stp_.doRRC,
                   "Whether do Relative Radiometric Correction or not for PAN after pre-stitch parameter calclationg");
    psa.add_flag  ("-c,--only-calculate", stp_.onlyParamCalc,
                   "Only do pre-stitch parameter calculation, do not output pixel-adjusted PAN file.");
    
    psa.callback([](){
        PreStitch();
    });

    // `stitch` sub command
    std::string image1;
    std::string image2;
    std::string outputFile;
    std::string bandMap;
    int foldCols = 0;
    bool useGDAL = false;
    CLI::App & sta = * app.add_subcommand("stitch",
                                          "Stitch two PAN or MSS images.");
    sta.add_option("--image1", image1, "Left image file path")->required();
    sta.add_option("--image2", image2, "Right image file path")->required();
    sta.add_option("-o,--out", outputFile, "Path of the output stitched image file");
    sta.add_option("-c,--fold-cols", foldCols,
                   "Folding cols (in pixel) when stitching two images"
                   )->required()->check([](const std::string & v) {
        int col = atoi(v.c_str());
        if (col < 2) return "fold column value too small";
        return "";
    });
    auto gdal =
    sta.add_flag  ("-g,--GDAL", useGDAL,
                   "Use GDAL to output stitched image file (TIFF only). "
                   "GDAL is always used for Big TIFF output even -g not supplied.")->default_val(false);
    sta.add_option("-m,--band-map", bandMap, "Map output band order (1-based), i.e '3,2,1,4'"
                   )->needs(gdal);
    sta.callback([&]() {
        int map[MSS_BANDS] = { 0 };
        if (bandMap.length() > 0) {
            if (sscanf(bandMap.c_str(), "%d,%d,%d,%d", map, map+1, map+2, map+3) != 4) {
                throw CLI::ValidationError("-m", "need 4 band indices");
            }
            for (int i = 0; i < MSS_BANDS; ++i) {
                if (map[i] <= 0 || map[i] > MSS_BANDS) {
                    throw CLI::ValidationError("-m", "invalid band index");
                }
            }
        }
        Stitcher::Stitch(image1, image2, outputFile, foldCols / 2, useGDAL, bandMap.length() > 0 ? map : NULL);
    });
    
    // default command arguments
    app.add_option("--pan", ips_.RawFilePAN, "PAN raw image file path")->check(CLI::ExistingFile);
    auto rrc4pan =
    app.add_flag  ("--do-rrc4pan", ips_.doRRC4PAN,
                   "Whether or not do Relative Radiometric Correction for PAN, "
                   "not(default) if this flag not provided");
    app.add_option("--rrc-pan", ips_.RRCParaPAN,
                   "Relative Radiometric Correction parameter file path for PAN image")->needs(rrc4pan);
    app.add_flag  ("--write-rrcpan,!--no-rrcpan",
                   ips_.outputRrcPanTiff,
                   "Whether or not write RRC PAN data as tiff image file")
        ->default_val(false)->needs(rrc4pan);
    
    app.add_option("--mss", ips_.RawFileMSS, "MSS raw image file path")->check(CLI::ExistingFile);
    app.add_flag  ("!--no-rrc4mss", ips_.doRRC4MSS,
                   "Whether or not do Relative Radiometric Correction for PAN");
    app.add_option("--rrc-msb1",
                   ips_.RRCParaMSS[0],
                   "Relative Radiometric Correction parameter file path for MSS band #1 (1-based band NO.)"
                   )->check(CLI::ExistingFile);
    app.add_option("--rrc-msb2",
                   ips_.RRCParaMSS[1],
                   "Relative Radiometric Correction parameter file path for MSS band #2 (1-based band NO.)"
                   )->check(CLI::ExistingFile);
    app.add_option("--rrc-msb3",
                   ips_.RRCParaMSS[2],
                   "Relative Radiometric Correction parameter file path for MSS band #3 (1-based band NO.)"
                   )->check(CLI::ExistingFile);
    app.add_option("--rrc-msb4",
                   ips_.RRCParaMSS[3],
                   "Relative Radiometric Correction parameter file path for MSS band #4 (1-based band NO.)"
                   )->check(CLI::ExistingFile);
    
    app.add_option("--slices",
                   ips_.IBCOR_Slices,
                   "Split slice count for inter-band correlation calculating")->default_val(IBCV_DEF_SLICES);
    app.add_option("--ibc-sections",
                   ips_.IBCOR_Sections,
                   "Split virtically section count for inter-band correlation calculating")->default_val(IBCV_DEF_SECTIONS);
    app.add_option("--ibc-threshold", ips_.IBCOR_Threshold,
                   "Threshold of valid inter-band correlation calculated parameter value"
                   )->default_val(IBCV_DEF_THRESHOLD)->check([](const std::string & v) {
        double dv = strtod(v.c_str(), NULL);
        if (dv < 0.0 or dv >= 1.0) {
            return "invalid threshold value";
        }
        return "";
    });
    app.add_option("--line-offset",
                   ips_.IBPA_LineOffset,
                   "Line offset for inter-band pixel alignment processing")->default_val(IBPA_DEFAULT_LINEOFFSET);
    app.add_option("--lines-section",
                   ips_.IBPA_BatchLines,
                   "Line-per-section for inter-band pixel alignment processing")->default_val(IBPA_DEFAULT_BATCHLINES);
    app.add_option("--overlap-lines",
                   ips_.IBPA_OverlapLines,
                   "Overlapped lines for each sibling portion during inter-band pixel alignment processing"
                   "")->default_val(IBPA_DEFAULT_LINEOVERLAP);
    
    app.callback([&](){
        if (app.get_subcommands().size() == 0) {
            DefaultAction();
        }
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

void PreStitch() {
    const StitchParams & stp = stp_;
    Stitcher stt(stp.rawFilePAN1,
                 stp.rawFilePAN2,
                 stp.rrcParaPAN1,
                 stp.rrcParaPAN2,
                 stp.sections,
                 stp.sectionLines,
                 stp.overlapCols);
    
    stt.CalcSttParameters();
    
    if (!stp.onlyParamCalc) {
        if (stp.doRRC) stt.DoRRC();
        stt.PreStitch();
    }
}

void DefaultAction() {
    const InputParameters & ip = ips_;
    if (ip.doRRC4PAN && ip.RRCParaPAN.length() == 0) {
        throw std::invalid_argument("RRC parameter file of PAN needed");
    }
    if (ip.doRRC4MSS
        &&(ip.RRCParaMSS[0].length() == 0
        || ip.RRCParaMSS[1].length() == 0
        || ip.RRCParaMSS[2].length() == 0
        || ip.RRCParaMSS[3].length() == 0)) {
        throw std::invalid_argument("RRC parameter file of all MSS Bands needed");
    }
    
    PreProcessor pp(  ip.RawFilePAN
                    , ip.RawFileMSS
                    , ip.RRCParaPAN
                    , ip.RRCParaMSS);
    pp.LoadPAN();
    pp.LoadMSS();
    
    if (ip.doRRC4PAN) {
        pp.DoRRC4PAN();
        if (ip.outputRrcPanTiff) pp.WriteRRCedPAN_TIFF(ip.IBPA_LineOffset);
    }
    
    if (ip.doRRC4MSS) pp.DoRRC4MSS();
    
    pp.CalcInterBandCorrelation(ip.IBCOR_Slices, ip.IBCOR_Sections, ip.IBCOR_Threshold);
    pp.DoInterBandAlignment(ip.IBPA_BatchLines, ip.IBPA_LineOffset, ip.IBPA_OverlapLines);
}

int main(int argc, const char * argv[]) {
    try {
        GDALAllRegister();
        return ParseInputParametersFromCommandLineArguments(argc, argv);
    } catch (std::exception & ex) {
        printf("FATAL ERROR: %s.\n", ex.what());
        return 254;
    } catch (...) {
        printf("UNKOWN FATAL ERROR OCCURED.\n");
        return 1;
    }
}
