//
//  ImageOperations.h
//  OpticalImageProcessor
//
//  Created by Stone PEN on 17/6/22.
//

#ifndef ImageOperations_h
#define ImageOperations_h

#include <sys/stat.h>
#include <gdal_priv.h>

#include "oipshared.h"
#include "toolbox.h"

BEGIN_NS(OIP)

const int REMAP_ROW_GUARD = 32767;
const int REMAP_SECTION_ROWS = 30000;

struct GdalDsDtor {
    inline void operator()(GDALDataset * ds) { if (ds) GDALClose(ds); }
};

struct RRCParam {
    double k;
    double b;
};

class ImageOperations;
typedef class ImageOperations IMO;
class ImageOperations {
//private:
//    ImageOperations() = default;
//public:
//    ImageOperations & get() {
//        static ImageOperations ios;
//        return ios;
//    }

public:
    static size_t FileSize(const std::string & filePath) {
        struct stat st = { 0 };
        if (stat(filePath.c_str(), &st)) throw errno_error("stat() call for file failed");
        return st.st_size;
    }
    
    /// size: read bytes
    /// offset: read-from byte offset
    /// total: bytes designated to read, 0 for all available
    static char * ReadFileContent(const std::string & filePath,
                                  size_t & size,
                                  size_t offset = 0,
                                  size_t total = 0,
                                  char * buff = NULL) {
        scoped_ptr<FILE, FileDtor> f = fopen(filePath.c_str(), "rb");
        if (f.is_null()) throw std::invalid_argument(xs("cannot open file [%s]: %d", filePath.c_str(), errno).s);
        
        size_t want_size = total;
        if (total == 0) {
            if (fseek(f, 0, SEEK_END)) throw std::invalid_argument(xs("ReadFileContent(): seek2end failed: %d", errno).s);
            want_size = ftello(f) - offset;
        }
        if (fseek(f, offset, SEEK_SET)) throw std::invalid_argument(xs("ReadFileContent(): rewind failed: %d", errno).s);
        
        if (buff == NULL) buff = new char[want_size];

        const int unit = 8 * 1024 * 1024; // 8MB
        size_t rb = 0;
        for (char * p = buff;;) {
            auto rn = fread(p, 1, std::min((size_t)unit, (size_t)(want_size - rb)), f);
            p += rn;
            rb += rn;
            if (rn == 0 || rb == want_size) {
                size = rb;
                break;
            }
        }
        
        return buff;
    }
    
    static size_t WriteBufferToFile(const char * buff, size_t size, const std::string & saveFilePath) {
        scoped_ptr<FILE, FileDtor> f = fopen(saveFilePath.c_str(), "wb");
        if (f.is_null()) throw std::runtime_error(xs("open file [%s] failed: %", saveFilePath.c_str(), errno).s);
        
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
    
    static std::string BuildOutputFilePath(const std::string & templatePath,
                                           const std::string & stemExtension,
                                           const char * replaceExtension = NULL) {
        auto cd = std::filesystem::current_path();
        std::filesystem::path tmplPath = templatePath;
        auto outputFilePath = cd / tmplPath.stem();
        outputFilePath += stemExtension;
        outputFilePath += replaceExtension ? replaceExtension : tmplPath.extension();
        return outputFilePath.string();
    }
    
    static void * LoadRawImage(const std::string & filePath,
                               size_t offset = 0,
                               size_t bytes = 0,
                               size_t expectedSize = 0) {
        OLOG("Reading raw image from file `%s' ...", filePath.c_str());
        size_t size = 0;
        stop_watch::rst();
        void * content = ReadFileContent(filePath, size, offset, bytes);
        if (expectedSize > 0 && size != expectedSize) {
            throw std::runtime_error(xs("file size(%lld) doesn't match with read byte count(%lld)", expectedSize, size).s);
        }
        auto es = stop_watch::tik().ellapsed;
        OLOG("%s bytes read in %s seconds (%s MBps).",
             comma_sep(size).sep(),
             comma_sep(es).sep(),
             comma_sep(size/es/1024.0/1024.0).sep());
        return content;
    }
    
    static void InplaceRRC(uint16_t * buff, int w, int h, const RRCParam * rrcParam) {
        for (size_t y = 0; y < h; ++y) {
            for (size_t x = 0; x < w; ++x) {
                size_t idx = w * y + x;
                uint16_t src = buff[idx];
                uint16_t dst = (uint16_t)(rrcParam[x].k * src + rrcParam[x].b);
                buff[idx] = dst;
            }
        }
    }
    
    static RRCParam * LoadRRCParamFile(const char * paramFilePath, int expectedLines) {
        OLOG("Loading RRC paramter from file `%s' ...", paramFilePath);
        scoped_ptr<FILE, FileDtor> f = fopen(paramFilePath, "rb");
        if (f.is_null()) throw errno_error("open RRC Param file failed");
        
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
    
    static uint16_t * DoRRC4RAW(const std::string & raw,
                                int pixelPerLine,
                                const std::string & rrc,
                                const std::string saveRaw = "",
                                bool keepBuffer = false) {
        size_t size = FileSize(raw);
        scoped_ptr<uint16_t> image = (uint16_t *)LoadRawImage(raw, 0, 0, size);
        
        int lines = (int)(size / (pixelPerLine * BYTES_PER_PIXEL));
        scoped_ptr<RRCParam> rrcParam = LoadRRCParamFile(rrc.c_str(), pixelPerLine);
        
        OLOG("Do inplace RRC ...");
        stop_watch::rst();
        InplaceRRC(image, pixelPerLine, lines, rrcParam);
        auto es = stop_watch::tik().ellapsed;
        OLOG("Done for %s bytes in %s seconds (%s MBps).",
             comma_sep(size).sep(),
             comma_sep(es).sep(),
             comma_sep(size/es/(1024.0*1024.0)).sep());

        if (saveRaw.length() > 0) {
            OLOG("Write RRC result as file \"%s\" ...", saveRaw.c_str());
            stop_watch::rst();
            WriteBufferToFile((const char *)image.get(), size, saveRaw);
            auto es = stop_watch::tik().ellapsed;
            OLOG("%s bytes written in %s seconds (%s MBps).",
                 comma_sep(size).sep(),
                 comma_sep(es).sep(),
                 comma_sep(size/es/(1024.0*1024.0)).sep());
        }
        if (keepBuffer) {
            return image.detach();
        }
        return NULL;
    }
    
    static int SectionaryRemap(int total_rows, int upper_cut, int bottom_cut,
                               std::function<cv::Mat(int row_offset, int rows)>get_src,
                               std::function<cv::Mat(int row_offset, int rows)>get_mapx,
                               std::function<cv::Mat(int row_offset, int rows)>get_mapy,
                               std::function<void(const cv::Mat &data, int row_offset)>write_upper,
                               std::function<void(const cv::Mat &data, int row_offset)>write_dst,
                               std::function<void(const cv::Mat &data, int row_offset)>write_bottom,
                               int interpolation = cv::INTER_CUBIC,
                               int border_mode = cv::BORDER_CONSTANT,
                               const cv::Scalar & border_value = cv::Scalar()) {
        
        assert(upper_cut >= 0 && bottom_cut >= 0);
        if (total_rows <= REMAP_ROW_GUARD) {
            throw std::invalid_argument("too few data rows, please use cv::remap()");
        }
        
        int total_cut = upper_cut + bottom_cut;
        int row_offset = 0;
        cv::Mat dst;
        for (int s = 0; ; ++s) {
            int rows = std::min(REMAP_SECTION_ROWS, total_rows - row_offset);
            if (rows <=  total_cut) {
                break;
            }
            
            cv::Mat src  = get_src (row_offset, rows);
            cv::Mat mapx = get_mapx(row_offset, rows);
            cv::Mat mapy = get_mapy(row_offset, rows);
            cv::remap(src, dst, mapx, mapy, interpolation, border_mode, border_value);
            
            if (s == 0 && upper_cut > 0) {
                cv::Mat cut = dst.rowRange(0, upper_cut);
                write_upper(cut, 0);
            }
            
            write_dst(dst.rowRange(upper_cut, rows - bottom_cut), row_offset);
            row_offset += rows - total_cut;
        }
        
        if (bottom_cut > 0) {
            cv::Mat cut = dst.rowRange(dst.rows - bottom_cut, dst.rows);
            write_bottom(cut, row_offset);
        }
        
        return row_offset;
    }

    static std::string StitchBigRaw(const std::string & leftImagePath,
                                    const std::string & rightImagePath,
                                    const std::string & stitchedFilePath,
                                    int pixelPerLine,
                                    int foldColPixels) {
        
        size_t szl = IMO::FileSize( leftImagePath);
        size_t szr = IMO::FileSize(rightImagePath);
        if (szl != szr) {
            throw std::invalid_argument(xs("RAW image sizes not match: left = %s bytes, right = %s bytes",
                                           comma_sep(szl).sep(),
                                           comma_sep(szr).sep()).s);
        }
        
        int bytesPerLine = pixelPerLine * BYTES_PER_PIXEL;
        int imageLines = (int)(szl / bytesPerLine);
        int foldBytes = foldColPixels * BYTES_PER_PIXEL;
        int outputHalfLineBytes = bytesPerLine - foldBytes;
        int outputFullLinePixels = (pixelPerLine - foldColPixels) * 2;
        
        bool outputIsTiff = true;
        std::string outputFilePath = stitchedFilePath;
        if (stitchedFilePath == "") {
            // default to output TIFF
            outputFilePath = (std::filesystem::current_path() /
                              xs("stitched_%dn%db" TIFF_FILE_EXT, outputFullLinePixels, BYTES_PER_PIXEL * 8).s).string();
        } else {
            auto outExt = std::filesystem::path(stitchedFilePath).extension().string();
            outputIsTiff = CLI::detail::to_lower(outExt) == CLI::detail::to_lower(TIFF_FILE_EXT);
        }
        
        scoped_ptr<FILE, FileDtor> fl = fopen( leftImagePath.c_str(), "rb");
        scoped_ptr<FILE, FileDtor> fr = fopen(rightImagePath.c_str(), "rb");
        scoped_ptr<char> lineBuff = new char[bytesPerLine];
        
        std::function<void(void*,int,int,int)> writer;
        scoped_ptr<FILE, FileDtor> fo;
        scoped_ptr<GDALDataset, GdalDsDtor> ds;
        GDALRasterBand * bnd = NULL;
        if (outputIsTiff) {
            GDALDriver * drv = GetGDALDriverManager()->GetDriverByName("GTiff");
            ds = drv->Create(outputFilePath.c_str(), outputFullLinePixels, imageLines, 1, GDT_UInt16, NULL);
            bnd = ds->GetRasterBand(1);
            writer = [&](void * buff, int bytes, int row, int col) {
                if (bnd->RasterIO(GF_Write,
                                  col, row, outputFullLinePixels / 2, 1,
                                  buff,
                                  outputFullLinePixels / 2, 1,
                                  GDT_UInt16, 0, 0) == CE_Failure) {
                    throw errno_error(xs("write stitched image file failed at line %d", row).s);
                }
            };
        } else {
            fo = fopen(outputFilePath.c_str(), "wb");
            writer = [&](void * buff, int bytes, int row, int col) {
                if (fwrite(buff, bytes, 1, fo) == 0) {
                    throw errno_error(xs("write stitched image file failed at line %d", row).s);
                }
            };
        }
        
        OLOG("Begin stitching two images ...");
        stop_watch::rst();
        for (int i = 0; i < imageLines; ++i) {
            if (fread(lineBuff, bytesPerLine, 1, fl) == 0) {
                throw errno_error(xs("read left image file failed at line %d", i).s);
            }
            writer(lineBuff, outputHalfLineBytes, i, 0);
            
            if (fread(lineBuff, bytesPerLine, 1, fr) == 0) {
                throw errno_error(xs("read right image file failed at line %d", i).s);
            }
            // memset(lineBuff, 0, bytesPerLine);
            writer(lineBuff + foldBytes, outputHalfLineBytes, i, outputFullLinePixels / 2);
            
            if ((i + 1) % 10000 == 0) {
                OLOG("%s lines of image data stitched.", comma_sep(i+1).sep());
            }
        }
        auto es = stop_watch::tik().ellapsed;
        OLOG("%s bytes written in %s seconds (%s MBps).",
             comma_sep(szl).sep(),
             comma_sep(es).sep(),
             comma_sep(szl/es/(1024.0*1024.0)).sep());
        
        return stitchedFilePath;
    }
    
    static std::string StitchTiff(const std::string & leftImagePath,
                                  const std::string & rightImagePath,
                                  const std::string & stitchedFilePath,
                                  int foldColPixels,
                                  bool useGDAL = false,
                                  int * bandMap = NULL) {
        
        std::string outputFilePath = stitchedFilePath;
        if (stitchedFilePath == "") {
            outputFilePath = (std::filesystem::current_path() / "stitched" TIFF_FILE_EXT).string();
        } else {
            auto outExt = std::filesystem::path(stitchedFilePath).extension().string();
            if (CLI::detail::to_lower(outExt) != CLI::detail::to_lower(TIFF_FILE_EXT)) {
                throw std::invalid_argument("Output file should be a tiff image");
            }
        }
        
        size_t szl = IMO::FileSize( leftImagePath);
        size_t szr = IMO::FileSize(rightImagePath);
        //const size_t MAX4G = 4000000000;
        //if (szl > MAX4G || szr > MAX4G) {
        //    throw std::argument_error("image size too large");
        //}
        OLOG("Reading tiff image from file `%s' ...", leftImagePath.c_str());
        stop_watch::rst();
        cv::Mat imageL = cv::imread( leftImagePath, cv::IMREAD_UNCHANGED /*cv::IMREAD_LOAD_GDAL*/);
        auto es = stop_watch::tik().ellapsed;
        OLOG("Image size: %d cols, %d rows, type: %d, channels: %d.",
             imageL.cols, imageL.rows,
             imageL.type(), imageL.channels());
        OLOG("%s bytes read in %s seconds (%s MBps).",
             comma_sep(szl).sep(),
             comma_sep(es).sep(),
             comma_sep(szl/es/1024.0/1024.0).sep());
        
        OLOG("Reading tiff image from file `%s' ...", rightImagePath.c_str());
        stop_watch::rst();
        cv::Mat imageR = cv::imread(rightImagePath, cv::IMREAD_UNCHANGED /*cv::IMREAD_LOAD_GDAL*/);
        es = stop_watch::tik().ellapsed;
        OLOG("Image size: %d cols, %d rows, type: %d, channels: %d.",
             imageR.cols, imageR.rows,
             imageR.type(), imageR.channels());
        OLOG("%s bytes read in %s seconds (%s MBps).",
             comma_sep(szr).sep(),
             comma_sep(es).sep(),
             comma_sep(szr/es/1024.0/1024.0).sep());

        if (imageL.rows != imageR.rows || imageL.cols != imageR.cols) {
            throw std::runtime_error("images have different sizes");
        }
        
        int outputHalfLinePixels = imageL.cols - foldColPixels;
        int outputFullLinePixels = outputHalfLinePixels * 2;
        if (szl < 4000000000 && !useGDAL) { // around 4GB
            cv::Mat stitchedImage(imageL.rows, outputFullLinePixels, CV_16UC4);
            cv::Mat stitchLeft = imageL.colRange(0, outputHalfLinePixels);
            cv::Mat stitchRiht = imageR.colRange(foldColPixels, imageR.cols);
            
            size_t copyBytes = (size_t)imageL.rows * outputHalfLinePixels * BYTES_PER_PIXEL * imageL.channels();
            OLOG("Copying left part image data ...");
            stop_watch::rst();
            stitchLeft.copyTo(stitchedImage.colRange(0, outputHalfLinePixels));
            es = stop_watch::tik().ellapsed;
            OLOG("%s bytes copied in %s seconds (%s MBps).",
                 comma_sep(copyBytes).sep(),
                 comma_sep(es).sep(),
                 comma_sep(copyBytes/es/1024.0/1024.0).sep());

            OLOG("Copying right part image data ...");
            stop_watch::rst();
            stitchRiht.copyTo(stitchedImage.colRange(outputHalfLinePixels, stitchedImage.cols));
            es = stop_watch::tik().ellapsed;
            OLOG("%s bytes copied in %s seconds (%s MBps).",
                 comma_sep(copyBytes).sep(),
                 comma_sep(es).sep(),
                 comma_sep(copyBytes/es/1024.0/1024.0).sep());
            
            OLOG("Write stitched image to file '%s' ...", outputFilePath.c_str());
            stop_watch::rst();
            if (!cv::imwrite(outputFilePath, stitchedImage)) {
                throw std::runtime_error("Writing stitched image as TIFF failed");
            }
            es = stop_watch::tik().ellapsed;
            OLOG("%s bytes written in %s seconds (%s MBps).",
                 comma_sep(copyBytes*2).sep(),
                 comma_sep(es).sep(),
                 comma_sep(copyBytes*2/es/1024.0/1024.0).sep());
        } else {
            StitchTiffGDAL(imageL, imageR, outputFilePath, foldColPixels, bandMap);
        }
        
        return outputFilePath;
    }
    
private:
    static void StitchTiffGDAL(cv::Mat & imageL,
                               cv::Mat & imageR,
                               const std::string & outputImagePath,
                               int foldColPixels,
                               int * bandMap = NULL,
                               bool setBandInterpretion = false) {
        int imageLines = imageL.rows;
        int outputHalfLinePixels = imageL.cols - foldColPixels;
        int outputFullLinePixels = outputHalfLinePixels * 2;
        
        char ** options = CSLParseCommandLine("");
        options = CSLSetNameValue(options, "COMPRESS", "LZW");
        options = CSLSetNameValue(options, "PREDICTOR", "2");
        options = CSLSetNameValue(options, "NUM_THREADS", "ALL_CPUS");
        options = CSLSetNameValue(options, "PHOTOMETRIC", "RGB");
        GDALDriver * drv = GetGDALDriverManager()->GetDriverByName("GTiff");
        scoped_ptr<GDALDataset, GdalDsDtor> ds = drv->Create(outputImagePath.c_str(),
                                                             outputFullLinePixels,
                                                             imageLines,
                                                             MSS_BANDS, GDT_UInt16, options);
        CSLDestroy(options);
        
        int sections = (imageLines - 1) / IBPA_DEFAULT_BATCHLINES + 1;
        int processedLines = 0;
        cv::Mat imageFullSection(IBPA_DEFAULT_BATCHLINES, outputFullLinePixels, CV_16UC4);
        cv::Mat splitBands[MSS_BANDS];
        cv::Mat splitConts[MSS_BANDS];

        stop_watch::rst();
        for (int s = 0; s < sections; ++s) {
            int sectionLines = std::min(imageLines - processedLines, IBPA_DEFAULT_BATCHLINES);
            
            cv::Range rowRng(processedLines, processedLines + sectionLines);
            cv::Range colRngL(0, outputHalfLinePixels);
            cv::Range colRngR(foldColPixels, imageR.cols);
            
            cv::Range rowRngT(0, sectionLines);
            cv::Range colRngTL = colRngL;
            cv::Range colRngTR(outputHalfLinePixels, outputFullLinePixels);
            
            OLOG("Merging 2 CMOS image data part %d/%d ...", s+1, sections);
            auto sectionL = imageL(rowRng, colRngL);
            auto sectionR = imageR(rowRng, colRngR);
            auto destL = imageFullSection(rowRngT, colRngTL);
            auto destR = imageFullSection(rowRngT, colRngTR);
            sectionL.copyTo(destL);
            sectionR.copyTo(destR);
            
            GDALColorInterp bandIntp[MSS_BANDS] = {
                GCI_RedBand,
                GCI_GreenBand,
                GCI_BlueBand,
                GCI_AlphaBand,
            };
            
            OLOG("Writing to TIFF image file ...");
            
            cv::split(imageFullSection, splitBands);
            for (int b = 0; b < MSS_BANDS; ++b) {
                if (splitBands[b].isContinuous()) {
                    splitConts[b] = splitBands[b];
                } else {
                    splitConts[b] = splitBands[b].clone();
                    OLOG("Cloned band #%d of section #%d.", b+1, s);
                }
            }
            for (int b = 0; b < MSS_BANDS; ++b) {
                GDALRasterBand * bnd = ds->GetRasterBand(1+b);
                if (setBandInterpretion) bnd->SetColorInterpretation(bandIntp[b]);
                int mappedBand = bandMap ? bandMap[b] - 1 : b;
                if (bnd->RasterIO(GF_Write,
                                  0, processedLines, outputFullLinePixels, sectionLines,
                                  (void *)splitConts[mappedBand].data,
                                  outputFullLinePixels, sectionLines,
                                  GDT_UInt16, 0, 0) == CE_Failure) {
                    throw errno_error(xs("write stitched image file failed at line %d of band #%d",
                                         processedLines, b).s);
                }
            }

            /*//
            for (int b = 0; b < MSS_BANDS; ++b) {
                GDALRasterBand * bnd = ds->GetRasterBand(1+b);
                if (setBandInterpretion) bnd->SetColorInterpretation(bandIntp[b]);
                if (bnd->RasterIO(GF_Write,
                                  0, processedLines, outputFullLinePixels, sectionLines,
                                  (void *)((char *)imageFullSection.data + b * BYTES_PER_PIXEL),
                                  outputFullLinePixels, sectionLines,
                                  GDT_UInt16,
                                  BYTES_PER_PIXEL * MSS_BANDS,
                                  outputFullLinePixels * BYTES_PER_PIXEL * MSS_BANDS) == CE_Failure) {
                    throw errno_error(xs("write stitched image file failed at line %d of band #%d",
                                         processedLines, b).s);
                }
            }//*/
            
            processedLines += sectionLines;
            OLOG("%s lines of image data stitched.", comma_sep(processedLines).sep());
        }
        
        size_t totalBytes = (size_t)imageLines * outputFullLinePixels * MSS_BANDS * BYTES_PER_PIXEL;
        auto es = stop_watch::tik().ellapsed;
        OLOG("Merged TIFF image file '%s' generated.", outputImagePath.c_str());
        OLOG("%s bytes processed in %s seconds (%s MBps).",
             comma_sep(totalBytes).sep(),
             comma_sep(es).sep(),
             comma_sep(totalBytes/es/1024.0/1024.0).sep());
    }
};

END_NS

#endif /* ImageOperations_h */
