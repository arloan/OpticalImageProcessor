//
//  ImageOperations.h
//  OpticalImageProcessor
//
//  Created by Stone PEN on 17/6/22.
//

#ifndef ImageOperations_h
#define ImageOperations_h

#include <sys/stat.h>

#include "oipshared.h"
#include "toolbox.h"

BEGIN_NS(OIP)

const int REMAP_MIN_ROWS = 5;
const int REMAP_ROW_GUARD = 32767;
const int REMAP_SECTION_ROWS = 30000;

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
        if (f.isNull()) throw std::invalid_argument(xs("cannot open file [%s]: %d", filePath.c_str(), errno).s);
        
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
        if (f.isNull()) throw std::runtime_error(xs("open file [%s] failed: %", saveFilePath.c_str(), errno).s);
        
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
        if (f.isNull()) throw errno_error("open RRC Param file failed");
        
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
                               std::function<void(cv::Mat)>concat_dst,
                               int interpolation,
                               int border_mode = cv::BORDER_CONSTANT,
                               const cv::Scalar & border_value = cv::Scalar()) {
        
        if (total_rows <= REMAP_ROW_GUARD) {
            throw std::invalid_argument("too few data rows, please use cv::remap()");
        }
        
        int total_cut = upper_cut + bottom_cut;
        int row_offset = 0;
        for (int s = 0; ; ++s) {
            int rows = std::min(REMAP_SECTION_ROWS, total_rows - row_offset);
            if (rows < REMAP_MIN_ROWS + total_cut) {
                break;
            }
            
            cv::Mat src  = get_src (row_offset, rows);
            cv::Mat mapx = get_mapx(row_offset, rows);
            cv::Mat mapy = get_mapy(row_offset, rows);
            cv::Mat dst;
            cv::remap(src, dst, mapx, mapy, interpolation, border_mode, border_value);
            
            concat_dst(dst.rowRange(upper_cut, rows - bottom_cut));
            row_offset += rows - total_cut;
        }
        return row_offset - upper_cut;
    }
};

END_NS

#endif /* ImageOperations_h */
