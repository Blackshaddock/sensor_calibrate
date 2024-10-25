#ifndef __UTILS_H__
#define __UTILS_H__


#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
// namespace of sensor calibration
namespace geosun {
namespace bfs = boost::filesystem;


struct sort_function {
    bool operator()(const std::string& str_1, const std::string& str_2)
    {
        return str_1 < str_2;
    }

};

std::vector<std::string> split(const std::string& str_in, char delim = ' ');

bool CreateDir(const std::string& path);

std::string GetFileName(const std::string& path);

std::string GetRootDirectory(const std::string& path);

std::string GetFileExtention(const std::string& path);


std::vector<std::string> GetFileNamesFromDir(const std::string& path);

bool IsDir(const std::string& path);

bool GetFileNamesFromDir(const std::string& path, std::vector<std::string>& filenames);

bool valid_file(const std::string& path);

bool IsExists(const std::string& path);


//获取磁盘的信息，包括剩余空间，总空间等
bfs::space_info DiskInfo(const std::string& path);

//获取文件大小,单位Kb
double FileSize(const std::string& path);


//十六进制转string

inline std::string binaryToHex(const std::vector<uint8_t>& binary) {
    std::ostringstream ss;
    for (uint8_t byte : binary) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)byte;
    }
    return ss.str();
}

inline std::string binaryToHex(const uint8_t& binary) {
    std::ostringstream ss;
    
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)binary;
    
    return ss.str();
}


}// namespace geosun
#endif // __UTILS_H__