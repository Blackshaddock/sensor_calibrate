#pragma once
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

// namespace of sensor calibration
namespace sc {

static double DEG2RAD = 0.0174532925199432;
static double RAD2DEG = 57.295779513082320;
namespace bfs = boost::filesystem;

enum deviceType {
	handDevice = 0,
	normalDevice

};

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

//根据欧拉角获取旋转矩阵
Eigen::Matrix3d GetRFromZYX(double alphax, double alphay, double alphaz);

void GetRFromZYX(double alphax, double alphay, double alphaz, Eigen::Matrix3d &R);


template <typename T>
bool GetData(const YAML::Node& node, T& value) {
	if (!node.IsDefined()) {
		return false;
	}
	value = node.as<T>();
	return true;
}

std::string UtfToGbk(const std::string& strValue);

}// namespace sc