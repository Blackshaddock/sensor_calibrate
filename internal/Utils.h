#pragma once
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

// namespace of sensor calibration
namespace sc {

static double DEG2RAD = 0.0174532925199432;
static double RAD2DEG = 57.295779513082320;
namespace bfs = boost::filesystem;

bool CreateDir(const std::string& path);

std::string GetFileName(const std::string& path);

std::string GetRootDirectory(const std::string& path);

std::string GetFileExtention(const std::string& path);

//template <typename T>
//bool GetData(const YAML::Node& node, T& value);

std::string UtfToGbk(const std::string& strValue);

}// namespace sc