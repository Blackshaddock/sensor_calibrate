#pragma once
#include "internal/Utils.h"
#include <glog/logging.h>
#include <Windows.h>

// namespace of sensor calibration
namespace sc {

bool CreateDir(const std::string& path) {
	if (!bfs::exists(path)) {
		boost::system::error_code ec;
		if (!bfs::create_directory(path, ec)) {
			LOG(INFO) << "Failed to create directory: "
					  << path << ", message: " << ec.message();
			return false;
		}
	}
	return true;
}

std::string GetFileName(const std::string& path) {
	bfs::path pathTmp(path);
	return pathTmp.stem().generic_string();
}

std::string GetRootDirectory(const std::string& path) {
	bfs::path pathTmp(path);
	return pathTmp.parent_path().generic_string();
}

std::string GetFileExtention(const std::string& path) {
	bfs::path pathTmp(path);
	return pathTmp.extension().generic_string();
}

//template <typename T>
//bool GetData(const YAML::Node& node, T& value) {
//	if (!node.IsDefined()) {
//		return false;
//	}
//	value = node.as<T>();
//	return true;
//}

std::string UtfToGbk(const std::string& strValue) {
	int len = MultiByteToWideChar(CP_UTF8, 0, strValue.c_str(), -1, NULL, 0);
	wchar_t* wstr = new wchar_t[len + 1];
	memset(wstr, 0, len + 1);
	MultiByteToWideChar(CP_UTF8, 0, strValue.c_str(), -1, wstr, len);
	len = WideCharToMultiByte(CP_ACP, 0, wstr, -1, NULL, 0, NULL, NULL);
	char* str = new char[len + 1];
	memset(str, 0, len + 1);
	WideCharToMultiByte(CP_ACP, 0, wstr, -1, str, len, NULL, NULL);
	if (wstr) delete[] wstr;
	return std::string(str);
}

}// namespace sc