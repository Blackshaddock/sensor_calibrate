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

std::vector<std::string>& GetFileNamesFromDir(const std::string& path)
{
	std::vector<std::string> filenames;
	for (const auto& path : bfs::directory_iterator(path))
	{
		if (!valid_file(path.path().string()))
		{
			continue;
		}

		filenames.emplace_back(path.path().string());

	}
	return filenames;
}

bool IsDir(const std::string& path)
{
	if (path.empty())
	{
		LOG(INFO) << "The input file name is empty! ";
		return false;
	}
	if (bfs::is_directory(path))
	{
		return true;
	}
	return false;
}

bool GetFileNamesFromDir(const std::string& path, std::vector<std::string>& filenames)
{
	filenames.clear();
	for (const auto& path : bfs::directory_iterator(path)) {
		if (!valid_file(path.path().string())) {
			continue;
		}

		filenames.emplace_back(path.path().string());
	}
	auto start = filenames.begin(), end = filenames.end();
	std::sort(start, end);
	return true;
}

bool valid_file(const std::string& path)
{
	return true;
}

bool IsExists(const std::string& path)
{
	if (path.empty())
	{
		return false;
	}
	if (bfs::exists(path))
	{
		return true;
	}
	return false;
}



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