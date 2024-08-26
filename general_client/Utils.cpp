#include "Utils.h"

namespace geosun {
	std::vector<std::string> split(const std::string& str_in, char delim)
	{
		std::vector<std::string> tokens;
		std::stringstream ss(str_in);
		std::string item;
		while (std::getline(ss, item, delim)) {
			if (!item.empty()) {
				tokens.push_back(item);
			}
		}
		return tokens;
	}
	bool CreateDir(const std::string& path) {
	if (!bfs::exists(path)) {
		boost::system::error_code ec;
		if (!bfs::create_directory(path, ec)) {
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

std::vector<std::string> GetFileNamesFromDir(const std::string& path)
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
	std::sort(filenames.begin(), filenames.end());
	return filenames;
}

bool IsDir(const std::string& path)
{
	if (path.empty())
	{
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


}// namespace geosun