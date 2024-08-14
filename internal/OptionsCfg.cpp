#include "internal/OptionsCfg.h"

bool slamBaseOptionsCfg::LoadConfigFile(const std::string& path)
{
	return false;
}
template<typename T>
inline bool slamBaseOptionsCfg::GetData(const YAML::Node& node, T& value)
{
	if (!node.IsDefined()) {
		return false;
	}
	value = node.as<T>();
	return true;
}
