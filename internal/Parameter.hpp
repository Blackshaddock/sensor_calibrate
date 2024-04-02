#pragma once
#include <string>

// namespace of sensor calibration
namespace sc {

struct ParamterBase {
	bool debugFlag;
	std::string debugRootDir;
	int threadNum_;

	ParamterBase() : debugFlag(false), threadNum_(1) {}
};
}// namespace sc