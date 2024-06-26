#include "module/LidarCameraCalibration.h"

int main(int argc, char** argv) {
	// 指定config文件夹的yaml文件
	if (argc != 2) {
		std::cout << "Please set config file path first." << std::endl;
		system("pause");
		return 0;
	}

	// 读入config参数
	sc::LidarCameraCalibrationConfig::Ptr lccCfgPtr(new sc::LidarCameraCalibrationConfig);
	if (!lccCfgPtr->LoadConfigFile(argv[1])) {
		std::cout << "Load config file failed." << std::endl;
		system("pause");
		return 0;
	}

	// 运行标定
	sc::LidarCameraCalibration lcc(lccCfgPtr);
	if (!lcc.Run()) {
		std::cout << "Calibration failed." << std::endl;
		system("pause");
		return 0;
	}

	std::cout << "Calibration finished." << std::endl;
	system("pause");

	return 0;
}
