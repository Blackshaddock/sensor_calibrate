#include "module/LidarCameraCalibration.h"

int main(int argc, char** argv) {
	// ָ��config�ļ��е�yaml�ļ�
	if (argc != 2) {
		std::cout << "Please set config file path first." << std::endl;
		system("pause");
		return 0;
	}

	// ����config����
	sc::LidarCameraCalibrationConfig::Ptr lccCfgPtr(new sc::LidarCameraCalibrationConfig);
	if (!lccCfgPtr->LoadConfigFile(argv[1])) {
		std::cout << "Load config file failed." << std::endl;
		system("pause");
		return 0;
	}

	// ���б궨
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
