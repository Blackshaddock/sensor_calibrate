#include "module/CameraLidarTracker.h"

int main(int argc, char** argv) {
	// ָ��config�ļ��е�yaml�ļ�
	printf(CV_VERSION);
	if (argc != 2) {
		std::cout << "Please set config file path first." << std::endl;
		system("pause");
		return 0;
	}

	// ����config����
	clt::CameraLidarTrackerConfig::Ptr lmcCfgPtr(new clt::CameraLidarTrackerConfig);
	if (!lmcCfgPtr->LoadConfigFile(argv[1])) {
		std::cout << "Load config file failed." << std::endl;
		system("pause");
		return 0;
	}

	clt::CameraLidarTracker lmc(lmcCfgPtr);
	if (!lmc.Run()) {
		std::cout << "Calibration failed." << std::endl;
		system("pause");
		return 0;
	}

	std::cout << "Calibration finished." << std::endl;
	system("pause");

	return 0;
}
