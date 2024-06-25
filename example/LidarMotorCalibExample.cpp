#include "module/LidarMotorCalibration.h"
#include "module/LidarSlam.h"
#include "module/ImuAnchorOptimization.h"
#include "internal/CeresFactors.h"
#include <deque>

int main(int argc, char** argv) {
	
	// 指定config文件夹的yaml文件
	if (argc != 3) {
		std::cout << "Please set config file path first." << std::endl;
		system("pause");
		return 0;
	}
	std::cout << argv[2] << " " <<   std::endl;
	std::string runtype = argv[2];
	if (runtype == "slam")
	{
		ls::LidarSlamConfig::Ptr lsCfgPtr(new ls::LidarSlamConfig);
		if (!lsCfgPtr->LoadConfigFile(argv[1]))
		{
			std::cout << "Load config file failed." << std::endl;
			return 0;
		}
		ls::LidarSlam ls(lsCfgPtr);
		if (!ls.Run())
		{
			std::cout << "lidar slam failed." << std::endl;
			return 0;
		}
		while (true)
		{
			std::this_thread::sleep_for(std::chrono::microseconds(1));
		}
		
	}
	else {
		// 读入config参数
		sc::LidarMotorCalibrationConfig::Ptr lmcCfgPtr(new sc::LidarMotorCalibrationConfig);
		if (!lmcCfgPtr->LoadConfigFile(argv[1])) {
			std::cout << "Load config file failed." << std::endl;
			system("pause");
			return 0;
		}

		// 运行标定
		sc::LidarMotorCalibration lmc(lmcCfgPtr);
		if (!lmc.Run()) {
			std::cout << "Calibration failed." << std::endl;
			system("pause");
			return 0;
		}

		std::cout << "Calibration finished." << std::endl;
		
		system("pause");
	}
	//getchar();
	return 0;
}
