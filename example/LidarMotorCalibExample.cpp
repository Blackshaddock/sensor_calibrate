#include "module/LidarMotorCalibration.h"
#include "module/LidarSlam.h"
#include "module/ImuAnchorOptimization.h"
#include "color_point/color_point.h"
#include "internal/CeresFactors.h"
#include <deque>

int main(int argc, char** argv) {
	// ָ��config�ļ��е�yaml�ļ�
	if (argc != 3) {
		std::cout << "Please set config file path first." << std::endl;
		system("pause");
		return 0;
	}
	/*Eigen::Matrix3d tmp;
	tmp = GetRFromZYX(0.8054, 0.0105, -0.2385) * GetRFromZYX(40, 0, 0);
	std::cout << tmp << std::endl;
	std::cout << argv[2] << " " <<   std::endl;*/
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
	else if (runtype == "color")
	{
		ColoredPoint::Ptr colorPtr(new ColoredPoint());
		colorPtr->LoadConf(argv[1]);
		colorPtr->Run();

	}
	else {
		// ����config����
		sc::LidarMotorCalibrationConfig::Ptr lmcCfgPtr(new sc::LidarMotorCalibrationConfig);
		if (!lmcCfgPtr->LoadConfigFile(argv[1])) {
			std::cout << "Load config file failed." << std::endl;
			system("pause");
			return 0;
		}

		// ���б궨
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
