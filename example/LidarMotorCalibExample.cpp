#include "module/LidarMotorCalibration.h"
#include "module/LidarSlam.h"
#include "module/ImuAnchorOptimization.h"
#include "color_point/color_point.h"
#include "internal/CeresFactors.h"
#include <deque>

void GetRLidar2IMU()
{
	Eigen::Vector3d alpha(0.3430, 0.0257, -0.1612);
	double cx = cos(alpha[0] * DEG2RAD), sx = sin(alpha[0] * DEG2RAD);
	double cy = cos(alpha[1] * DEG2RAD), sy = sin(alpha[1] * DEG2RAD);
	double cz = cos(alpha[2] * DEG2RAD), sz = sin(alpha[2] * DEG2RAD);
	Eigen::Matrix3d a;
	a << cy * cz, cy* sz, -sy,
		-cx * sz + sx * sy * cz, cx* cz + sx * sy * sz, sx* cy,
		sx* sz + cx * sy * cz, -sx * cz + cx * sy * sz, cx* cy;
	Eigen::Matrix3d b;
	b << 1, 0, 0, 0, 0.7660444, 0.6427876, 0, -0.6427876, 0.7660444;
	a = a * b;
	std::cout << a << std::endl;
}
int main(int argc, char** argv) {
	
	GetRLidar2IMU();
	// 指定config文件夹的yaml文件
	if (argc != 3) {
		std::cout << "Please set config file path first." << std::endl;
		system("pause");
		return 0;
	}
	Eigen::Matrix3d tmp;
	tmp = GetRFromZYX(-5.0000000000000003e-02, -3.7000000000000000e-01,
		5.0000000000000000e-01) * GetRFromZYX(-3.3495100000000001e+01, -3.2014000000000003e+01,
		2.4684000000000001e+01);
	std::cout << tmp << std::endl;
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
	else if (runtype == "color")
	{
		ColoredPoint::Ptr colorPtr(new ColoredPoint());
		colorPtr->LoadConf(argv[1]);
		colorPtr->Run();

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
