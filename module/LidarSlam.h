#ifndef __LIDARSLAM_H__
#define __LIDARSLAM_H__
#include "../scan/scan.h"


namespace ls {
	using namespace scanframe;
	struct LidarSlamConfig
	{
		typedef std::shared_ptr<LidarSlamConfig> Ptr;
		SenSorParamsConfig::Ptr sensorParamsConfigPtr;
		ScanFrameConfig::Ptr scanFrameConfigPtr;
		bool LoadConfigFile(const std::string& path);
	};
	
	class LidarSlam {
	public:
		typedef std::shared_ptr<LidarSlam> Ptr;
		LidarSlam();
		LidarSlam(const LidarSlamConfig::Ptr lidarSlamConfig);
		bool Run();

	private:
		ScanFrame::Ptr scanFrame_;
	};
}


#endif // !__LIDARSLAM_H__
