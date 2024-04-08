#pragma once

#include "internal/PoseD.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <yaml-cpp/yaml.h>
namespace clt {
using namespace sc;

struct CameraLidarTrackerConfig {
	typedef std::shared_ptr<CameraLidarTrackerConfig> Ptr;

	std::string videoFilePath;			// 视频文件路径
	std::string lidarFilePath;			// 雷达文件路径

	Eigen::Vector3d extPosCam2Lidar;      // Cam -> Lidar positin 外参
	Eigen::Matrix3d extRotCam2Lidar;      // Cam -> Lidar rotation 外参
	PoseD extPoseCam2Lidar;				// 换一种格式，外参

	Eigen::Matrix3d camIntrinsicMat;    // 相机内参
	Eigen::Vector4d camDistCoeffs;      // 畸变参数

	int mode;							// 跟踪算法类型
	

	bool optExtParam;					// 是否优化外参
	bool optTimeDelaySeparate;			// 是否分开优化时间偏移
	bool optTimeDelayOnlyOne;			// 是否只优化一个时间偏移

	CameraLidarTrackerConfig() : mode(0) {}

	bool LoadConfigFile(const std::string& path);

private:
	template <typename T>
	bool GetData(const YAML::Node& node, T& value);
};




class CameraLidarTracker{
public:
	CameraLidarTracker(CameraLidarTrackerConfig::Ptr& config);

    ~CameraLidarTracker();

	bool Run();

private:
	bool InitWithConfig();


private:
	static CameraLidarTrackerConfig::Ptr configPtr_;
};
}