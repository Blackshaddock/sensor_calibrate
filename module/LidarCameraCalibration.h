#pragma once

#include "internal/TrajectoryManager.hpp"
#include "internal/Parameter.hpp"
#include "internal/Structs.h"
#include <map>
#include <memory>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <yaml-cpp/yaml.h>
namespace sc {

struct LidarCameraCalibrationConfig : ParamterBase {
	typedef std::shared_ptr<LidarCameraCalibrationConfig> Ptr;

	std::string lidarTrajPath;			// 雷达轨迹路径
	std::string lidarCloudPath;			// 雷达点云路径
	std::string cameraTimeFilePath;		// 相机时间文件路径
	std::string cameraPicPath;			// 相机照片路径

	Eigen::Vector3d extPosCam2Imu;      // imu -> camera positin 外参
	Eigen::Matrix3d extRotCam2Imu;      // imu -> camera rotation 外参
	PoseD extPoseCam2Imu;				// 换一种格式，外参

	Eigen::Matrix3d camIntrinsicMat;    // 相机内参
	Eigen::Vector4d camDistCoeffs;      // 畸变参数
	double timeDelay;	
	int mode;							// 1 = 照片选点标定；2 = 外参标定
	std::map<int, double> calibTimeVec;	// 每张标定照片对应的时间延迟参数，待优化
	std::vector<int> calibImgIdVec;		// 照片选标定点id

	bool optExtParam;					// 是否优化外参
	bool optTimeDelaySeparate;			// 是否分开优化时间偏移
	bool optTimeDelayOnlyOne;			// 是否只优化一个时间偏移

	LidarCameraCalibrationConfig() : timeDelay(0), optExtParam(true),
									 optTimeDelaySeparate(true),
									 optTimeDelayOnlyOne(false) {}

	bool LoadConfigFile(const std::string& path);

private:
	template <typename T>
	bool GetData(const YAML::Node& node, T& value);
};

struct PnPData {
	Eigen::Vector2i uv;
	Eigen::Vector3f pt3D;
	int imageId;
	double time;
	bool useFlag;
};
typedef std::vector<PnPData> PnPDatas;

class LidarCameraCalibration {
public:
    LidarCameraCalibration(LidarCameraCalibrationConfig::Ptr& config);

    ~LidarCameraCalibration();

	bool Run();

private:
	bool InitWithConfig();

	// 图片选点标定
	bool MarkImagePoint();

	static void HandlerMouseEvent(int event, int x, int y, int flags, void* p);

	bool LoadObservation();

	bool RunCalibration();

	bool SaveOptimizeResult();

	void CheckRecolor();

	void SetCameraPoses(const TrajectoryManager::Ptr& globalTraj);

	void CloudProjection(const BaseCloudPtr& cloudIn, const int imageId, ColorCloudPtr& cloudOut);

private:
	static LidarCameraCalibrationConfig::Ptr configPtr_;
	static std::vector<std::string> texts_;
	static std::vector<cv::Point> points_;
	static int countId_;
	static std::string curImageId_;

    TrajectoryManager::Ptr cameraTrajPtr_;
    TrajectoryManager::Ptr lidarTrajPtr_;

	PnPDatas pnpDatas_;
};
}