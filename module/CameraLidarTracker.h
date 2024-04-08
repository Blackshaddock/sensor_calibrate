#pragma once

#include "internal/PoseD.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <yaml-cpp/yaml.h>
namespace clt {
using namespace sc;

struct CameraLidarTrackerConfig {
	typedef std::shared_ptr<CameraLidarTrackerConfig> Ptr;

	std::string videoFilePath;			// ��Ƶ�ļ�·��
	std::string lidarFilePath;			// �״��ļ�·��

	Eigen::Vector3d extPosCam2Lidar;      // Cam -> Lidar positin ���
	Eigen::Matrix3d extRotCam2Lidar;      // Cam -> Lidar rotation ���
	PoseD extPoseCam2Lidar;				// ��һ�ָ�ʽ�����

	Eigen::Matrix3d camIntrinsicMat;    // ����ڲ�
	Eigen::Vector4d camDistCoeffs;      // �������

	int mode;							// �����㷨����
	

	bool optExtParam;					// �Ƿ��Ż����
	bool optTimeDelaySeparate;			// �Ƿ�ֿ��Ż�ʱ��ƫ��
	bool optTimeDelayOnlyOne;			// �Ƿ�ֻ�Ż�һ��ʱ��ƫ��

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