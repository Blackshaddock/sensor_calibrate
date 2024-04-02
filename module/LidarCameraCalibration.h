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

	std::string lidarTrajPath;			// �״�켣·��
	std::string lidarCloudPath;			// �״����·��
	std::string cameraTimeFilePath;		// ���ʱ���ļ�·��
	std::string cameraPicPath;			// �����Ƭ·��

	Eigen::Vector3d extPosCam2Imu;      // imu -> camera positin ���
	Eigen::Matrix3d extRotCam2Imu;      // imu -> camera rotation ���
	PoseD extPoseCam2Imu;				// ��һ�ָ�ʽ�����

	Eigen::Matrix3d camIntrinsicMat;    // ����ڲ�
	Eigen::Vector4d camDistCoeffs;      // �������
	double timeDelay;	
	int mode;							// 1 = ��Ƭѡ��궨��2 = ��α궨
	std::map<int, double> calibTimeVec;	// ÿ�ű궨��Ƭ��Ӧ��ʱ���ӳٲ��������Ż�
	std::vector<int> calibImgIdVec;		// ��Ƭѡ�궨��id

	bool optExtParam;					// �Ƿ��Ż����
	bool optTimeDelaySeparate;			// �Ƿ�ֿ��Ż�ʱ��ƫ��
	bool optTimeDelayOnlyOne;			// �Ƿ�ֻ�Ż�һ��ʱ��ƫ��

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

	// ͼƬѡ��궨
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