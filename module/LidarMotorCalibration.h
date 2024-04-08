#pragma once
#include "internal/BagManager.h"
#include <yaml-cpp/yaml.h>

// namespace of sensor calibration
namespace sc {

struct LidarMotorCalibrationConfig {
public:
	typedef std::shared_ptr<LidarMotorCalibrationConfig> Ptr;

	BagManagerConfig::Ptr bagManagerCfgPtr;					// bag解码参数设置

	MotorCalibrationParam::Ptr motorCalibParam;				// 待优化参数，编码器内参，角度为度

	MotorAngleCorrectParam::Ptr motorAngleCorrectParamPtr;	// 待优化参数，360个编码器角度修正值，度

	std::vector<int> calibLaserIdVec1;						// 用于标定360个角度修正值的laser id
	std::vector<int> calibLaserIdVec2;						// 用于标定编码器大于0 和 小于0 的laser id

	bool optimizeMotorIntrisincParam;
	bool optimizeMotorAngleCorrParam;
	bool optimizeLidarIntrisincParam;

	LidarMotorCalibrationConfig() : optimizeLidarIntrisincParam(true),
									optimizeMotorAngleCorrParam(true), 
									optimizeMotorIntrisincParam(true) {}

	bool LoadConfigFile(const std::string& path);

	bool IsValid() const { return bagManagerCfgPtr->IsValid(); }

private:
	template <typename T>
	bool GetData(const YAML::Node& node, T& value);
};

class LidarMotorCalibration {
public:
	LidarMotorCalibration(const LidarMotorCalibrationConfig::Ptr& configPtr);

	~LidarMotorCalibration();

	bool Run();

private:
	// 参数初始化
	bool InitWithConfig();

	// 数据预处理，将bag解码的原始点云数据进行坐标转换
	bool DataPrepare();
	
	// 读取txt、pcd等格式数据作为原始数据进行坐标转换
	bool DataPrepare4geosun();

	// 使用激光器内参校正bag原始解码的点云
	bool CorrectPointUseLidarIntrParam(BaseCloudPtr& cloud);

	// 将bag解析出的原始点云数据，转换到当前帧时刻编码器独立坐标系，仅转换lidar轴系
	void ConvertToMotorLocalCoordinate(BaseCloudPtr& cloud);

	// 将所有原始点数据转换到当前帧时刻编码器独立坐标系L-M系
	void ConvertToMotorLocalCoordinate(BasePoint& pt);

	// 结合编码器角度值，内插每个点所处的编码器角度，转换到编码器全局坐标系
	bool ConvertToMotorGlobalCoordinate(const BaseCloudPtr& cloudIn, BaseCloudPtr& cloudOut);

	//
	bool ConvertToMotorGlobalCoordinate4geosun(const BaseCloudPtr& cloudIn, BaseCloudPtr& cloudOut);

	// 抽取特定ring点云
	void ExtractLaserLineCloud(const BaseCloudPtr& cloudLocal, const BaseCloudPtr& cloudGlobal);

	// 标定
	bool RunCalibration();

	// 从全部点云中，按照id抽取标定点云
	bool ExtractCalibrationCloud(BaseCloudPtr& fullCloud, BaseCloudPtr& sourceCloud,
								 BaseCloudPtr& targetCloud, const std::vector<int>& calibIdVec,
								 bool keepLarge0 = false);

	// 计算法向
	void GetNormal(BaseCloudPtr &cloud, std::vector<std::vector<int>>& indexes);

	// 根据带法向的点云，计算联通平面点云
	bool ExtractMatchingPairs(BaseCloudPtr& fullCloud, const std::vector<int>& calibIdVec,
							  MotorCalibMatchPairs& matchPairs, bool keepLarge0 = false);


	// icp获取同名点
	bool GetCorrespondPointIndex(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud,
		const BaseCloudPtr& matchPairs);


	// 特征关联
	bool FeatureAssociation(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud, const BaseCloudPtr& cloudIn,
							MotorCalibMatchPairs& matchPairs);

	bool FeatureAssociation4geosun(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud, const BaseCloudPtr&  sourceCloudDs, const BaseCloudPtr& targetCloudDs, const BaseCloudPtr& cloudIn,MotorCalibMatchPairs& matchPairs);

	// 保存优化结果
	bool SaveOptimizeResult();

	// 验证标定结果，重新解算点云
	bool CheckOptimizeResult();

private:
	LidarMotorCalibrationConfig::Ptr configPtr_;

	BagManager::Ptr bagManagerPtr_;
	MotorManager::Ptr motorManagerPtr_;

	bool debugFlag_;
	std::string debugOptDataPath_;
	std::string debugOptResultPath_;
	std::string debugTestPath_;
	
	
	int threadNum_;
};

}// namespace sc
