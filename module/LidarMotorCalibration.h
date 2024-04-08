#pragma once
#include "internal/BagManager.h"
#include <yaml-cpp/yaml.h>

// namespace of sensor calibration
namespace sc {

struct LidarMotorCalibrationConfig {
public:
	typedef std::shared_ptr<LidarMotorCalibrationConfig> Ptr;

	BagManagerConfig::Ptr bagManagerCfgPtr;					// bag�����������

	MotorCalibrationParam::Ptr motorCalibParam;				// ���Ż��������������ڲΣ��Ƕ�Ϊ��

	MotorAngleCorrectParam::Ptr motorAngleCorrectParamPtr;	// ���Ż�������360���������Ƕ�����ֵ����

	std::vector<int> calibLaserIdVec1;						// ���ڱ궨360���Ƕ�����ֵ��laser id
	std::vector<int> calibLaserIdVec2;						// ���ڱ궨����������0 �� С��0 ��laser id

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
	// ������ʼ��
	bool InitWithConfig();

	// ����Ԥ������bag�����ԭʼ�������ݽ�������ת��
	bool DataPrepare();
	
	// ��ȡtxt��pcd�ȸ�ʽ������Ϊԭʼ���ݽ�������ת��
	bool DataPrepare4geosun();

	// ʹ�ü������ڲ�У��bagԭʼ����ĵ���
	bool CorrectPointUseLidarIntrParam(BaseCloudPtr& cloud);

	// ��bag��������ԭʼ�������ݣ�ת������ǰ֡ʱ�̱�������������ϵ����ת��lidar��ϵ
	void ConvertToMotorLocalCoordinate(BaseCloudPtr& cloud);

	// ������ԭʼ������ת������ǰ֡ʱ�̱�������������ϵL-Mϵ
	void ConvertToMotorLocalCoordinate(BasePoint& pt);

	// ��ϱ������Ƕ�ֵ���ڲ�ÿ���������ı������Ƕȣ�ת����������ȫ������ϵ
	bool ConvertToMotorGlobalCoordinate(const BaseCloudPtr& cloudIn, BaseCloudPtr& cloudOut);

	//
	bool ConvertToMotorGlobalCoordinate4geosun(const BaseCloudPtr& cloudIn, BaseCloudPtr& cloudOut);

	// ��ȡ�ض�ring����
	void ExtractLaserLineCloud(const BaseCloudPtr& cloudLocal, const BaseCloudPtr& cloudGlobal);

	// �궨
	bool RunCalibration();

	// ��ȫ�������У�����id��ȡ�궨����
	bool ExtractCalibrationCloud(BaseCloudPtr& fullCloud, BaseCloudPtr& sourceCloud,
								 BaseCloudPtr& targetCloud, const std::vector<int>& calibIdVec,
								 bool keepLarge0 = false);

	// ���㷨��
	void GetNormal(BaseCloudPtr &cloud, std::vector<std::vector<int>>& indexes);

	// ���ݴ�����ĵ��ƣ�������ͨƽ�����
	bool ExtractMatchingPairs(BaseCloudPtr& fullCloud, const std::vector<int>& calibIdVec,
							  MotorCalibMatchPairs& matchPairs, bool keepLarge0 = false);


	// icp��ȡͬ����
	bool GetCorrespondPointIndex(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud,
		const BaseCloudPtr& matchPairs);


	// ��������
	bool FeatureAssociation(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud, const BaseCloudPtr& cloudIn,
							MotorCalibMatchPairs& matchPairs);

	bool FeatureAssociation4geosun(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud, const BaseCloudPtr&  sourceCloudDs, const BaseCloudPtr& targetCloudDs, const BaseCloudPtr& cloudIn,MotorCalibMatchPairs& matchPairs);

	// �����Ż����
	bool SaveOptimizeResult();

	// ��֤�궨��������½������
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
