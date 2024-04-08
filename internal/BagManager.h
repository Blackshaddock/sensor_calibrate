#pragma once
#include "internal/MotorManager.h"
#include "internal/Structs.h"
#include "Bag2File.h"
#include "internal/Parameter.hpp"
#include "internal/Utils.h"
#include <thread>
#include <mutex>
#include <deque>
#include <string>
#include <iomanip>

// namespace of sensor calibration
namespace sc {

struct LidarIntrinsicParam {
	typedef std::shared_ptr<LidarIntrinsicParam>	Ptr;
	double** parameter_;

	LidarIntrinsicParam(const int laserNum) {
		parameter_ = new double*[laserNum];
		for (int i = 0; i < laserNum; i++) {
			parameter_[i] = new double[3];
			for (int j = 0; j < 3; j++) {
				parameter_[i][j] = 0;
			}
		}
	}

	~LidarIntrinsicParam() {
		delete parameter_;
	}

	double** GetParam() { return parameter_; }

	bool Load(const std::string& path) {
		std::ifstream ifs(path);
		if (!ifs.is_open()) {
			return false;
		}
		int countId = 0;
		std::string line, elem;
		std::getline(ifs, line);
		while (std::getline(ifs, line)) {
			std::stringstream ss(line);
			std::vector<std::string> elems;
			while (std::getline(ss, elem, ',')) {
				elems.emplace_back(elem);
			}

			if (elems.size() != 3) {
				continue;
			}

			for (int i = 0; i < 3; i++) {
				parameter_[countId][i] = std::stod(elems[i]);
			}
		}

		return countId >= 16;
	}

	bool Save(const std::string& path) {
		std::ofstream ofs(path);
		if (!ofs.is_open()) {
			return false;
		}
		
		ofs << "vertical(deg), horizontal(deg), distance(m)" << std::endl;
		ofs << std::fixed << std::setprecision(6);
		for (int i = 0; i < 16; i++ ) {
			ofs << parameter_[i][0] * RAD2DEG << ", " << parameter_[i][1] * RAD2DEG
				<< ", " << parameter_[i][2] << std::endl;
		}

		return true;
	}


};

struct BagManagerConfig : public ParamterBase {
	typedef std::shared_ptr<BagManagerConfig>	Ptr;

	std::string bagPath;						// bag ·��
	std::string srcptsPath;						// ԭʼ����·��
	std::string motorPath;						// ����������·��
	std::string lidarintrisicPath;              // �������ڲ�����·��
	std::vector<std::string> callbackTopics;	// �ص�topic
	float minDetectDist;						// ��������Сɨ�����
	float maxDetectDist;						// ���������ɨ�����
	double startProcessTime;					// ��ʼ����bagʱ��
	double endProcessTime;						// ��������bagʱ��
	int startProcessId;							// ��ʼ����bag id
	int endProcessId;							// ��������bag id������������id��Χ������Ĵ���ʱ�䷶Χ����Ϊ�˵���ĳ������ʹ�ã��������ã�Ĭ��ȫ��
	double processDuration;						// �궨����Χ��Ĭ���Ա�������ʼ��ת���process_duration����������ڱ궨

	double bagStartTime;						// bag��ʼ��¼ʱ�䣬һ���starttm.txt���룬�������ã�Ĭ���Ե�һ֡laserʱ����Ϊ��ʼʱ��
	bool useTimeReference;						// ��ʱ��ͬ�����

	BagManagerConfig() : bagStartTime(-1.), useTimeReference(true),
						minDetectDist(0.), maxDetectDist(FLT_MAX),
						startProcessTime(-DBL_MAX), endProcessTime(DBL_MAX),
						startProcessId(INT_MIN), endProcessId(INT_MAX), processDuration(DBL_MAX) {}

	bool IsValid() const { return !bagPath.empty() && !callbackTopics.empty(); }

	bool HasTopic(const std::string& topic) const {
		if (std::find(callbackTopics.begin(), callbackTopics.end(), topic) != 
			callbackTopics.end()) {
			return true;
		}
		return false;
	}
};


class BagManager {
public:
	enum LaserType{VELODYNE_16 = 0,
				   VELODYNE_16_H,
				   HESAI_16,
				   HESAI_16_H,
				   MOTOR};

	const std::vector<std::string> SensorTopics{"/velodyne_packets",
												"/ns1/velodyne_packets",
												"/hesai/pandar_packets",
												"/ns1/hesai/pandar_packets",
												"/GV_MotorConfig"};

public:
	typedef std::shared_ptr<BagManager> Ptr;

	BagManager();

	BagManager(const BagManagerConfig::Ptr& config);

	~BagManager();

	// ����bag�����߳�
	bool Start();

	// bag�����߳���ɱ�־
	bool IsFinish();

	// ˮƽ�������ص����������ݲ������ã���bag����laser��������ת�����������
	static void HandleHorizontalLaser(const char* topic, BagPointCloud& laserCloud);

	// ��ֱ�������ص����������ݲ������ã���bag����laser��������ת�����������
	static void HandleVerticalLaser(const char* topic, BagPointCloud& laserCloud);

	// ��ȡһ֡ˮƽ����������
	bool GetHLidarFrame(LidarFrame& frame);

	// ��ȡһ֡��ֱ����������
	bool GetVLidarFrame(LidarFrame& frame);

	// ����motorManagerPtr_
	MotorManager::Ptr GetMotorManager();

	// ���ؼ������ڲ�
	LidarIntrinsicParam::Ptr GetLidarIntrParam();

	LidarIntrinsicParam::Ptr lidarIntrisincParamPtr_;
private:
	// bag��ȡ������ʼ��
	bool InitWithConfig();

	//
	bool setlidarIntrisincParamPtr();
	

	// ת��������
	static void ConvertPointType(const BagPoint& ptIn, BasePoint& ptOut);

	// ת��bag������������
	static bool ConverCloudType(const BagPointCloud& cloudIn, LidarFrame& cloudOut);

private:
	// ��������
	static BagManagerConfig::Ptr configPtr_;

	std::shared_ptr<BagFileReader> bagFileReaderPtr_;

	// �������ڲ�
	

	MotorManager::Ptr motorManagerPtr_;

	std::thread decodeThread_;

	static std::mutex mtxH_;
	static std::mutex mtxV_;
	static int cacheFrameSize_;
	static bool decodeLaserFinished_;
	static std::deque<LidarFrame> rawHorizontalLaserDeq_;
	static std::deque<LidarFrame> rawVerticalLaserDeq_;
};

}// namespace sc
