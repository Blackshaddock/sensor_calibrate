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

	std::string bagPath;						// bag 路径
	std::string srcptsPath;						// 原始数据路径
	std::string motorPath;						// 编码器数据路径
	std::string lidarintrisicPath;              // 激光器内参数据路径
	std::vector<std::string> callbackTopics;	// 回调topic
	float minDetectDist;						// 激光器最小扫描距离
	float maxDetectDist;						// 激光器最大扫描距离
	double startProcessTime;					// 开始解析bag时间
	double endProcessTime;						// 结束处理bag时间
	int startProcessId;							// 开始处理bag id
	int endProcessId;							// 结束处理bag id，这两个处理id范围和上面的处理时间范围都是为了调试某段数据使用，若不设置，默认全解
	double processDuration;						// 标定处理范围，默认以编码器开始旋转后的process_duration秒的数据用于标定

	double bagStartTime;						// bag起始记录时间，一般从starttm.txt读入，若不设置，默认以第一帧laser时间作为起始时间
	bool useTimeReference;						// 与时间同步相关

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

	// 启动bag解码线程
	bool Start();

	// bag解码线程完成标志
	bool IsFinish();

	// 水平激光器回调函数，根据参数设置，将bag解码laser点云类型转换，存入队列
	static void HandleHorizontalLaser(const char* topic, BagPointCloud& laserCloud);

	// 垂直激光器回调函数，根据参数设置，将bag解码laser点云类型转换，存入队列
	static void HandleVerticalLaser(const char* topic, BagPointCloud& laserCloud);

	// 获取一帧水平激光器数据
	bool GetHLidarFrame(LidarFrame& frame);

	// 获取一帧垂直激光器数据
	bool GetVLidarFrame(LidarFrame& frame);

	// 返回motorManagerPtr_
	MotorManager::Ptr GetMotorManager();

	// 返回激光器内参
	LidarIntrinsicParam::Ptr GetLidarIntrParam();

	LidarIntrinsicParam::Ptr lidarIntrisincParamPtr_;
private:
	// bag读取参数初始化
	bool InitWithConfig();

	//
	bool setlidarIntrisincParamPtr();
	

	// 转换点类型
	static void ConvertPointType(const BagPoint& ptIn, BasePoint& ptOut);

	// 转换bag解析点云类型
	static bool ConverCloudType(const BagPointCloud& cloudIn, LidarFrame& cloudOut);

private:
	// 参数设置
	static BagManagerConfig::Ptr configPtr_;

	std::shared_ptr<BagFileReader> bagFileReaderPtr_;

	// 激光器内参
	

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
