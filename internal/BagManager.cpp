#include "internal/BagManager.h"
#include <glog/logging.h>

// namespace of sensor calibration
namespace sc {

std::mutex BagManager::mtxH_;
std::mutex BagManager::mtxV_;
bool BagManager::decodeLaserFinished_;
int BagManager::cacheFrameSize_;
BagManagerConfig::Ptr BagManager::configPtr_;
std::deque<LidarFrame> BagManager::rawHorizontalLaserDeq_;
std::deque<LidarFrame> BagManager::rawVerticalLaserDeq_;

BagManager::BagManager() {}

BagManager::~BagManager() { 
	bagFileReaderPtr_->Stop(); 
	bagFileReaderPtr_->Close(); 
}

BagManager::BagManager(const BagManagerConfig::Ptr& config) { configPtr_ = config;

}

bool BagManager::Start() {
	if (!InitWithConfig()) {
		return false;
	}

	decodeThread_ = std::thread(std::bind([](BagFileReader* bagFileReader)
								{bagFileReader->Start(); }, bagFileReaderPtr_.get()));
	decodeThread_.detach();

	return true;
}

bool BagManager::IsFinish() { return decodeLaserFinished_; }

MotorManager::Ptr BagManager::GetMotorManager() { return motorManagerPtr_; }

LidarIntrinsicParam::Ptr BagManager::GetLidarIntrParam() { return lidarIntrisincParamPtr_; }

bool BagManager::InitWithConfig() {
	if (configPtr_ == nullptr || !configPtr_->IsValid()) {
		LOG(INFO) << "Please set bag reader config first." << std::endl;
		return false;
	}

	// 设置bag解码laser参数
	cacheFrameSize_ = 40;				// 解码bag线程缓存帧的数量
	decodeLaserFinished_ = false;		// 解码bag线程结束标志

	// 打开bag，默认useTimeReference为true
	bagFileReaderPtr_.reset(new BagFileReader);
	bagFileReaderPtr_->SetUseTimeReferens(configPtr_->useTimeReference);
	if (!bagFileReaderPtr_->Open(configPtr_->bagPath.c_str())) {
		LOG(INFO) << "Bag file read error: " << configPtr_->bagPath << std::endl;
		return false;
	}

	// 设置数据topic回调
	BagInfo* bagInfo = nullptr;
	int topicNum = bagFileReaderPtr_->GetTopics(bagInfo);
	for (int i = 0; i < topicNum; ++i) {
		std::string curTopic(bagInfo[i].topic);
		if (!configPtr_->HasTopic(curTopic)) {
			continue;
		}

		// 水平激光器回调，这四种topic均对应一种回调
		if (curTopic == SensorTopics[VELODYNE_16]	&&	bagInfo[i].type == MSG_TYPE_VLPSCAN_16 ||
			curTopic == SensorTopics[VELODYNE_16_H] &&	bagInfo[i].type == MSG_TYPE_VLPSCAN_16 ||
			curTopic == SensorTopics[HESAI_16]		&&	bagInfo[i].type == MSG_TYPE_PANDAR_PACKETS ||
			curTopic == SensorTopics[HESAI_16_H]	&&	bagInfo[i].type == MSG_TYPE_PANDAR_PACKETS) {
			// 根据topic包含msg个数，设置bag解析范围
			configPtr_->startProcessId = std::max(configPtr_->startProcessId, 0);
			configPtr_->endProcessId = std::min(configPtr_->endProcessId, bagInfo[i].num);

			bagFileReaderPtr_->SetCallBack(bagInfo[i].topic, HandleHorizontalLaser);
			bagFileReaderPtr_->SetStartAndEndIdx(configPtr_->startProcessId, configPtr_->endProcessId);
			LOG(INFO) << "Bag process index range: " << configPtr_->startProcessId 
					  << " - " << configPtr_->endProcessId << std::endl;
		}

		// 读取编码器数据，并转换格式，编码器数据的处理与使用都在motorManagerPtr_中
		if (curTopic == SensorTopics[MOTOR]) {
			motorManagerPtr_.reset(new MotorManager);
			if (!motorManagerPtr_->ParseMotorData(bagFileReaderPtr_.get())) {
				LOG(INFO) << "Decode motor data error" << std::endl;
				return false;
			}

			motorManagerPtr_->SaveMotorData(configPtr_->debugRootDir + "motor_datas.txt");

			// 若config不设置bag起始时间，使用第一帧编码器时间作为起始时间
			if (configPtr_->bagStartTime < 0) {
				//configPtr_->bagStartTime = 1657942933.3816452;	// 调试
				//configPtr_->bagStartTime = 0;						// 调试
				configPtr_->bagStartTime = motorManagerPtr_->GetMotorDatas().front().time;
				LOG(INFO) << "start time: " << std::fixed << std::setprecision(6) 
						  << configPtr_->bagStartTime << std::endl;
			}
		}
	}

	lidarIntrisincParamPtr_ = std::make_shared<LidarIntrinsicParam>(16);

	return true;
}

//bool BagManager::setlidarIntrisincParamPtr()
//{
//	
//	lidarIntrisincParamPtr_ = std::make_shared<LidarIntrinsicParam>(16);
//	
//	return true;
//}

void BagManager::HandleHorizontalLaser(const char* topic, BagPointCloud& laserCloud) {
	// 若config不设置bag起始时间，使用第一帧时间作为起始时间
	if (configPtr_->bagStartTime < 0) {
		configPtr_->bagStartTime = laserCloud.endtm;
		LOG(INFO) << "bag decode thread id: " << std::this_thread::get_id() << std::endl;
	}

	// bag原始解析出的endtm是系统时间，数值很大，需要减去bag起始记录时间
	laserCloud.endtm -= configPtr_->bagStartTime;

	// configPtr_->endProcessTime是传入的相对configPtr_->bagStartTime时间，超过此时间无需解析
	if (laserCloud.endtm > configPtr_->endProcessTime &&
		rawHorizontalLaserDeq_.empty()) {
		decodeLaserFinished_ = true;
		return;
	}
	if (laserCloud.endtm < configPtr_->startProcessTime ||
		laserCloud.endtm > configPtr_->endProcessTime) {
		return;
	}

	// 点云数据类型转换，将bag解析点云转换为包含：x,y,z,intensity,ring,time属性的点云
	LidarFrame lidarFrame;
	ConverCloudType(laserCloud, lidarFrame);

	// 帧内点云按时间排序
	std::sort(lidarFrame.cloudPtr->begin(), lidarFrame.cloudPtr->end(),
		[](const BasePoint&l, const BasePoint& r) {return l.time < r.time; });

	mtxH_.lock();
	rawHorizontalLaserDeq_.emplace_back(std::move(lidarFrame));
	mtxH_.unlock();

	while (rawHorizontalLaserDeq_.size() == cacheFrameSize_) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void BagManager::HandleVerticalLaser(const char* topic, BagPointCloud& laserCloud) {
	// 若config不设置bag起始时间，使用第一帧时间作为起始时间
	if (configPtr_->bagStartTime < 0) {
		configPtr_->bagStartTime = laserCloud.endtm;
	}

	// bag原始解析出的endtm是系统时间，数值很大，需要减去bag起始记录时间
	laserCloud.endtm -= configPtr_->bagStartTime;

	// configPtr_->endProcessTime是传入的相对configPtr_->bagStartTime时间，超过此时间无需解析
	if (laserCloud.endtm > configPtr_->endProcessTime &&
		rawHorizontalLaserDeq_.empty()) {
		decodeLaserFinished_ = true;
		return;
	}
	if (laserCloud.endtm < configPtr_->startProcessTime ||
		laserCloud.endtm > configPtr_->endProcessTime) {
		return;
	}

	// 点云数据类型转换，将bag解析点云转换为包含：x,y,z,intensity,ring,time属性的点云
	LidarFrame lidarFrame;
	ConverCloudType(laserCloud, lidarFrame);

	// 帧内点云按时间排序
	std::sort(lidarFrame.cloudPtr->begin(), lidarFrame.cloudPtr->end(),
		[](const BasePoint&l, const BasePoint& r) {return l.time < r.time; });

	mtxV_.lock();
	rawVerticalLaserDeq_.emplace_back(std::move(lidarFrame));
	mtxV_.unlock();

	while (rawVerticalLaserDeq_.size() == cacheFrameSize_) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

bool BagManager::ConverCloudType(const BagPointCloud& cloudIn, LidarFrame& cloudOut) {
	cloudOut.endTime = cloudIn.endtm;
	cloudOut.frameId = cloudIn.rangeid;
	cloudOut.cloudPtr->reserve(cloudIn.ptsNum);

	// 记录一帧点云内的最大时间，bag内记录每个点的时间的范围是：0 ~ 0.1 (一帧的间隔)
	double maxTime = -1;
	for (int ptId = 0; ptId < cloudIn.ptsNum; ++ptId) {
		const BagPoint& curPt = cloudIn.pts[ptId];
		if (curPt.tm > maxTime) {
			maxTime = curPt.tm;
		}
	}

	// 反推一帧起始时刻
	cloudOut.startTime = cloudOut.endTime - maxTime;

	const float minDist2Thres = configPtr_->minDetectDist * configPtr_->minDetectDist;
	const float maxDist2Thres = configPtr_->maxDetectDist * configPtr_->maxDetectDist;

	// 点云类型转换，保留x,y,z,intensity,ring,time属性，按距离范围筛选
	for (int ptId = 0; ptId < cloudIn.ptsNum; ++ptId) {
		const BagPoint& ptIn = cloudIn.pts[ptId];
		float dist2 = ptIn.x * ptIn.x + ptIn.y * ptIn.y + ptIn.z *ptIn.z;

		if (minDist2Thres < dist2 && dist2 < maxDist2Thres) {
			BasePoint ptOut;
			ConvertPointType(ptIn, ptOut);
			ptOut.time += cloudOut.startTime;	// 恢复相对bag起始时刻的时间
			ptOut.id = cloudOut.frameId;
			cloudOut.cloudPtr->push_back(ptOut);
		}
	}

	return true;
}

void BagManager::ConvertPointType(const BagPoint& ptIn, BasePoint& ptOut) {
	ptOut.x = ptIn.x;
	ptOut.y = ptIn.y;
	ptOut.z = ptIn.z;
	ptOut.intensity = ptIn.intensity;
	ptOut.time = ptIn.tm;			// 范围：0 ~ 0.1，帧内相对时间
	ptOut.ring = ptIn.scanID;			// 线id
}

bool BagManager::GetHLidarFrame(LidarFrame& frame) {
	if (rawHorizontalLaserDeq_.empty()) {
		return false;
	}

	mtxH_.lock();
	frame = std::move(rawHorizontalLaserDeq_.front());
	rawHorizontalLaserDeq_.pop_front();
	mtxH_.unlock();

	return true;
}

bool BagManager::GetVLidarFrame(LidarFrame& frame) {
	if (rawVerticalLaserDeq_.empty()) {
		return false;
	}

	mtxH_.lock();
	frame = rawVerticalLaserDeq_.front();
	rawVerticalLaserDeq_.pop_front();
	mtxH_.unlock();

	return true;
}
}// namespace sc