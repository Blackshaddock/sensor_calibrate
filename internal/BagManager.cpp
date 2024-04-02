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

	// ����bag����laser����
	cacheFrameSize_ = 40;				// ����bag�̻߳���֡������
	decodeLaserFinished_ = false;		// ����bag�߳̽�����־

	// ��bag��Ĭ��useTimeReferenceΪtrue
	bagFileReaderPtr_.reset(new BagFileReader);
	bagFileReaderPtr_->SetUseTimeReferens(configPtr_->useTimeReference);
	if (!bagFileReaderPtr_->Open(configPtr_->bagPath.c_str())) {
		LOG(INFO) << "Bag file read error: " << configPtr_->bagPath << std::endl;
		return false;
	}

	// ��������topic�ص�
	BagInfo* bagInfo = nullptr;
	int topicNum = bagFileReaderPtr_->GetTopics(bagInfo);
	for (int i = 0; i < topicNum; ++i) {
		std::string curTopic(bagInfo[i].topic);
		if (!configPtr_->HasTopic(curTopic)) {
			continue;
		}

		// ˮƽ�������ص���������topic����Ӧһ�ֻص�
		if (curTopic == SensorTopics[VELODYNE_16]	&&	bagInfo[i].type == MSG_TYPE_VLPSCAN_16 ||
			curTopic == SensorTopics[VELODYNE_16_H] &&	bagInfo[i].type == MSG_TYPE_VLPSCAN_16 ||
			curTopic == SensorTopics[HESAI_16]		&&	bagInfo[i].type == MSG_TYPE_PANDAR_PACKETS ||
			curTopic == SensorTopics[HESAI_16_H]	&&	bagInfo[i].type == MSG_TYPE_PANDAR_PACKETS) {
			// ����topic����msg����������bag������Χ
			configPtr_->startProcessId = std::max(configPtr_->startProcessId, 0);
			configPtr_->endProcessId = std::min(configPtr_->endProcessId, bagInfo[i].num);

			bagFileReaderPtr_->SetCallBack(bagInfo[i].topic, HandleHorizontalLaser);
			bagFileReaderPtr_->SetStartAndEndIdx(configPtr_->startProcessId, configPtr_->endProcessId);
			LOG(INFO) << "Bag process index range: " << configPtr_->startProcessId 
					  << " - " << configPtr_->endProcessId << std::endl;
		}

		// ��ȡ���������ݣ���ת����ʽ�����������ݵĴ�����ʹ�ö���motorManagerPtr_��
		if (curTopic == SensorTopics[MOTOR]) {
			motorManagerPtr_.reset(new MotorManager);
			if (!motorManagerPtr_->ParseMotorData(bagFileReaderPtr_.get())) {
				LOG(INFO) << "Decode motor data error" << std::endl;
				return false;
			}

			motorManagerPtr_->SaveMotorData(configPtr_->debugRootDir + "motor_datas.txt");

			// ��config������bag��ʼʱ�䣬ʹ�õ�һ֡������ʱ����Ϊ��ʼʱ��
			if (configPtr_->bagStartTime < 0) {
				//configPtr_->bagStartTime = 1657942933.3816452;	// ����
				//configPtr_->bagStartTime = 0;						// ����
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
	// ��config������bag��ʼʱ�䣬ʹ�õ�һ֡ʱ����Ϊ��ʼʱ��
	if (configPtr_->bagStartTime < 0) {
		configPtr_->bagStartTime = laserCloud.endtm;
		LOG(INFO) << "bag decode thread id: " << std::this_thread::get_id() << std::endl;
	}

	// bagԭʼ��������endtm��ϵͳʱ�䣬��ֵ�ܴ���Ҫ��ȥbag��ʼ��¼ʱ��
	laserCloud.endtm -= configPtr_->bagStartTime;

	// configPtr_->endProcessTime�Ǵ�������configPtr_->bagStartTimeʱ�䣬������ʱ���������
	if (laserCloud.endtm > configPtr_->endProcessTime &&
		rawHorizontalLaserDeq_.empty()) {
		decodeLaserFinished_ = true;
		return;
	}
	if (laserCloud.endtm < configPtr_->startProcessTime ||
		laserCloud.endtm > configPtr_->endProcessTime) {
		return;
	}

	// ������������ת������bag��������ת��Ϊ������x,y,z,intensity,ring,time���Եĵ���
	LidarFrame lidarFrame;
	ConverCloudType(laserCloud, lidarFrame);

	// ֡�ڵ��ư�ʱ������
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
	// ��config������bag��ʼʱ�䣬ʹ�õ�һ֡ʱ����Ϊ��ʼʱ��
	if (configPtr_->bagStartTime < 0) {
		configPtr_->bagStartTime = laserCloud.endtm;
	}

	// bagԭʼ��������endtm��ϵͳʱ�䣬��ֵ�ܴ���Ҫ��ȥbag��ʼ��¼ʱ��
	laserCloud.endtm -= configPtr_->bagStartTime;

	// configPtr_->endProcessTime�Ǵ�������configPtr_->bagStartTimeʱ�䣬������ʱ���������
	if (laserCloud.endtm > configPtr_->endProcessTime &&
		rawHorizontalLaserDeq_.empty()) {
		decodeLaserFinished_ = true;
		return;
	}
	if (laserCloud.endtm < configPtr_->startProcessTime ||
		laserCloud.endtm > configPtr_->endProcessTime) {
		return;
	}

	// ������������ת������bag��������ת��Ϊ������x,y,z,intensity,ring,time���Եĵ���
	LidarFrame lidarFrame;
	ConverCloudType(laserCloud, lidarFrame);

	// ֡�ڵ��ư�ʱ������
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

	// ��¼һ֡�����ڵ����ʱ�䣬bag�ڼ�¼ÿ�����ʱ��ķ�Χ�ǣ�0 ~ 0.1 (һ֡�ļ��)
	double maxTime = -1;
	for (int ptId = 0; ptId < cloudIn.ptsNum; ++ptId) {
		const BagPoint& curPt = cloudIn.pts[ptId];
		if (curPt.tm > maxTime) {
			maxTime = curPt.tm;
		}
	}

	// ����һ֡��ʼʱ��
	cloudOut.startTime = cloudOut.endTime - maxTime;

	const float minDist2Thres = configPtr_->minDetectDist * configPtr_->minDetectDist;
	const float maxDist2Thres = configPtr_->maxDetectDist * configPtr_->maxDetectDist;

	// ��������ת��������x,y,z,intensity,ring,time���ԣ������뷶Χɸѡ
	for (int ptId = 0; ptId < cloudIn.ptsNum; ++ptId) {
		const BagPoint& ptIn = cloudIn.pts[ptId];
		float dist2 = ptIn.x * ptIn.x + ptIn.y * ptIn.y + ptIn.z *ptIn.z;

		if (minDist2Thres < dist2 && dist2 < maxDist2Thres) {
			BasePoint ptOut;
			ConvertPointType(ptIn, ptOut);
			ptOut.time += cloudOut.startTime;	// �ָ����bag��ʼʱ�̵�ʱ��
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
	ptOut.time = ptIn.tm;			// ��Χ��0 ~ 0.1��֡�����ʱ��
	ptOut.ring = ptIn.scanID;			// ��id
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