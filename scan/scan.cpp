#include "scan.h"

namespace scanframe
{
	using namespace sc;
	ScanFrame::ScanFrame(SenSorParamsConfig::Ptr sensorParamsConfig, ScanFrameConfig::Ptr scanFrameConfig)
	{
		gSensorData_ = std::make_shared<SendataIo>(sensorParamsConfig);
		scanFrameConfig_ = scanFrameConfig;
		scanFrameConfig_->correctAngles.resize(360);
		scanFrameConfig_->LoadAngleCorrectionFile();
		threadNum_ = omp_get_max_threads() * 0.8;
		LOG(INFO) << "Thread use: " << threadNum_;
		std::string debugRootDir = sensorParamsConfig->LaserFilePath;
		debugRootDir = GetRootDirectory(sensorParamsConfig->LaserFilePath) + "/ProcessData/";
		CreateDir(debugRootDir);
		debugProcessDataPath_ = debugRootDir;
	}
	void ScanFrame::InputLidarFrame(LidarFrame &lidarFrame)
	{

		m_LidarFrames_.lock();
		qLidarFrames_.emplace(lidarFrame);
		m_LidarFrames_.unlock();
		boost::posix_time::ptime p_time = boost::posix_time::microsec_clock::local_time();
		std::cout << std::setprecision(14) << "lidar: " << p_time <<  " " << lidarFrame.frameId << " " << lidarFrame.startTime << " " << lidarFrame.endTime << " " << lidarFrame.cloudPtr->points.size() <<  std::endl;
	}
	void ScanFrame::InputImuFrame(ImuFrame& imuFrame)
	{
		m_ImuFrames_.lock();
		qImuFrames_.emplace(imuFrame);
		m_ImuFrames_.unlock();
		boost::posix_time::ptime p_time = boost::posix_time::microsec_clock::local_time();
		std::cout << std::setprecision(14) << "imu: " << p_time <<" " <<  imuFrame.time << std::endl;
	}
	//1. 转到编码器坐标系， 2. 转到编码器的0轴坐标系
	void ScanFrame::ConverPoints2M0(BaseCloudPtr cloudIn)
	{
		pcl::transformPointCloud(*cloudIn, *cloudIn, scanFrameConfig_->rotLidar2Motor.cast<float>());
#pragma omp parallel for num_threads(threadNum_)
		for (auto &pt : cloudIn->points)
		{
			PoseD tmpPose;
			// + 或者- 取决于标定的模型
			double angle = pt.angle - Slerp(pt.angle);
			angle = angle * DEG2RAD;
			Eigen::Quaterniond qZ(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
			Eigen::Quaterniond qY(Eigen::AngleAxisd(scanFrameConfig_->alphaY*DEG2RAD, Eigen::Vector3d::UnitY()));
			Eigen::Quaterniond qX(Eigen::AngleAxisd(scanFrameConfig_->alphaX*DEG2RAD, Eigen::Vector3d::UnitX()));
			tmpPose.q = qX * qY * qZ;
			tmpPose.p = qZ * scanFrameConfig_->dp;
			pt.getArray3fMap() = tmpPose.q.cast<float>() * pt.getArray3fMap() + tmpPose.p.cast<float>();
		}
	}

	double ScanFrame::Slerp(double angle)
	{
		double absAngle = std::abs(angle);
		double frontAngle, backAngle;
		int frontId = int(absAngle) % 360;
		int backId = int(absAngle + 1) % 360;
		frontAngle = scanFrameConfig_->correctAngles[frontId];
		backAngle = scanFrameConfig_->correctAngles[backId];
		double ratio = absAngle - int(absAngle);
		return (1 - ratio) * frontAngle + ratio * backAngle;
	}

	bool ScanFrame::Run()
	{
		gSensorData_->Start(boost::bind(&ScanFrame::InputLidarFrame, this, _1), boost::bind(&ScanFrame::InputImuFrame, this, _1));
		while (true)
		{
			if (qImuFrames_.empty() || qLidarFrames_.empty())
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(2));
				continue;
			}
			curLidarFrame_.Clear();
			m_LidarFrames_.lock();
			curLidarFrame_ = qLidarFrames_.front();
			qLidarFrames_.pop();
			m_LidarFrames_.unlock();
			
			PlyIo fullOriginPly(debugProcessDataPath_ + "cloudM0_" + std::to_string(curLidarFrame_.frameId) + ".ply");
			PlyIo fullProcessPly(debugProcessDataPath_ + "cloudProcessM0_" + std::to_string(curLidarFrame_.frameId) + ".ply");
			fullOriginPly.SetOnePatch(*curLidarFrame_.cloudPtr);
			fullOriginPly.Flush();
			fullOriginPly.Close();
			 //将点云从雷达坐标系转到编码器坐标系
			if (scanFrameConfig_->processType == 0)
			{
				ConverPoints2M0(curLidarFrame_.cloudPtr);
			}
			fullProcessPly.SetOnePatch(*curLidarFrame_.cloudPtr);
			fullProcessPly.Flush();
			fullProcessPly.Close();
		}
		return true;
	}
	bool ScanFrameConfig::LoadAngleCorrectionFile()
	{
		std::ifstream ifs(correctAngleFile);
		if (!ifs.is_open()) {
			return false;
		}

		int countId = 0;
		std::string line;
		while (std::getline(ifs, line)) {
			correctAngles[countId] = std::stod(line);
			countId++;

		}
		return countId == 360;
	}
}