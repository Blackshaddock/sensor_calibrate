#include "scan.h"
#include <pcl/registration/ndt.h>
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


		double cx = cos(scanFrameConfig_->alphaX * DEG2RAD), sx = sin(scanFrameConfig_->alphaX * DEG2RAD);
		double cy = cos(scanFrameConfig_->alphaY * DEG2RAD), sy = sin(scanFrameConfig_->alphaY * DEG2RAD);
		double cz = cos(scanFrameConfig_->alphaZ * DEG2RAD), sz = sin(scanFrameConfig_->alphaZ * DEG2RAD);
		Eigen::Matrix3d a;
		a << cy * cz, cy* sz, -sy,
			-cx * sz + sx * sy * cz, cx* cz + sx * sy * sz, sx* cy,
			sx* sz + cx * sy * cz, -sx * cz + cx * sy * sz, cx* cy;
		scanFrameConfig_->mtrixXY = a;
		Eigen::Matrix3d b;
		b << 1, 0, 0, 0, 0.7660444, 0.6427876, 0, -0.6427876, 0.7660444;
		a = a * b;
		std::string debugRootDir = sensorParamsConfig->LaserFilePath;
		debugRootDir = GetRootDirectory(sensorParamsConfig->LaserFilePath) + "/ProcessData/";
		CreateDir(debugRootDir);
		debugProcessDataPath_ = debugRootDir;

		//ndt_omp_ = ndtInit(scanFrameConfig_->ndt_resolution);

	}
	
	pclomp::NormalDistributionsTransform<BasePoint, BasePoint>::Ptr ScanFrame::ndtInit(double ndt_resolution)
	{
		auto ndt_omp = pclomp::NormalDistributionsTransform<BasePoint, BasePoint>::Ptr();
		ndt_omp->setResolution(ndt_resolution);
		ndt_omp->setNumThreads(4);
		ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
		ndt_omp->setTransformationEpsilon(1e-3);
		ndt_omp->setStepSize(0.01);
		ndt_omp->setMaximumIterations(50);
		return ndt_omp;
	}

	void ScanFrame::InputLidarFrame(LidarFrame &lidarFrame)
	{
		//SaveLidarFrame(lidarFrame);
		ConverPoints2M0(lidarFrame.cloudPtr);
		SaveLidarFrame(lidarFrame);
		//m_LidarFrames_.lock();
		m_rR3live.push_back(lidarFrame);
		//m_LidarFrames_.unlock();
		boost::posix_time::ptime p_time = boost::posix_time::microsec_clock::local_time();
		//LOG(INFO) << p_time << std::endl;
	}
	void ScanFrame::InputImuFrame(ImuFrame& imuFrame)
	{
		//m_ImuFrames_.lock();
		m_rR3live.push_back(imuFrame);
		//m_ImuFrames_.unlock();
		boost::posix_time::ptime p_time = boost::posix_time::microsec_clock::local_time();
	}


	void ScanFrame::SaveLidarFrame(LidarFrame& lidarFrame)
	{
		PlyIo fullOriginPly(debugProcessDataPath_ + "M0_" + to_string(lidarFrame.frameId) +".ply");
		fullOriginPly.SetOnePatch(*lidarFrame.cloudPtr);
		fullOriginPly.Flush();
		fullOriginPly.Close();

	}


	//1. 转到编码器坐标系， 2. 转到编码器的0轴坐标系
	void ScanFrame::ConverPoints2M0(BaseCloudPtr cloudIn)
	{
		pcl::transformPointCloud(*cloudIn, *cloudIn, scanFrameConfig_->rotLidar2Motor.cast<float>());
#pragma omp parallel for num_threads(threadNum_)
		//for (auto &pt : cloudIn->points)
		//{
		//	PoseD tmpPose;
		//	// + 或者- 取决于标定的模型
		//	double angle = pt.angle - Slerp(pt.angle);
		//	angle = angle * DEG2RAD;
		//	Eigen::Quaterniond qZ(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
		//	Eigen::Quaterniond qY(Eigen::AngleAxisd(scanFrameConfig_->alphaY*DEG2RAD, Eigen::Vector3d::UnitY()));
		//	Eigen::Quaterniond qX(Eigen::AngleAxisd(scanFrameConfig_->alphaX*DEG2RAD, Eigen::Vector3d::UnitX()));
		//	tmpPose.q = qX * qY * qZ;
		//	tmpPose.p = qZ * scanFrameConfig_->dp;
		//	pt.getArray3fMap() = tmpPose.q.cast<float>() * pt.getArray3fMap() + tmpPose.p.cast<float>();
		//}
		for (auto& pt : cloudIn->points)
		{
			PoseD tmpPose;
			// + 或者- 取决于标定的模型
			 
			double angle = -pt.angle * DEG2RAD;
			double cz = std::cos(angle), sz = std::sin(angle);
			Eigen::Matrix3d b;
			b << cz, sz, 0,
				-sz, cz, 0,
				0, 0, 1;
			//pose.q = motorParam2Ptr_->qAlphaX* motorParam2Ptr_->qAlphaY*qZ;
			tmpPose.q = b * scanFrameConfig_->mtrixXY;
			tmpPose.p = b * scanFrameConfig_->dp;
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
		LOG(INFO) << "Parse Lidar/IMU and Input!" << std::endl;
		gSensorData_->Start(boost::bind(&ScanFrame::InputLidarFrame, this, _1), boost::bind(&ScanFrame::InputImuFrame, this, _1));
		m_rR3live.start();
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