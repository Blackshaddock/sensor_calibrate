#include "module/LidarMotorCalibration.h"
#include "internal/Utils.h"
#include "internal/CeresOptimizer.h"
#include "internal/CeresHelper.h"
#include "internal/PlyIo.hpp"

#include <pcl/io/pcd_io.h>		// 仅用于调试，后续可以取消
#include <pcl/io/ply_io.h>		// 仅用于调试，后续可以取消
#include <pcl/common/impl/io.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>
#include <omp.h>

#ifdef USEPLANEDETECT
#include "PlaneExtraction.hpp"
#endif // USEPLANEDETECT

// namespace of sensor calibration
namespace sc {

LidarMotorCalibration::LidarMotorCalibration(const LidarMotorCalibrationConfig::Ptr& configPtr)
	: configPtr_(configPtr), debugFlag_(true) {}

LidarMotorCalibration::~LidarMotorCalibration() {}

bool LidarMotorCalibration::Run() {
	// config参数初始化，新建调试输出路径
	if (!InitWithConfig()) {
		return false;
	}

	// bag解析点云，通过初始编码器内参、角度修正值、激光器内参，
	// 将bag原始点云转到global系下，为后续标定提供原始数据
	if (!DataPrepare4geosun()) {
		return false;
	}

	// 标定，使用大于小于0的点云，同时标定编码器内参、角度修正、激光内参
	if (!RunCalibration()) {
		return false;
	}

	// 保存三种参数的标定结果
	if (!SaveOptimizeResult()) {
		return false;
	}

	// 根据标定结果，校正原始点云，验证标定结果是否正确
	if (!CheckOptimizeResult()) {
		return false;
	}

	return true;
}

bool LidarMotorCalibration::InitWithConfig() {
	if (configPtr_ == nullptr || !configPtr_->IsValid()) {
		std::cout << "Please set calibration config first." << std::endl;
		return false;
	}

	threadNum_ = omp_get_max_threads() * 0.8;
	LOG(INFO) << "Thread use: " << threadNum_;

	// 调试信息初始化
	std::string debugRootDir = configPtr_->bagManagerCfgPtr->bagPath;
	debugOptDataPath_ = debugRootDir;
	if (debugFlag_) {
		debugRootDir = GetRootDirectory(configPtr_->bagManagerCfgPtr->bagPath) + "/" +
					   GetFileName(configPtr_->bagManagerCfgPtr->bagPath) + "/";
		if (CreateDir(debugRootDir)) {
			debugOptDataPath_ = debugRootDir + "optimize_cloud/";
			if (!CreateDir(debugOptDataPath_)) {
				debugOptDataPath_ = debugRootDir;
			}

			debugOptResultPath_ = debugRootDir + "optimize_result/";
			if (!CreateDir(debugOptResultPath_)) {
				debugOptResultPath_ = debugRootDir;
			}

			debugTestPath_ = debugRootDir + "test/";
			if (!CreateDir(debugTestPath_)) {
				debugTestPath_ = debugRootDir;
			}
		}
		else {
			debugFlag_ = false;
		}
	}

	configPtr_->bagManagerCfgPtr->debugRootDir = 
		configPtr_->motorCalibParam->debugRootDir = debugOptDataPath_;
	configPtr_->bagManagerCfgPtr->debugFlag =
		configPtr_->motorCalibParam->debugFlag = debugFlag_;
	configPtr_->bagManagerCfgPtr->threadNum_ =
		configPtr_->motorCalibParam->threadNum_ = threadNum_;

	// 初始化glog
	google::InitGoogleLogging(debugRootDir.c_str());
	FLAGS_log_dir = debugRootDir;
	FLAGS_alsologtostderr = true;
	google::SetLogFilenameExtension("calibration");
	FLAGS_logbufsecs = 0;
	FLAGS_stop_logging_if_full_disk = true;
	FLAGS_max_log_size = 1024;

	return true;
}


bool LidarMotorCalibration::DataPrepare4geosun()
{
	bagManagerPtr_.reset(new BagManager(configPtr_->bagManagerCfgPtr));
	bagManagerPtr_->lidarIntrisincParamPtr_ = std::make_shared<LidarIntrinsicParam>(16);
	bagManagerPtr_->lidarIntrisincParamPtr_->Load(configPtr_->bagManagerCfgPtr->lidarintrisicPath);
	motorManagerPtr_.reset(new MotorManager);
	//motorManagerPtr_ = bagManagerPtr_->GetMotorManager();
	motorManagerPtr_->SetMotorParameter(configPtr_->motorCalibParam, configPtr_->motorAngleCorrectParamPtr);
	
	PlyIo fullOriginPly(debugOptDataPath_ + "full_cloud_origin.ply");

	std::fstream fd(configPtr_->bagManagerCfgPtr->srcptsPath);
	std::string tmp;
	BasePoint tmp_pt;
	LidarFrame lidarFrameLocal, lidarFrameGlobal;
	for (int i = 0;; i++)
	{
		std::getline(fd, tmp);
		if (tmp == "")
		{
			break;
		}
		sscanf(tmp.c_str(), " %f %f %f %f %f %d %f", &tmp_pt.x, &tmp_pt.y, &tmp_pt.z, &tmp_pt.intensity, &tmp_pt.time, &tmp_pt.ring, &tmp_pt.angle);
		//将bag解析的原始坐标系下的点云数据转到适配编码器的局部坐标系
		if (abs(tmp_pt.x) < 0.1 || abs(tmp_pt.y) < 0.1 || abs(tmp_pt.z) < 0.1 || abs(tmp_pt.x) > 100 || abs(tmp_pt.y) > 100 || abs(tmp_pt.z) > 100)
		{
			continue;
		}
		tmp_pt.normal_x = tmp_pt.x;
		tmp_pt.normal_y = tmp_pt.y;
		tmp_pt.normal_z = tmp_pt.z;
		
		//tmp_pt.angle *= -1.0;
		ConvertToMotorLocalCoordinate(tmp_pt);
		lidarFrameLocal.cloudPtr->push_back(tmp_pt);
	}
	// 根据读入的雷达内参，矫正数据 
	CorrectPointUseLidarIntrParam(lidarFrameLocal.cloudPtr);
	//根据编码器角度观测值，结合编码器内参、编码器角度修正初值，将编码器局部坐标系点云转到全局坐标系
	ConvertToMotorGlobalCoordinate4geosun(lidarFrameLocal.cloudPtr, lidarFrameGlobal.cloudPtr);
	fullOriginPly.SetOnePatch(*lidarFrameGlobal.cloudPtr);
	fullOriginPly.Flush();
	fullOriginPly.Close();
	return true;
	
}

bool LidarMotorCalibration::DataPrepare() {
	// 启动bag解码线程
	bagManagerPtr_.reset(new BagManager(configPtr_->bagManagerCfgPtr));
	if (!bagManagerPtr_->Start()) {
		return false;
	}

	// 获取编码器数据，传入编码器标定参数
	motorManagerPtr_ = bagManagerPtr_->GetMotorManager();
	motorManagerPtr_->SetMotorParameter(configPtr_->motorCalibParam, 
										configPtr_->motorAngleCorrectParamPtr,
										configPtr_->bagManagerCfgPtr->bagStartTime);

	// config设置时间范围内的所有点云，经过编码器角度转换、
	// 初始编码器内参与360个修正角修正后的全局点云，用于后续标定的原始点
	// 注意：已经为每个点添加了ring、id、time、angle、normal附加属性，会一起写出到ply点云中
	PlyIo fullOriginPly(debugOptDataPath_ + "full_cloud_origin.ply");

	LidarFrame lidarFrameLocal, lidarFrameGlobal;
	while (!bagManagerPtr_->IsFinish()) {		
		lidarFrameLocal.Clear(), lidarFrameGlobal.Clear();

		// 获取一帧水平激光器数据，此数据还是bag原始解码坐标系下的点云数据
		if (!bagManagerPtr_->GetHLidarFrame(lidarFrameLocal)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		// 将bag解析的原始坐标系下的点云数据转到适配编码器的局部坐标系
		ConvertToMotorLocalCoordinate(lidarFrameLocal.cloudPtr);

		// 根据读入的雷达内参，矫正数据 
		CorrectPointUseLidarIntrParam(lidarFrameLocal.cloudPtr);

		// 根据编码器角度观测值，结合编码器内参、编码器角度修正初值，将编码器局部坐标系点云转到全局坐标系
		ConvertToMotorGlobalCoordinate(lidarFrameLocal.cloudPtr, lidarFrameGlobal.cloudPtr);

		//// 根据参数设置，提取特定laser line点用于标定  --> 现在使用所有点，这里不做筛选
		//ExtractLaserLineCloud(lidarFrameLocal.cloudPtr, lidarFrameGlobal.cloudPtr);

		fullOriginPly.SetOnePatch(*lidarFrameGlobal.cloudPtr);
		fullOriginPly.Flush();
	}

	//bagManagerPtr_->~BagManager();	// 这个有bug
	fullOriginPly.Close();

	if (debugFlag_) {
		PlyIo::SavePLYFileBinary(debugOptDataPath_ + "local_frame.ply", *lidarFrameLocal.cloudPtr);
		PlyIo::SavePLYFileBinary(debugOptDataPath_ + "global_frame.ply", *lidarFrameGlobal.cloudPtr);
	}
	return true;
}

bool LidarMotorCalibration::CorrectPointUseLidarIntrParam(BaseCloudPtr& cloud) {
	//double** lidarIntrParam = bagManagerPtr_->GetLidarIntrParam()->GetParam();
	double *lidarIntrParam = motorManagerPtr_->GetMotorAngleCorrectParamPtr()->GetHorizons();
	
	BaseCloudPtr correctCloud(new BaseCloud);

	for (BasePoint& p : cloud->points) {
		Eigen::Vector3d info, pOri, pCorrect;
		pOri = p.getVector3fMap().cast<double>();

		// 反算local点的基本信息：垂直角、水平角、距离
		if (!CeresHelper<double>::GetPointInfo(pOri.data(), info.data())) {
			return false;
		}

		// 根据当前点所属ring，得到读入的初始激光器内参修正量，修正原始点
		CeresHelper<double>::CorrectPointUseLidarIntrParam(
			info.data(), &lidarIntrParam[p.ring], pCorrect.data());
		/*std::cout << p.x << " " << p.y << " " << p.z << " "  << unsigned(p.ring) <<" " << pCorrect[0] << " " << pCorrect[1] << " " << pCorrect[2] << " " << info[0] <<
			" " << info[1] << " " << info[2] <<" " <<  lidarIntrParam[p.ring] <<  std::endl;*/
		p.x = pCorrect[0];
		p.y = pCorrect[1];
		p.z = pCorrect[2];

		correctCloud->push_back(p);
	}

	cloud = correctCloud;
	correctCloud = nullptr;
	return true;
}


void LidarMotorCalibration::ConvertToMotorLocalCoordinate(BaseCloudPtr& cloud) {
	// 这个转换关系仅在激光-编码器标定使用，内网解析原始数据的转换不是这样的
	// P_new = R * P_ori;
	// R = [ 0  0  -1
	//      -1  0   0
	//       0  1   0]

	for (BasePoint& p : cloud->points) {
		float xOri = p.x;
		float yOri = p.y;
		float zOri = p.z;

		p.x = -zOri;
		p.y = -xOri;
		p.z = yOri;
	}
}

void LidarMotorCalibration::ConvertToMotorLocalCoordinate(BasePoint& pt)
{
	//geosun 雷达系-编码器
	/*R= [ -1 0 0 
		 0  0 -1
		 0  -1 0]*/


	float xOri = pt.x;
	float yOri = pt.y;
	float zOri = pt.z;

	pt.x = -xOri;
	pt.y = -zOri;
	pt.z = -yOri;


}

bool LidarMotorCalibration::ConvertToMotorGlobalCoordinate(const BaseCloudPtr& cloudIn, 
														   BaseCloudPtr& cloudOut) {
	int cloudSize = cloudIn->size();
	cloudOut->reserve(cloudSize);

	for (int id = 0; id < cloudSize; id++) {
		BasePoint& curPt = cloudIn->points[id], transPt;

		// 根据当前点时间戳，内插编码器pose，该pose可以直接用于转换编码器local坐标系下的点到global坐标系
		PoseD curPose;		// 编码器pose
		double angle = 0;	// 编码器角度
		if (!motorManagerPtr_->GetMotorPose(curPt.time, curPose, angle)) {
			LOG(INFO) << "Motor slerp pose error: " << curPt.time;
			return false;
		}

		curPt.angle = angle * RAD2DEG;
		transPt = curPt;	// 保留原始点所有属性
		transPt.getVector3fMap() = curPose.q.cast<float>() * curPt.getVector3fMap() +
								   curPose.p.cast<float>();
		transPt.getNormalVector3fMap() = curPt.getVector3fMap();	// 临时使用normal属性存储局部坐标系下的点
		cloudOut->push_back(transPt);
	}
	return true;
}

bool LidarMotorCalibration::ConvertToMotorGlobalCoordinate4geosun(const BaseCloudPtr& cloudIn, BaseCloudPtr& cloudOut)
{
	int cloudSize = cloudIn->size();
	cloudOut->reserve(cloudSize);

	for (int id = 0; id < cloudSize; id++) {
		BasePoint& curPt = cloudIn->points[id], transPt;

		// 根据当前点的编码器pose，该pose可以直接用于转换编码器local坐标系下的点到global坐标系

		PoseD curPose;		// 编码器pose
		double angle = curPt.angle;	// 编码器角度
		
		if (!motorManagerPtr_->GetMotorPose4geosun(angle, curPose, configPtr_->optimizeMotorAngleCorrParam)) {
			LOG(INFO) << "Motor slerp pose error: " << curPt.time;
				return false;
		}
		

		transPt = curPt;	// 保留原始点所有属性
		transPt.getVector3fMap() = curPose.q.cast<float>() * curPt.getVector3fMap() +
			curPose.p.cast<float>();
		transPt.getNormalVector3fMap() = curPt.getVector3fMap();	// 临时使用normal属性存储局部坐标系下的点
		cloudOut->push_back(transPt);
	}
	return true;
}

void LidarMotorCalibration::ExtractLaserLineCloud(const BaseCloudPtr& cloudLocal, const BaseCloudPtr& cloudGlobal) {
	std::vector<BaseCloud> laserCloudVec;

	const auto& laserIdVec1 = configPtr_->calibLaserIdVec1;
	int calibLaserNum1 = laserIdVec1.size();
	laserCloudVec.resize(calibLaserNum1);

	for (int id = 0; id < cloudLocal->size(); id++) {
		const BasePoint& ptLocal = cloudLocal->points[id];
		const BasePoint& ptGlobal = cloudGlobal->points[id];

		for (int i = 0; i < calibLaserNum1; i++) {
			if (ptLocal.ring == laserIdVec1[i]) {
				if (ptLocal.x > 0) {
					laserCloudVec[i].push_back(ptGlobal);
				}
			}
		}
	}
}

bool LidarMotorCalibration::RunCalibration() {
	BaseCloudPtr fullCloud;
	MotorCalibMatchPairs matchPairs;

	// 读入原始数据，根据需求提取标定匹配对
	if (!ExtractMatchingPairs(fullCloud, configPtr_->calibLaserIdVec1, matchPairs, true)) {
		return false;
	}

	// 设置优化参数
	CeresOptimizerConfig::Ptr option = std::make_shared<CeresOptimizerConfig>();
	option->threadNum_ = threadNum_;		// 线程数
	option->debugFlag = debugFlag_;			// 输出debug信息
	option->debugRootDir = debugTestPath_;	// debug信息输出路径
	option->optimizeLidarIntrinsicParam = configPtr_->optimizeLidarIntrisincParam;	// 优化编码器内参
	option->optimizeMotorAngleParam = configPtr_->optimizeMotorAngleCorrParam;		// 优化编码器角度修正量
	option->optimizeMotorIntrinsicParam = configPtr_->optimizeMotorIntrisincParam;	// 优化激光器内参

	CeresOptimizer optimizer(option);

	// 传入待优化的编码器内参、角度修正量、激光器内参 参数
	optimizer.SetLidarMotorCalibParam(motorManagerPtr_, bagManagerPtr_->GetLidarIntrParam());

	// 此时添加的是同时优化编码器内参、角度修正量、激光器内参的观测值
	optimizer.AddLidarMotorCalibObservation4(matchPairs);

	// 解算
	optimizer.Solve();

	// 调试残差是否符合预期，会重新统计优化后残差，验证结果对不对
	if (debugFlag_) {
		std::cout << "   " << std::endl;
		//optimizer.AddLidarMotorCalibObservation4(matchPairs);
	}

	return true;
}

bool LidarMotorCalibration::ExtractMatchingPairs(BaseCloudPtr& fullCloud, const std::vector<int>& calibIdVec,
												 MotorCalibMatchPairs& matchPairs, bool keepLarge0) {
	BaseCloudPtr sourceCloud(new BaseCloud), targetCloud(new BaseCloud);

	// 提取原始点大于0 和 小于0 的点云，分别做source 和 target点
	if (!ExtractCalibrationCloud(fullCloud, sourceCloud, targetCloud, calibIdVec, false)) {
		return false;
	}

	// 固定比例抽稀
	BaseCloudPtr sourceCloudDs(new BaseCloud), targetCloudDs(new BaseCloud);

	int remainSize = 100000;
	int step = std::ceil(sourceCloud->size() / remainSize) +1;
	for (int i = 0; i < sourceCloud->size(); i = i + step) {
		sourceCloudDs->push_back(sourceCloud->points[i]);
	}
	for (int i = 0; i < targetCloud->size(); i = i + step) {
		targetCloudDs->push_back(targetCloud->points[i]);
	}
	/*sourceCloud = sourceCloudDs;
	targetCloud = targetCloudDs;*/

	
	// 计算法向，统计近邻点id，这个id会在平面检测时会用到
	std::vector<std::vector<int>> indexes1, indexes2;
	std::vector<std::vector<int>> indexes3, indexes4;
	GetNormal(sourceCloud, indexes1);
	GetNormal(targetCloud, indexes2);
	GetNormal(sourceCloudDs, indexes3);
	GetNormal(targetCloudDs, indexes3);

#ifdef USEPLANEDETECT
	// 若使用平面检测库，则用拟合平面后的点做特征关联
	plane_extract::ExtractPlaneCloud(sourceCloud, sourceCloud, indexes1);
	plane_extract::ExtractPlaneCloud(targetCloud, targetCloud, indexes2);
#endif // USEPLANEDETECT

	//ICP配准获取同名点索引，R、T不用进行优化
	BaseCloudPtr outCloud(new BaseCloud);
	if (!GetCorrespondPointIndex(sourceCloud, targetCloud, outCloud))
	{
		return false;
	}

	// 提取匹配对
	/*if (!FeatureAssociation(sourceCloud, targetCloud, outCloud, matchPairs)) {
		return false;
	}*/

	if (!FeatureAssociation4geosun(sourceCloud, targetCloud, sourceCloudDs, targetCloudDs,outCloud, matchPairs)) {
		return false;
	}
	
	if (debugFlag_) {
		PlyIo::SavePLYFileBinary(debugTestPath_ + "source_cloud.ply", *sourceCloud);
		PlyIo::SavePLYFileBinary(debugTestPath_ + "target_cloud.ply", *targetCloud);
		PlyIo::SavePLYFileBinary(debugTestPath_ + "out_cloud.ply", *outCloud);
	}
	return true;
}

bool LidarMotorCalibration::GetCorrespondPointIndex(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud, const BaseCloudPtr& outCloud)
{
	if (sourceCloud->empty() || targetCloud->empty()) {
		return false;
	}
	//pcl::IterativeClosestPoint<BasePoint, BasePoint>::Ptr icp(new pcl::IterativeClosestPoint<BasePoint, BasePoint>);
	pcl::GeneralizedIterativeClosestPoint<BasePoint, BasePoint>::Ptr icp(new pcl::GeneralizedIterativeClosestPoint<BasePoint, BasePoint>);
	icp->setInputSource(sourceCloud);
	icp->setInputTarget(targetCloud);
	/*icp->setMaximumIterations(50);
	icp->setMaxCorrespondenceDistance(0.2);
	icp->setTransformationEpsilon(1e-8);
	icp->setEuclideanFitnessEpsilon(0.01);*/
	
	icp->align(*outCloud);
	if (!icp->hasConverged())
	{
		LOG(INFO) << "ICP Converge: 0"  ;
		return false;
	}
	std::cout << icp->getFitnessScore() << std::endl;
	//pcl::transformPointCloud(*sourceCloud, *outCloud, icp->getFinalTransformation());
	
	return true;
}

bool LidarMotorCalibration::ExtractCalibrationCloud(BaseCloudPtr& fullCloud, BaseCloudPtr& sourceCloud, 
													BaseCloudPtr& targetCloud, const std::vector<int>& calibIdVec,
													bool keepLarge0) {
	if (fullCloud == nullptr || fullCloud->empty()) {
		fullCloud.reset(new BaseCloud);
		pcl::io::loadPLYFile(debugOptDataPath_ + "full_cloud_origin.ply", *fullCloud);
		if (fullCloud->empty()) {
			LOG(INFO) << "Please run datapretrate first.";
			return false;
		}
	}

	if (keepLarge0) {
		for (const BasePoint& pt : fullCloud->points) {
			
			// 这里临时用normal属性存储原始坐标系下的点
			if (pt.normal_x > 0) {
				if (pt.ring == calibIdVec[0]) {
					sourceCloud->push_back(pt);
				}
				else if (pt.ring == calibIdVec[1]) {
					targetCloud->push_back(pt);
				}
			}
		}
	}
	else {
		for (const BasePoint& pt : fullCloud->points) {
			// 这里临时用normal属性存储原始坐标系下的点
			if (pt.normal_x > 0) {
				sourceCloud->push_back(pt);
			}
			else {
				targetCloud->push_back(pt);
			}
		}
	}

	if (debugFlag_) {
		PlyIo::SavePLYFileBinary(debugTestPath_ + "source_cloud_extract.ply", *sourceCloud);
		PlyIo::SavePLYFileBinary(debugTestPath_ + "target_cloud_extract.ply", *targetCloud);
	}

	return !sourceCloud->empty() && !targetCloud->empty();
}

void LidarMotorCalibration::GetNormal(BaseCloudPtr &cloud, std::vector<std::vector<int>>& indexes) {
	pcl::KdTreeFLANN<BasePoint>::Ptr kdtree(new pcl::KdTreeFLANN<BasePoint>);
	kdtree->setInputCloud(cloud);

	const int cloudSize = cloud->size(), knnNum = 30;
	indexes.resize(cloudSize);
#pragma omp parallel for num_threads(threadNum_)
	for (int i = 0; i < cloudSize; i++) {
		BasePoint& pt = cloud->points[i];
		std::vector<int> knnIdx(knnNum);
		std::vector<float> knnDist(knnNum);

		kdtree->nearestKSearch(pt, knnNum, knnIdx, knnDist);

		BaseCloudPtr cloudSearch(new BaseCloud);
		pcl::copyPointCloud(*cloud, knnIdx, *cloudSearch);

		pcl::PCA<BasePoint> pca;
		pca.setInputCloud(cloudSearch);
		Eigen::Matrix3f eigeVector = pca.getEigenVectors();
		Eigen::Matrix<float, 3, 1> eigenValues;

		eigenValues = pca.getEigenValues();
		Eigen::Vector3f vect_2 = eigeVector.col(2);// please fit to your own coordinate
		Eigen::Vector3f vect_1 = Eigen::Vector3f(pt.x, pt.y, pt.z);
		double cosValNew = vect_1.dot(vect_2) / (vect_1.norm()*vect_2.norm());
		double angleNew = acos(cosValNew) * 180 / M_PI;
		if (angleNew > 90) {
			vect_2 = -vect_2;
		}

		indexes[i] = std::move(knnIdx);

		pt.normal_x = vect_2.x();
		pt.normal_y = vect_2.y();
		pt.normal_z = vect_2.z();
		pt.data[3] = eigenValues(0) / (eigenValues(2) + 1e-4);  // normalConfidence;
		pt.data_n[3] = eigenValues(2) / eigenValues.array().sum();  // curvature;
	}
}

bool LidarMotorCalibration::FeatureAssociation(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud, const BaseCloudPtr& cloudIn,
											   MotorCalibMatchPairs& matchPairs) {
	if (sourceCloud->empty() || targetCloud->empty() || cloudIn->empty()) {
		return false;
	}

	pcl::KdTreeFLANN<BasePoint>::Ptr kdtree(new pcl::KdTreeFLANN<BasePoint>);
	pcl::KdTreeFLANN<BasePoint>::Ptr kdtree1(new pcl::KdTreeFLANN<BasePoint>);
	kdtree->setInputCloud(targetCloud);
	kdtree1->setInputCloud(sourceCloud);

	int knnSize = 2;
	float knnDistThres = 0.05;

	int sourceCloudSize = sourceCloud->size();
	matchPairs.clear();
	matchPairs.resize(sourceCloudSize);
	std::ofstream fout(debugOptResultPath_ + "correspone_name.txt");
	std::ofstream fout1(debugOptResultPath_ + "correspone_name1.txt");
#pragma omp parallel for num_threads(threadNum_)
	for (int i = 0; i < sourceCloudSize; i++) {
		const BasePoint& sPt = sourceCloud->points[i];
		std::vector<int> knnIdx(knnSize), knnIdx1(knnSize);
		std::vector<float> knnDist(knnSize), knnDist1(knnSize);

		kdtree->nearestKSearch(sPt, knnSize, knnIdx, knnDist);

		MotorCalibMatchPair& matchPair = matchPairs[i];
		matchPair.sPt = sPt;
		matchPair.tPt = targetCloud->points[knnIdx[0]];
		kdtree1->nearestKSearch(targetCloud->points[knnIdx[0]], knnSize, knnIdx1, knnDist1);
		if (knnIdx1[0] == i && knnDist.front() < knnDistThres)
		{
			matchPair.isValid = true;
		}

		/*if (knnDist.front() < knnDistThres) {
			matchPair.isValid = true;
		}*/
	}


	for (int i = 0; i < matchPairs.size(); ++i) {
		const MotorCalibMatchPair& pair = matchPairs[i];
		if (!pair.isValid) {
			continue;
		}
		fout << pair.sPt.x << " " << pair.sPt.y << " " << pair.sPt.z << " " << i <<" " << pair.sPt.angle << " " << pair.sPt.time <<  std::endl;
		fout1 << pair.tPt.x << " " << pair.tPt.y << " " << pair.tPt.z << " " << i << " " << pair.tPt.angle << " " << pair.tPt.time << std::endl;
	}
	fout.close();
	fout1.close();
	return !matchPairs.empty();
}

bool LidarMotorCalibration::FeatureAssociation4geosun(const BaseCloudPtr& sourceCloud, const BaseCloudPtr& targetCloud, const BaseCloudPtr& sourceCloudDs, const BaseCloudPtr& targetCloudDs, const BaseCloudPtr& cloudIn, MotorCalibMatchPairs& matchPairs)
{
	if (sourceCloud->empty() || targetCloud->empty() || sourceCloudDs->empty()) {
		return false;
	}

	pcl::KdTreeFLANN<BasePoint>::Ptr kdtree(new pcl::KdTreeFLANN<BasePoint>);
	pcl::KdTreeFLANN<BasePoint>::Ptr kdtree1(new pcl::KdTreeFLANN<BasePoint>);
	kdtree->setInputCloud(targetCloud);
	kdtree1->setInputCloud(sourceCloudDs);

	int knnSize = 2;
	float knnDistThres = 0.05;

	int sourceCloudSize = sourceCloudDs->size();
	matchPairs.clear();
	matchPairs.resize(sourceCloudSize);
	std::ofstream fout(debugOptResultPath_ + "correspone_name.txt");
	std::ofstream fout1(debugOptResultPath_ + "correspone_name1.txt");
#pragma omp parallel for num_threads(threadNum_)
	for (int i = 0; i < sourceCloudSize; i++) {
		const BasePoint& sPt = sourceCloudDs->points[i];
		std::vector<int> knnIdx(knnSize), knnIdx1(knnSize);
		std::vector<float> knnDist(knnSize), knnDist1(knnSize);

		kdtree->nearestKSearch(sPt, knnSize, knnIdx, knnDist);

		MotorCalibMatchPair& matchPair = matchPairs[i];
		matchPair.sPt = sPt;
		matchPair.tPt = targetCloud->points[knnIdx[0]];
		kdtree1->nearestKSearch(targetCloud->points[knnIdx[0]], knnSize, knnIdx1, knnDist1);
		if (knnIdx1[0] == i && knnDist.front() < knnDistThres)
		{
			matchPair.isValid = true;
		}

		/*if (knnDist.front() < knnDistThres) {
			matchPair.isValid = true;
		}*/
	}


	for (int i = 0; i < matchPairs.size(); ++i) {
		const MotorCalibMatchPair& pair = matchPairs[i];
		if (!pair.isValid) {
			continue;
		}
		fout << pair.sPt.x << " " << pair.sPt.y << " " << pair.sPt.z << " " << i << " " << pair.sPt.angle << " " << pair.sPt.time << std::endl;
		fout1 << pair.tPt.x << " " << pair.tPt.y << " " << pair.tPt.z << " " << i << " " << pair.tPt.angle << " " << pair.tPt.time << std::endl;
	}
	fout.close();
	fout1.close();
	return !matchPairs.empty();
}

// 保存优化结果
bool LidarMotorCalibration::SaveOptimizeResult() {
	// 保存编码器内参结果、编码器360个角度修正量结果
	if (!motorManagerPtr_->SaveCalibrationResults2(debugOptResultPath_)) {
		return false;
	}

	// 保存激光器内参结果
	/*if (!bagManagerPtr_->GetLidarIntrParam()->Save(debugOptResultPath_ + "lidar_intrisic_param.txt")) {
		return false;
	}*/

	LOG(INFO) << "Calibration results write to: " << debugOptResultPath_;
	return true;
}

bool LidarMotorCalibration::CheckOptimizeResult() {
	// 这里读入的是初始标定参global系下的点云
	BaseCloudPtr fullCloud(new BaseCloud);
	std::string fullCloudPath = debugOptDataPath_ + "full_cloud_origin.ply";
	pcl::io::loadPLYFile(fullCloudPath, *fullCloud);
	if (fullCloud->empty()) {
		LOG(INFO) << "Please run datapretrate first.";
		return false;
	}

	// 这些是优化后的参数，包括：编码器内参、编码器角度修正量、激光器内参
	auto motorOptParam = motorManagerPtr_->GetMotorIntrinsicParam2Ptr();
	auto motorAngleParam = motorManagerPtr_->GetMotorAngleCorrectParamPtr();
	auto correctAngles = motorAngleParam->GetAngles();
	auto lidarIntrParam = bagManagerPtr_->GetLidarIntrParam()->GetParam();

	std::vector<BaseCloud> laserPlys1(16);	// 调试输出16根激光点云
	std::vector<BaseCloud> laserPlys2(2);	// 调试输出大于小于0点云

	for (int id = 0; id < fullCloud->size(); ++id) {
		BasePoint& pt = fullCloud->points[id];

		double ptOri[3] = { pt.x, pt.y, pt.z };
		// 注：这里的normal实际是locla系下的点坐标，并不是真的法向
		double ptNOri[3] = { pt.normal_x, pt.normal_y, pt.normal_z };
		double ptCorrect[3], ptNCorrect[3];

		int sFrontId, sBackId;
		double sRatio, sAngle;
		PoseD sPoseOri, sPoseOpt;

		// 注：这里是根据当前时间戳，内插原始编码器参数下的点的编码器pose
		// 使用这个pose的逆，可以将读入的global系下的点转到local系下
		if (!motorManagerPtr_->GetMotorPose4geosun(pt.angle, sPoseOri, false)) {
			return false;
		}

		double sCorrectAngle = (pt.angle ) * DEG2RAD;
		sPoseOri.SetInverse();
		

		CeresHelper<double>::CorrectPoint4geosun(ptOri, ptNOri, ptCorrect, ptNCorrect,
			motorOptParam->qAlphaX, motorOptParam->qAlphaY,
			motorOptParam->dP, sCorrectAngle, sPoseOri);
#ifdef lvtu
		if (!motorManagerPtr_->GetMotorPoseInfoOrigin(pt.time, sPoseOri)) {
			return false;
		}

		// 注：这里内插的是优化后的360个角度修正量对应的角度修正值
		if (!motorAngleParam->Slerp(pt.angle, sFrontId, sBackId, sRatio)) {
			return false;
		}

		// 求逆后，用于将global系点转到local系
		sPoseOri.SetInverse();

		// 根据原始编码器角度值，结合优化后360个角度修正值对应的值，得到优化
		// 编码器内参后，编码器最终的编码器角度值
		double sDeltAngle = (1.0 - sRatio) * correctAngles[sFrontId] + 
							 sRatio * correctAngles[sBackId];
		double sCorrectAngle = (pt.angle - sDeltAngle) * DEG2RAD;

		// 根据优化后的编码器内参、编码器角度修正值、激光器内参，重新解算修正后点云和法向
		// 由于初始的三种优化参数都是0，这里这么用应该是可以的
		CeresHelper<double>::CorrectPoint2(ptOri, ptNOri, ptCorrect, ptNCorrect, 
										   motorOptParam->qAlphaX, motorOptParam->qAlphaZ, 
										   motorOptParam->dP, sCorrectAngle, 
										   lidarIntrParam[pt.ring], sPoseOri);
#endif
		pt.x = ptCorrect[0];
		pt.y = ptCorrect[1];
		pt.z = ptCorrect[2];

		if (debugFlag_) {
			laserPlys1[pt.ring].push_back(pt);

			// normal存的是原始点坐标
			if (pt.normal_x > 0) {
				laserPlys2[0].push_back(pt);
			}
			else {
				laserPlys2[1].push_back(pt);
			}
		}

		pt.normal_x = ptNCorrect[0];
		pt.normal_y = ptNCorrect[1];
		pt.normal_z = ptNCorrect[2];
	}

	PlyIo::SavePLYFileBinary(debugOptDataPath_ + "full_cloud_correct.ply", *fullCloud);
	if (debugFlag_) {
		for (int i = 0; i < 16; i++) {
			PlyIo::SavePLYFileBinary(debugOptDataPath_ + std::to_string(i) + "_laser.ply", laserPlys1[i]);
		}

		PlyIo::SavePLYFileBinary(debugOptDataPath_ + "x_large_0.ply", laserPlys2[0]);
		PlyIo::SavePLYFileBinary(debugOptDataPath_ + "x_small_0.ply", laserPlys2[1]);
	}
	return true;
}

template <typename T>
bool LidarMotorCalibrationConfig::GetData(const YAML::Node& node, T& value) {
	if (!node.IsDefined()) {
		return false;
	}
	value = node.as<T>();
	return true;
}

bool LidarMotorCalibrationConfig::LoadConfigFile(const std::string& path) {
	YAML::Node rootNode;

	try {
		rootNode = YAML::LoadFile(path);
	}
	catch (std::exception& exc) {
		LOG(INFO) << "Load calibration file failed: " 
				  << exc.what() << ", path: " << path;
		return false;
	}

	bagManagerCfgPtr = std::make_shared<BagManagerConfig>();
	motorCalibParam = std::make_shared<MotorCalibrationParam>();
	motorAngleCorrectParamPtr = std::make_shared<MotorAngleCorrectParam>();

	try {
		const auto& calibNode = rootNode["calibration_config"];
		GetData(calibNode["optimize_motor_intrisinc_param"], optimizeMotorIntrisincParam);
		GetData(calibNode["optimize_motor_angle_correct_param"], optimizeMotorAngleCorrParam);
		GetData(calibNode["optimize_lidar_intrisinc_param"], optimizeLidarIntrisincParam);
		
		const auto& bagNode = rootNode["bag_decode"];
		std::string lidarTopic, motorTopic;
		GetData(bagNode["bag_path"], bagManagerCfgPtr->bagPath);
		GetData(bagNode["srcpts_path"], bagManagerCfgPtr->srcptsPath);
		GetData(bagNode["lidar_topic"], lidarTopic);
		GetData(bagNode["motor_topic"], motorTopic);

		GetData(bagNode["process_duration"], bagManagerCfgPtr->processDuration);
		GetData(bagNode["min_detect_distance"], bagManagerCfgPtr->minDetectDist);
		GetData(bagNode["max_detect_distance"], bagManagerCfgPtr->maxDetectDist);

		GetData(bagNode["start_process_time"], bagManagerCfgPtr->startProcessTime);
		GetData(bagNode["end_process_time"], bagManagerCfgPtr->endProcessTime);

		GetData(bagNode["start_process_id"], bagManagerCfgPtr->startProcessId);
		GetData(bagNode["end_process_id"], bagManagerCfgPtr->endProcessId);
		bagManagerCfgPtr->callbackTopics.push_back(lidarTopic);
		bagManagerCfgPtr->callbackTopics.push_back(motorTopic);

		const auto& motorIntriNode = rootNode["motor_intrinsic_parameter"];
		std::vector<double> dp;
		GetData(motorIntriNode["alpha1"], motorCalibParam->alpha1);
		GetData(motorIntriNode["alpha2"], motorCalibParam->alpha2);
		GetData(motorIntriNode["alpha3"], motorCalibParam->alpha3);
		GetData(motorIntriNode["dp"], dp);
		if (dp.size() == 3) {
			motorCalibParam->dP << dp[0], dp[1], dp[2];
		}

		const auto& motorAngleNode = rootNode["motor_angle_calibration_parameter"];
		GetData(motorAngleNode["angle_correction_file"], motorAngleCorrectParamPtr->GetAngleCorrectionFile());
		GetData(motorAngleNode["lidar_intrisic_file"], motorAngleCorrectParamPtr->GetLidarCorrectionFile());
		GetData(motorAngleNode["calibration_laser_id"], calibLaserIdVec1);
	}
	catch (std::exception& exc) {
		LOG(INFO) << "Load calibration file failed: " << exc.what();
		return false;
	}

	return true;
}
}// namespace sc
