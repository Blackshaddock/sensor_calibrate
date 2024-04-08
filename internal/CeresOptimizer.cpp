#include "internal/CeresOptimizer.h"

#include <pcl/io/ply_io.h>

// namespace of sensor calibration
namespace sc {

CeresOptimizer::CeresOptimizer(const CeresOptimizerConfig::Ptr& config) 
	: configPtr_(config) {
	InitWithConfig();
}

CeresOptimizer::~CeresOptimizer() {}

void CeresOptimizer::InitWithConfig() {
	problemPtr_ = std::make_unique<ceres::Problem>();
}

void CeresOptimizer::SetLidarMotorCalibParam(MotorManager::Ptr& motorManagerPtr,
											 LidarIntrinsicParam::Ptr& lidarIntrParamPtr) {
	motorManagerPtr_ = motorManagerPtr;
	lidarIntrParamPtr_ = lidarIntrParamPtr;
}

void CeresOptimizer::SetLidarCameraCalibParam(LidarCameraCalibrationConfig::Ptr& lidarCameraParamPtr) {
	lidarCameraParamPtr_ = lidarCameraParamPtr;
}

// 标定编码器内参（编码器内参使用四元数）、编码器360个角度修正
bool CeresOptimizer::AddLidarMotorCalibObservation(const MotorCalibMatchPairs& matchPairs) {
	if (problemPtr_ == nullptr || matchPairs.empty() || motorManagerPtr_ == nullptr) {
		LOG(INFO) << "Add lidar motor calibrate observation error.";
		return false;
	}

	MotorCalibrationParam2::Ptr motorIntriParamPtr = motorManagerPtr_->GetMotorIntrinsicParam2Ptr();
	MotorAngleCorrectParam::Ptr motorAngleParamPtr = motorManagerPtr_->GetMotorAngleCorrectParamPtr();
	double* angleCorrectVec = motorAngleParamPtr->GetAngles();
	double* qAlphaX = motorIntriParamPtr->qAlphaX.coeffs().data();
	double* qAlphaZ = motorIntriParamPtr->qAlphaZ.coeffs().data();
	double* dP = motorIntriParamPtr->dP.data();

	ceres::LocalParameterization* eigenQParam = new ceres::EigenQuaternionParameterization();
	problemPtr_->AddParameterBlock(qAlphaX, 4, eigenQParam);
	problemPtr_->AddParameterBlock(qAlphaZ, 4, eigenQParam);

	for (int i = 0; i < matchPairs.size(); ++i) {
		const MotorCalibMatchPair& pair = matchPairs[i];
		if (!pair.isValid) {
			continue;
		}

		int sFrontId, sBackId, tFrontId, tBackId;
		double sRatio, tRatio, sAngle, tAngle;
		PoseD sPose, tPose;
		std::vector<int> angleCorrectIdVec;

		if (!motorManagerPtr_->GetMotorPoseInfo(pair.sPt.time, sPose, sAngle, sFrontId, sBackId, sRatio) ||
			!motorManagerPtr_->GetMotorPoseInfo(pair.tPt.time, tPose, tAngle, tFrontId, tBackId, tRatio)) {
			continue;
		}

		sPose.SetInverse();
		tPose.SetInverse();

		angleCorrectIdVec.push_back(sFrontId);
		angleCorrectIdVec.push_back(sBackId);
		angleCorrectIdVec.push_back(tFrontId);
		angleCorrectIdVec.push_back(tBackId);

		auto func = LidarMotorCalibFactor::Create(pair, sPose, tPose, sRatio, tRatio, angleCorrectIdVec, 1.0);

		problemPtr_->AddResidualBlock(func, nullptr, qAlphaX, qAlphaZ, dP, angleCorrectVec);
	}

	return problemPtr_->NumResidualBlocks();
}

// 标定编码器内参（编码器内参使用欧拉角）、编码器360个角度修正
bool CeresOptimizer::AddLidarMotorCalibObservation2(const MotorCalibMatchPairs& matchPairs) {
	if (problemPtr_ == nullptr || matchPairs.empty() || motorManagerPtr_ == nullptr) {
		LOG(INFO) << "Add lidar motor calibrate observation error.";
		return false;
	}

	MotorCalibrationParam::Ptr& motorIntriParamPtr = motorManagerPtr_->GetMotorIntrinsicParamPtr();
	MotorAngleCorrectParam::Ptr& motorAngleParamPtr = motorManagerPtr_->GetMotorAngleCorrectParamPtr();
	double* angleCorrectVec = motorAngleParamPtr->GetAngles();
	double* dP = motorIntriParamPtr->dP.data();

	double costSum = 0, costSum2 = 0;
	static int countIII = 0;

	std::string rootPath = configPtr_->debugRootDir + std::to_string(countIII++);

	int countValid = 0;

	BaseCloud sCloud, tCloud, sCloud2, tCloud2;
	for (int i = 0; i < matchPairs.size(); ++i) {
		const MotorCalibMatchPair& pair = matchPairs[i];
		if (!pair.isValid) {
			continue;
		}

		int sFrontId, sBackId, tFrontId, tBackId;
		double sRatio, tRatio, sAngle, tAngle;
		PoseD sPose, tPose;
		std::vector<int> angleCorrectIdVec;

		if (!motorManagerPtr_->GetMotorPoseInfo(pair.sPt.time, sPose, sAngle, sFrontId, sBackId, sRatio) ||
			!motorManagerPtr_->GetMotorPoseInfo(pair.tPt.time, tPose, tAngle, tFrontId, tBackId, tRatio)) {
			continue;
		}

		sPose.SetInverse();
		tPose.SetInverse();

		angleCorrectIdVec.push_back(sFrontId);
		angleCorrectIdVec.push_back(sBackId);
		angleCorrectIdVec.push_back(tFrontId);
		angleCorrectIdVec.push_back(tBackId);

		auto func = LidarMotorCalibFactor2::Create(pair, sPose, tPose, sRatio,
												   tRatio, angleCorrectIdVec, 1.0);

		problemPtr_->AddResidualBlock(func, nullptr, &motorIntriParamPtr->alpha1, 
									  &motorIntriParamPtr->alpha2, dP, angleCorrectVec);

		// 调试ceres内部优化点位是否正常，优化残差是否符合预期
		if (configPtr_->debugFlag) {
			double res[2], datas[30];
			LidarMotorCalibFactor2 lmcf(pair, sPose, tPose, sRatio, tRatio, angleCorrectIdVec, 1.0);
			lmcf.operator()(&motorIntriParamPtr->alpha1, &motorIntriParamPtr->alpha2, dP, angleCorrectVec, res, datas);
			double curRes = (res[0] * res[0] + res[1] * res[1]);
			costSum += curRes;

			if (countIII == 1) {
				factors_.emplace_back(lmcf);
			}
			else {
				factors_[countValid].operator()(&motorIntriParamPtr->alpha1, &motorIntriParamPtr->alpha2,
												dP, angleCorrectVec, res, datas);
				curRes = (res[0] * res[0] + res[1] * res[1]);
				costSum2 += curRes;
			}

			countValid++;
			BasePoint sPt, tPt, sPt2, tPt2;
			{
				sPt.x = datas[0];
				sPt.y = datas[1];
				sPt.z = datas[2];
				sPt.normal_x = datas[3];
				sPt.normal_y = datas[4];
				sPt.normal_z = datas[5];

				tPt.x = datas[6];
				tPt.y = datas[7];
				tPt.z = datas[8];
				tPt.normal_x = datas[9];
				tPt.normal_y = datas[10];
				tPt.normal_z = datas[11];


				sPt2.x = datas[12];
				sPt2.y = datas[13];
				sPt2.z = datas[14];
				sPt2.normal_x = datas[15];
				sPt2.normal_y = datas[16];
				sPt2.normal_z = datas[17];

				tPt2.x = datas[18];
				tPt2.y = datas[19];
				tPt2.z = datas[20];
				tPt2.normal_x = datas[21];
				tPt2.normal_y = datas[22];
				tPt2.normal_z = datas[23];

				sCloud.push_back(sPt);
				sCloud2.push_back(sPt2);
				tCloud.push_back(tPt);
				tCloud2.push_back(tPt2);
			}
		}
	}
	
	if (configPtr_->debugFlag) {
		pcl::io::savePLYFileBinary(rootPath + "source_ori.ply", sCloud);
		pcl::io::savePLYFileBinary(rootPath + "source_opt.ply", sCloud2);
		pcl::io::savePLYFileBinary(rootPath + "target_ori.ply", tCloud);
		pcl::io::savePLYFileBinary(rootPath + "target_opt.ply", tCloud2);
	}

	LOG(INFO) << "Initial cost: " << costSum << " " << countValid << " -> " << costSum2;
	return problemPtr_->NumResidualBlocks();
}

// 标定编码器内参（编码器内参使用欧拉角）、编码器360个角度修正、激光器内参
bool CeresOptimizer::AddLidarMotorCalibObservation4(const MotorCalibMatchPairs& matchPairs) {
	if (problemPtr_ == nullptr || matchPairs.empty() || motorManagerPtr_ == nullptr) {
		LOG(INFO) << "Add lidar motor calibrate observation error.";
		return false;
	}

	double** lidarIntriParam = lidarIntrParamPtr_->GetParam();
	MotorCalibrationParam::Ptr& motorIntriParamPtr = motorManagerPtr_->GetMotorIntrinsicParamPtr();
	MotorAngleCorrectParam::Ptr& motorAngleParamPtr = motorManagerPtr_->GetMotorAngleCorrectParamPtr();
	double* angleCorrectVec = motorAngleParamPtr->GetAngles();
	double* lidarhorizonVec = motorAngleParamPtr->GetHorizons();
	//double* dP = motorIntriParamPtr->dP.data();

	double costSum = 0, costSum2 = 0;
	static int countIII = 0;

	std::string rootPath = configPtr_->debugRootDir + std::to_string(countIII++);

	int countValid = 0;

	BaseCloud sCloud, tCloud, sCloud2, tCloud2;
	for (int i = 0; i < matchPairs.size(); ++i) {
		const MotorCalibMatchPair& pair = matchPairs[i];
		if (!pair.isValid) {
			continue;
		}

		std::vector<double*> paramBlocks;
		std::vector<double*> paramBlocks2;

		int sFrontId, sBackId, tFrontId, tBackId;
		double sRatio, tRatio, sAngle, tAngle;
		PoseD sPose, tPose;
		std::vector<int> angleCorrectIdVec, ringIdVec;

#ifdef lvtu  
		if (!motorManagerPtr_->GetMotorPoseInfo(pair.sPt.time, sPose, sAngle, sFrontId, sBackId, sRatio) ||
			!motorManagerPtr_->GetMotorPoseInfo(pair.tPt.time, tPose, tAngle, tFrontId, tBackId, tRatio)) {
			continue;
		}
#endif
		
		if (!motorManagerPtr_->GetMotorPoseInfo4geosun(pair.sPt.angle, sPose, sFrontId, sBackId, sRatio, configPtr_->optimizeMotorAngleParam) ||
			!motorManagerPtr_->GetMotorPoseInfo4geosun(pair.tPt.angle, tPose, tFrontId, tBackId, tRatio, configPtr_->optimizeMotorAngleParam)) {
			continue;
		}
		
		sAngle = pair.sPt.angle * DEG2RAD;
		tAngle = pair.tPt.angle * DEG2RAD;


		sPose.SetInverse();
		tPose.SetInverse();


		

		paramBlocks.push_back(&motorIntriParamPtr->alpha1);
		paramBlocks.push_back(&motorIntriParamPtr->alpha2);
		paramBlocks.push_back(motorIntriParamPtr->dP.data());
		if (configPtr_->optimizeMotorAngleParam)
		{
			paramBlocks.push_back(angleCorrectVec);
			angleCorrectIdVec.push_back(sFrontId);
			angleCorrectIdVec.push_back(sBackId);
			angleCorrectIdVec.push_back(tFrontId);
			angleCorrectIdVec.push_back(tBackId);
			if (configPtr_->optimizeLidarIntrinsicParam)
			{
				paramBlocks.push_back(lidarhorizonVec);
				angleCorrectIdVec.push_back(pair.sPt.ring);
				angleCorrectIdVec.push_back(pair.tPt.ring);
				auto func = LidarMotorCalibFactor4geosun4::Create(pair, sPose, tPose, sRatio, tRatio, angleCorrectIdVec, 1.0);

				problemPtr_->AddResidualBlock(func, nullptr, paramBlocks);
				//problemPtr_->SetParameterBlockConstant(paramBlocks[2]);
			}
			else {
				auto func = LidarMotorCalibFactor4geosun2::Create(pair, sPose, tPose, sRatio, tRatio, angleCorrectIdVec, 1.0);
				problemPtr_->AddResidualBlock(func, nullptr, paramBlocks);
			}
			
		}
		else {
			auto func = LidarMotorCalibFactor4geosun::Create(pair, sPose, tPose, 1.0);
			
			problemPtr_->AddResidualBlock(func, nullptr, paramBlocks);
			//problemPtr_->SetParameterBlockConstant(paramBlocks[2]);
		}
		


		
		
#ifdef lvtu
		paramBlocks.push_back(angleCorrectVec);
		if (pair.sPt.ring == pair.tPt.ring) {
			paramBlocks.push_back(lidarIntriParam[pair.sPt.ring]);
			ringIdVec.push_back(0);
			ringIdVec.push_back(0);
		}
		else {
			paramBlocks.push_back(lidarIntriParam[pair.sPt.ring]);
			paramBlocks.push_back(lidarIntriParam[pair.tPt.ring]);
			ringIdVec.push_back(0);
			ringIdVec.push_back(1);
		}

		angleCorrectIdVec.push_back(sFrontId);
		angleCorrectIdVec.push_back(sBackId);
		angleCorrectIdVec.push_back(tFrontId);
		angleCorrectIdVec.push_back(tBackId);

		auto func = LidarMotorCalibFactor4::Create(pair, sPose, tPose, sRatio,
			tRatio, angleCorrectIdVec, ringIdVec, 1.0);
#endif // DEBUG

		
		
#ifdef lvtu
		// 调试ceres内部优化点位是否正常，优化残差是否符合预期
		if (configPtr_->debugFlag) {
			double res[2], datas[30];
			LidarMotorCalibFactor4geosun lmcf(pair, sPose, tPose, 1.0);
			lmcf.operator()(paramBlocks.data(), res, datas);
			double curRes = (res[0] * res[0] + res[1] * res[1]);
			costSum += curRes;

			if (countIII == 1) {
				factors4_.emplace_back(lmcf);
			}
			else {
				factors4_[countValid].operator()(paramBlocks.data(), res, datas);
				curRes = (res[0] * res[0] + res[1] * res[1]);
				costSum2 += curRes;
			}

			countValid++;
			BasePoint sPt, tPt, sPt2, tPt2;
			{
				sPt.x = datas[0];
				sPt.y = datas[1];
				sPt.z = datas[2];
				sPt.normal_x = datas[3];
				sPt.normal_y = datas[4];
				sPt.normal_z = datas[5];

				tPt.x = datas[6];
				tPt.y = datas[7];
				tPt.z = datas[8];
				tPt.normal_x = datas[9];
				tPt.normal_y = datas[10];
				tPt.normal_z = datas[11];


				sPt2.x = datas[12];
				sPt2.y = datas[13];
				sPt2.z = datas[14];
				sPt2.normal_x = datas[15];
				sPt2.normal_y = datas[16];
				sPt2.normal_z = datas[17];

				tPt2.x = datas[18];
				tPt2.y = datas[19];
				tPt2.z = datas[20];
				tPt2.normal_x = datas[21];
				tPt2.normal_y = datas[22];
				tPt2.normal_z = datas[23];

				sCloud.push_back(sPt);
				sCloud2.push_back(sPt2);
				tCloud.push_back(tPt);
				tCloud2.push_back(tPt2);
			}
	}
#endif // lvtu

		
		
	}

	if (configPtr_->debugFlag) {
		pcl::io::savePLYFileBinary(rootPath + "source_ori.ply", sCloud);
		pcl::io::savePLYFileBinary(rootPath + "source_opt.ply", sCloud2);
		pcl::io::savePLYFileBinary(rootPath + "target_ori.ply", tCloud);
		pcl::io::savePLYFileBinary(rootPath + "target_opt.ply", tCloud2);
	}

	LOG(INFO) << "Initial cost: " << costSum << " " << countValid << " -> " << costSum2;
	return problemPtr_->NumResidualBlocks();
}

// 雷达相机外参标定
bool CeresOptimizer::AddLidarCameraCalibObservation(const PnPDatas& pnpDatas, const TrajectoryManager::Ptr& lidarTraj) {
	auto& extPoseI2C = lidarCameraParamPtr_->extPoseCam2Imu;

	int countValid = 0;
	for (int i = 0; i < pnpDatas.size(); i++) {
		auto& curData = pnpDatas[i];

		if (curData.useFlag) {
			ceres::CostFunction *func = LidarCameraCalibFactor::Create(curData, lidarTraj);
			
			if (configPtr_->optimizeTimeDelayOnlyOne) {
				// 只优化一个时间偏移
				problemPtr_->AddResidualBlock(func, nullptr, extPoseI2C.q.coeffs().data(), extPoseI2C.p.data(),
											  &lidarCameraParamPtr_->calibTimeVec.begin()->second,
											  lidarCameraParamPtr_->camIntrinsicMat.data());
			}
			else {
				// 多张照片分开优化时间偏移
				problemPtr_->AddResidualBlock(func, nullptr, extPoseI2C.q.coeffs().data(), extPoseI2C.p.data(),
											  &lidarCameraParamPtr_->calibTimeVec[curData.imageId],
											  lidarCameraParamPtr_->camIntrinsicMat.data());
			}
			countValid++;
		}
	}

	return countValid > 3;
}

bool CeresOptimizer::Solve(bool outputFlag, bool resetAfterOptimize) {
	SetOption();

	return SolveProblem(outputFlag, resetAfterOptimize);
}

void CeresOptimizer::SetOption() {
	if (motorManagerPtr_ != nullptr) {
		double* motorAngleParamPtr = motorManagerPtr_->GetMotorAngleCorrectParamPtr()->GetAngles();
		MotorCalibrationParam::Ptr& motorIntriParamPtr = motorManagerPtr_->GetMotorIntrinsicParamPtr();

		if (!configPtr_->optimizeMotorIntrinsicParam &&
			problemPtr_->HasParameterBlock(&motorIntriParamPtr->alpha1) &&
			problemPtr_->HasParameterBlock(&motorIntriParamPtr->alpha2) &&
			problemPtr_->HasParameterBlock(motorIntriParamPtr->dP.data())) {
			problemPtr_->SetParameterBlockConstant(&motorIntriParamPtr->alpha1);
			problemPtr_->SetParameterBlockConstant(&motorIntriParamPtr->alpha2);
			problemPtr_->SetParameterBlockConstant(motorIntriParamPtr->dP.data());
		}
				
		if (!configPtr_->optimizeMotorAngleParam &&
			problemPtr_->HasParameterBlock(motorAngleParamPtr)) {
			problemPtr_->SetParameterBlockConstant(motorAngleParamPtr);
		}
	}

	if (lidarIntrParamPtr_ != nullptr) {
		double** lidarIntriParam = lidarIntrParamPtr_->GetParam();
		if (!configPtr_->optimizeLidarIntrinsicParam) {
			for (int i = 0; i < 16; i++) {
				if (!configPtr_->optimizeMotorAngleParam &&
					problemPtr_->HasParameterBlock(lidarIntriParam[i])) {
					problemPtr_->SetParameterBlockConstant(lidarIntriParam[i]);
				}
			}
		}
	}

	if (lidarCameraParamPtr_ != nullptr) {
		auto& extPoseI2C = lidarCameraParamPtr_->extPoseCam2Imu;

		if (problemPtr_->HasParameterBlock(extPoseI2C.q.coeffs().data())) {
			ceres::LocalParameterization *quatParameterization = new ceres::EigenQuaternionParameterization();
			problemPtr_->AddParameterBlock(extPoseI2C.q.coeffs().data(), 4, quatParameterization);
		}

		if (problemPtr_->HasParameterBlock(lidarCameraParamPtr_->camIntrinsicMat.data())) {
			problemPtr_->SetParameterBlockConstant(lidarCameraParamPtr_->camIntrinsicMat.data());
		}

		for (auto& t : lidarCameraParamPtr_->calibTimeVec) {
			if (!configPtr_->optimizeTimeDelaySeparate && 
				!configPtr_->optimizeTimeDelayOnlyOne &&
				problemPtr_->HasParameterBlock(&t.second)) {
				problemPtr_->SetParameterBlockConstant(&t.second);
			}
		}
	}
}

bool CeresOptimizer::SolveProblem(bool outputFlag, bool resetAfterOptimize) {
	ceres::Solver::Summary summary;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 30;
	options.minimizer_progress_to_stdout = outputFlag;
	options.num_threads = configPtr_->threadNum_;
	options.function_tolerance = 1e-8;
	ceres::Solve(options, problemPtr_.get(), &summary);

	if (resetAfterOptimize) {
		problemPtr_ = std::make_unique<ceres::Problem>();
	}

	LOG(INFO) << "parameter num: " << problemPtr_->NumParameters()
			  << ", residual num: " << problemPtr_->NumResiduals()
			  << ", convergence: " << summary.IsSolutionUsable();

	return summary.IsSolutionUsable();
}

}// namespace sc
