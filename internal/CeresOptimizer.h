#pragma once

#include "internal/Structs.h"
#include "internal/Parameter.hpp"
#include "internal/MotorManager.h"
#include "internal/BagManager.h"
#include "module/LidarCameraCalibration.h"
#include "internal/TrajectoryManager.hpp"
#include <ceres/ceres.h>
#include "internal/CeresFactors.h"

// namespace of sensor calibration
namespace sc {

struct CeresOptimizerConfig : public ParamterBase {
	typedef std::shared_ptr<CeresOptimizerConfig> Ptr;

	bool optimizeMotorIntrinsicParam;
	bool optimizeMotorAngleParam;
	bool optimizeLidarIntrinsicParam;

	bool optimizeExtParam;
	bool optimizeTimeDelaySeparate;
	bool optimizeTimeDelayOnlyOne;

	CeresOptimizerConfig() : optimizeMotorIntrinsicParam(true),
							 optimizeMotorAngleParam(true),
							 optimizeLidarIntrinsicParam(true),
							 optimizeExtParam(true),
							 optimizeTimeDelaySeparate(true),
							 optimizeTimeDelayOnlyOne(true) {}
};

class CeresOptimizer {
public:

	CeresOptimizer(const CeresOptimizerConfig::Ptr& config);

	~CeresOptimizer();

	// 标定编码器内参（编码器内参使用四元数）、编码器360个角度修正
	bool AddLidarMotorCalibObservation(const MotorCalibMatchPairs& matchPairs);

	// 标定编码器内参（编码器内参使用欧拉角）、编码器360个角度修正
	bool AddLidarMotorCalibObservation2(const MotorCalibMatchPairs& matchPairs);

	// 标定编码器内参（编码器内参使用欧拉角）、编码器360个角度修正、激光器内参
	bool AddLidarMotorCalibObservation4(const MotorCalibMatchPairs& matchPairs);

	bool AddLidarCameraCalibObservation(const PnPDatas& pnpDatas, const TrajectoryManager::Ptr& lidarTraj);

	// 解算
	bool Solve(bool outputFlag = true, bool resetAfterOptimize = false);

	void SetLidarMotorCalibParam(MotorManager::Ptr& motorManagerPtr, 
								 LidarIntrinsicParam::Ptr& lidarIntrParamPtr);

	void SetLidarCameraCalibParam(LidarCameraCalibrationConfig::Ptr& lidarCameraParamPtr);

private:
	void InitWithConfig();

	// 调试优化结果
	void CheckOptimizeResult();

	void SetOption();

	// 解算
	bool SolveProblem(bool outputFlag = true, bool resetAfterOptimize = false);

private:
	std::unique_ptr<ceres::Problem> problemPtr_;
	std::vector<LidarMotorCalibFactor2> factors_;
	std::vector<LidarMotorCalibFactor4> factors4_;
	CeresOptimizerConfig::Ptr configPtr_;
	MotorManager::Ptr motorManagerPtr_;
	LidarIntrinsicParam::Ptr lidarIntrParamPtr_;
	LidarCameraCalibrationConfig::Ptr lidarCameraParamPtr_;
};

}// namespace sc
