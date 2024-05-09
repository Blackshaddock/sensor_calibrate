#pragma once

#include "internal/Structs.h"
#include "internal/PoseD.h"
#include "internal/Utils.h"
#include "internal/CeresHelper.h"
#include "internal/TrajectoryManager.hpp"
#include "module/LidarCameraCalibration.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// namespace of sensor calibration
namespace sc {

struct LidarMotorCalibFactor {
	LidarMotorCalibFactor(const MotorCalibMatchPair& pair, const PoseD& sPose, 
						  const PoseD& tPose, const double sRatio, const double tRatio, 
						  const std::vector<int>& idVec, const double weight)
		: matchPair_(pair), sPose_(sPose), tPose_(tPose),
		  sRatio_(sRatio), tRatio_(tRatio), idVec_(idVec), weight_(weight) {}

	template <typename T>
	bool operator()(const T* alphaX, const T* alphaZ, const T* dp,
					const T* angles, T* residuals) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		Eigen::Map<const QuaT> qAlphaX(alphaX);		// 待优化编码器X轴修正角转四元数
		Eigen::Map<const QuaT> qAlphaZ(alphaZ);		// 待优化编码器Z轴修正角转四元数
		Eigen::Map<const Vec3T> dP(dp);				// 待优化编码器三轴平移修正量

		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sDeltAngle = (T(1.0) - T(sRatio_)) * angles[idVec_[0]] + T(sRatio_) * angles[idVec_[1]];
		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle) - sDeltAngle);

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tDeltAngle = (T(1.0) - T(tRatio_)) * angles[idVec_[2]] + T(tRatio_) * angles[idVec_[3]];
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		CeresHelper<T>::CorrectPoint(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(), 
									 sGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, sCorrectAngle, sPose_);

		CeresHelper<T>::CorrectPoint(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
									 tGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, tCorrectAngle, tPose_);

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);
		return true;
	}

	// 分别表示：编码器 X轴四元数、Z轴四元数、三轴平移修正量、360个角度修正值关联前后两个角度值
	static ceres::CostFunction *Create(const MotorCalibMatchPair& pair,
									   const PoseD& sPose, const PoseD& tPose,
									   const double sRatio, const double tRatio,
									   const std::vector<int>& idVec, const double weight) {
		return new ceres::AutoDiffCostFunction<LidarMotorCalibFactor, 
			   2, 4, 4, 3, 360>(new LidarMotorCalibFactor(pair, sPose, tPose, sRatio, tRatio, idVec, weight));
	}

	MotorCalibMatchPair matchPair_;			// 匹配对，点坐标位于编码器global系
	PoseD sPose_;							// source点原始pose的逆，用于将编码器global系转到local系
	PoseD tPose_;							// target点原始pose的逆，用于将编码器global系转到local系
	double sRatio_;							// source点内插360个角度修正值的比例
	double tRatio_;							// target点内插360个角度修正值的比例
	double weight_;							// 权重
	std::vector<int> idVec_;				// 编码器角度插值id，4个，分别为source点front、back id 和 target点front、back id
};

struct LidarMotorCalibFactor2 {
	LidarMotorCalibFactor2(const MotorCalibMatchPair& pair, const PoseD& sPose,
						   const PoseD& tPose, const double sRatio, const double tRatio,
						   const std::vector<int>& idVec, const double weight)
		: matchPair_(pair), sPose_(sPose), tPose_(tPose),
		sRatio_(sRatio), tRatio_(tRatio), idVec_(idVec), weight_(weight) {}

	template <typename T>
	bool operator()(const T* alphaX, const T* alphaZ, const T* dp,
		const T* angles, T* residuals) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		QuaT qAlphaX(Eigen::AngleAxis<T>(alphaX[0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaZ(Eigen::AngleAxis<T>(alphaZ[0], Vec3T::UnitZ()));		// 待优化编码器Z轴修正角转四元数
		Eigen::Map<const Vec3T> dP(dp);									// 待优化编码器三轴平移修正量

		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sDeltAngle = (T(1.0) - T(sRatio_)) * angles[idVec_[0]] + T(sRatio_) * angles[idVec_[1]];
		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle) - sDeltAngle);

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tDeltAngle = (T(1.0) - T(tRatio_)) * angles[idVec_[2]] + T(tRatio_) * angles[idVec_[3]];
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		// 这里在debug优化禁用编译时，会报错
		CeresHelper<T>::CorrectPoint(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
									 sGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, sCorrectAngle, sPose_);

		CeresHelper<T>::CorrectPoint(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
									 tGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, tCorrectAngle, tPose_);

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);
		return true;
	}

	template <typename T>
	bool operator()(const T* alphaX, const T* alphaZ, const T* dp,
					const T* angles, T* residuals, T* datas) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		//QuaT qAngleY(Eigen::AngleAxis<T>(angleCorrect, Vec3T::UnitY()));

		QuaT qAlphaX(Eigen::AngleAxis<T>(alphaX[0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaZ(Eigen::AngleAxis<T>(alphaZ[0], Vec3T::UnitZ()));		// 待优化编码器Z轴修正角转四元数
		Eigen::Map<const Vec3T> dP(dp);									// 待优化编码器三轴平移修正量

																		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sDeltAngle = (T(1.0) - T(sRatio_)) * angles[idVec_[0]] + T(sRatio_) * angles[idVec_[1]];
		T sCorrectAngle = T(DEG2RAD) * ((matchPair_.sPt.angle) - sDeltAngle);

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tDeltAngle = (T(1.0) - T(tRatio_)) * angles[idVec_[2]] + T(tRatio_) * angles[idVec_[3]];
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		CeresHelper<T>::CorrectPoint(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, sCorrectAngle, sPose_);

		CeresHelper<T>::CorrectPoint(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, tCorrectAngle, tPose_);

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);

		datas[0] = sGlobalPt[0];
		datas[1] = sGlobalPt[1];
		datas[2] = sGlobalPt[2];
		datas[3] = sGlobalPtN[0];
		datas[4] = sGlobalPtN[1];
		datas[5] = sGlobalPtN[2];

		datas[6] = tGlobalPt[0];
		datas[7] = tGlobalPt[1];
		datas[8] = tGlobalPt[2];
		datas[9] = tGlobalPtN[0];
		datas[10] = tGlobalPtN[1];
		datas[11] = tGlobalPtN[2];


		datas[12] = sGlobalPtCorrect[0];
		datas[13] = sGlobalPtCorrect[1];
		datas[14] = sGlobalPtCorrect[2];
		datas[15] = sGlobalPtNCorrect[0];
		datas[16] = sGlobalPtNCorrect[1];
		datas[17] = sGlobalPtNCorrect[2];

		datas[18] = tGlobalPtCorrect[0];
		datas[19] = tGlobalPtCorrect[1];
		datas[20] = tGlobalPtCorrect[2];
		datas[21] = tGlobalPtNCorrect[0];
		datas[22] = tGlobalPtNCorrect[1];
		datas[23] = tGlobalPtNCorrect[2];

		datas[24] = residuals[0];
		datas[25] = residuals[1];

		return true;
	}

	// 分别表示：编码器 X轴四元数、Z轴四元数、三轴平移修正量、360个角度修正值关联前后两个角度值
	static ceres::CostFunction *Create(const MotorCalibMatchPair& pair,
									   const PoseD& sPose, const PoseD& tPose,
									   const double sRatio, const double tRatio,
									   const std::vector<int>& idVec, const double weight) {
		return new ceres::AutoDiffCostFunction<LidarMotorCalibFactor2,
				2, 1, 1, 3, 360>(new LidarMotorCalibFactor2(pair, sPose, tPose, sRatio, tRatio, idVec, weight));
	}

	MotorCalibMatchPair matchPair_;			// 匹配对，点坐标位于编码器global系
	PoseD sPose_;							// source点原始pose的逆，用于将编码器global系转到local系
	PoseD tPose_;							// target点原始pose的逆，用于将编码器global系转到local系
	double sRatio_;							// source点内插360个角度修正值的比例
	double tRatio_;							// target点内插360个角度修正值的比例
	double weight_;							// 权重
	std::vector<int> idVec_;				// 编码器角度插值id，4个，分别为source点front、back id 和 target点front、back id
};

struct LidarMotorCalibFactor4 {
	LidarMotorCalibFactor4(const MotorCalibMatchPair& pair, const PoseD& sPose,
		const PoseD& tPose, const double sRatio, const double tRatio,
		//const float sPtInfo[3], const float tPtInfo[3],
		const std::vector<int>& idVec, const std::vector<int>& ringIdVec, const double weight)
		: matchPair_(pair), sPose_(sPose), tPose_(tPose),
		sRatio_(sRatio), tRatio_(tRatio), idVec_(idVec), ringIdVec_(ringIdVec), weight_(weight) {
		//std::memcpy(sPtInfo_, sPtInfo, 3 * sizeof(float));
		//std::memcpy(tPtInfo_, tPtInfo, 3 * sizeof(float));
	}

	template <typename T>
	bool operator()(T const * const *parameters, T* residuals) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		QuaT qAlphaX(Eigen::AngleAxis<T>(parameters[0][0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaZ(Eigen::AngleAxis<T>(parameters[1][0], Vec3T::UnitZ()));		// 待优化编码器Z轴修正角转四元数
		Eigen::Map<const Vec3T> dP(parameters[2]);									// 待优化编码器三轴平移修正量

																		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sDeltAngle = (T(1.0) - T(sRatio_)) * parameters[3][idVec_[0]] + T(sRatio_) * parameters[3][idVec_[1]];
		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle) - sDeltAngle);

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tDeltAngle = (T(1.0) - T(tRatio_)) * parameters[3][idVec_[2]] + T(tRatio_) * parameters[3][idVec_[3]];
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		// 这里在debug优化禁用编译时，会报错
		CeresHelper<T>::CorrectPoint2(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, sCorrectAngle, parameters[4], sPose_);

		if (matchPair_.sPt.ring == matchPair_.tPt.ring) {
			CeresHelper<T>::CorrectPoint2(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
				tGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, tCorrectAngle, parameters[4], tPose_);
		}
		else {
			CeresHelper<T>::CorrectPoint2(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
				tGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, tCorrectAngle, parameters[5], tPose_);
		}

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);
		return true;
	}

	template <typename T>
	bool operator()(T const * const *parameters, T* residuals, T* datas) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		QuaT qAlphaX(Eigen::AngleAxis<T>(parameters[0][0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaZ(Eigen::AngleAxis<T>(parameters[1][0], Vec3T::UnitZ()));		// 待优化编码器Z轴修正角转四元数
		Eigen::Map<const Vec3T> dP(parameters[2]);									// 待优化编码器三轴平移修正量

																					// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sDeltAngle = (T(1.0) - T(sRatio_)) * parameters[3][idVec_[0]] + T(sRatio_) * parameters[3][idVec_[1]];
		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle) - sDeltAngle);

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tDeltAngle = (T(1.0) - T(tRatio_)) * parameters[3][idVec_[2]] + T(tRatio_) * parameters[3][idVec_[3]];
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		// 这里在debug优化禁用编译时，会报错
		CeresHelper<T>::CorrectPoint2(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, sCorrectAngle, parameters[4], sPose_);

		if (matchPair_.sPt.ring == matchPair_.tPt.ring) {
			CeresHelper<T>::CorrectPoint2(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
				tGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, tCorrectAngle, parameters[4], tPose_);
		}
		else {
			CeresHelper<T>::CorrectPoint2(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
				tGlobalPtNCorrect.data(), qAlphaX, qAlphaZ, dP, tCorrectAngle, parameters[5], tPose_);
		}

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);

		datas[0] = sGlobalPt[0];
		datas[1] = sGlobalPt[1];
		datas[2] = sGlobalPt[2];
		datas[3] = sGlobalPtN[0];
		datas[4] = sGlobalPtN[1];
		datas[5] = sGlobalPtN[2];

		datas[6] = tGlobalPt[0];
		datas[7] = tGlobalPt[1];
		datas[8] = tGlobalPt[2];
		datas[9] = tGlobalPtN[0];
		datas[10] = tGlobalPtN[1];
		datas[11] = tGlobalPtN[2];


		datas[12] = sGlobalPtCorrect[0];
		datas[13] = sGlobalPtCorrect[1];
		datas[14] = sGlobalPtCorrect[2];
		datas[15] = sGlobalPtNCorrect[0];
		datas[16] = sGlobalPtNCorrect[1];
		datas[17] = sGlobalPtNCorrect[2];

		datas[18] = tGlobalPtCorrect[0];
		datas[19] = tGlobalPtCorrect[1];
		datas[20] = tGlobalPtCorrect[2];
		datas[21] = tGlobalPtNCorrect[0];
		datas[22] = tGlobalPtNCorrect[1];
		datas[23] = tGlobalPtNCorrect[2];

		datas[24] = residuals[0];
		datas[25] = residuals[1];

		return true;
	}

	// 分别表示：编码器 X轴四元数、Z轴四元数、三轴平移修正量、360个角度修正值关联前后两个角度值
	static ceres::CostFunction *Create(const MotorCalibMatchPair& pair,
									   const PoseD& sPose, const PoseD& tPose,
									   const double sRatio, const double tRatio,
									   //const float sPtInfo[3], const float tPtInfo[3],
									   const std::vector<int>& idVec, const std::vector<int>& ringIdVec,
									   const double weight) {

		LidarMotorCalibFactor4* factor = new LidarMotorCalibFactor4(pair, sPose, tPose, sRatio, tRatio,
																	idVec, ringIdVec, weight);
		auto costFunc = new ceres::DynamicAutoDiffCostFunction<LidarMotorCalibFactor4>(factor);

		costFunc->SetNumResiduals(2);

		costFunc->AddParameterBlock(1);		// alpha1
		costFunc->AddParameterBlock(1);		// alpha2
		costFunc->AddParameterBlock(3);		// dp
		costFunc->AddParameterBlock(360);	// motor correct angle

		if (pair.sPt.ring == pair.tPt.ring) {
			costFunc->AddParameterBlock(3);		// source & target point lidar intrinsic param
		}
		else {
			costFunc->AddParameterBlock(3);		// source point lidar intrinsic param
			costFunc->AddParameterBlock(3);		// target point lidar intrinsic param
		}
		
		return costFunc;
	}

	MotorCalibMatchPair matchPair_;			// 匹配对，点坐标位于编码器global系
	PoseD sPose_;							// source点原始pose的逆，用于将编码器global系转到local系
	PoseD tPose_;							// target点原始pose的逆，用于将编码器global系转到local系
	double sRatio_;							// source点内插360个角度修正值的比例
	double tRatio_;							// target点内插360个角度修正值的比例
	double weight_;							// 权重
	std::vector<int> idVec_;				// 编码器角度插值id，4个，分别为source点front、back id 和 target点front、back id
	std::vector<int> ringIdVec_;
											//float sPtInfo_[3];
											//float tPtInfo_[3];
};

struct LidarMotorCalibFactor4geosun
{
	LidarMotorCalibFactor4geosun(const MotorCalibMatchPair& pair, const PoseD& sPose,
		const PoseD& tPose,  const double weight)
		: matchPair_(pair), sPose_(sPose), tPose_(tPose), weight_(weight) {}

	template <typename T>
	bool operator()(const T* alphaX, const T* alphaY,  const T* dp, 
		 T* residuals) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		//Eigen::Map<const QuaT> qAlphaX(alphaX);		// 待优化编码器X轴修正角转四元数
		//Eigen::Map<const QuaT> qAlphaY(alphaY);		// 待优化编码器Z轴修正角转四元数
		//Eigen::Map<const Vec3T> dP(dp);				// 待优化编码器三轴平移修正量
		QuaT qAlphaX(Eigen::AngleAxis<T>(alphaX[0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaY(Eigen::AngleAxis<T>(alphaY[0], Vec3T::UnitY()));		// 待优化编码器Z轴修正角转四元数
		
		Eigen::Map<const Vec3T> dP(dp);
		T sRatio = T(matchPair_.sPt.angle - int(matchPair_.sPt.angle));
		int sFrontId =int(matchPair_.sPt.angle) % 360;
		int sBackId = int(matchPair_.sPt.angle +1) % 360;


		T tRatio = T(matchPair_.tPt.angle - int(matchPair_.tPt.angle));
		int  tFrontId = int(matchPair_.tPt.angle) % 360;
		int tBackId = int(matchPair_.tPt.angle + 1) % 360;
		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();

		
		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle ));

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle));

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		CeresHelper<T>::CorrectPoint4geosun(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaY,  dP, sCorrectAngle, sPose_);

		CeresHelper<T>::CorrectPoint4geosun(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), qAlphaX, qAlphaY,  dP, tCorrectAngle, tPose_);

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);
		return true;
	}

	template <typename T>
	bool operator()(T const* const* parameters, T* residuals, T* datas) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		QuaT qAlphaX(Eigen::AngleAxis<T>(parameters[0][0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaY(Eigen::AngleAxis<T>(parameters[1][0], Vec3T::UnitY()));
		QuaT qAlphaZ(Eigen::AngleAxis<T>(parameters[2][0], Vec3T::UnitZ()));		// 待优化编码器Z轴修正角转四元数
		Eigen::Map<const Vec3T> dP(parameters[3]);									// 待优化编码器三轴平移修正量

		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle));

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle));

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		// 这里在debug优化禁用编译时，会报错
		CeresHelper<T>::CorrectPoint4geosun(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaY, sCorrectAngle, sPose_);

		
		CeresHelper<T>::CorrectPoint4geosun(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP, tCorrectAngle, tPose_);
		

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);

		datas[0] = sGlobalPt[0];
		datas[1] = sGlobalPt[1];
		datas[2] = sGlobalPt[2];
		datas[3] = sGlobalPtN[0];
		datas[4] = sGlobalPtN[1];
		datas[5] = sGlobalPtN[2];

		datas[6] = tGlobalPt[0];
		datas[7] = tGlobalPt[1];
		datas[8] = tGlobalPt[2];
		datas[9] = tGlobalPtN[0];
		datas[10] = tGlobalPtN[1];
		datas[11] = tGlobalPtN[2];


		datas[12] = sGlobalPtCorrect[0];
		datas[13] = sGlobalPtCorrect[1];
		datas[14] = sGlobalPtCorrect[2];
		datas[15] = sGlobalPtNCorrect[0];
		datas[16] = sGlobalPtNCorrect[1];
		datas[17] = sGlobalPtNCorrect[2];

		datas[18] = tGlobalPtCorrect[0];
		datas[19] = tGlobalPtCorrect[1];
		datas[20] = tGlobalPtCorrect[2];
		datas[21] = tGlobalPtNCorrect[0];
		datas[22] = tGlobalPtNCorrect[1];
		datas[23] = tGlobalPtNCorrect[2];

		datas[24] = residuals[0];
		datas[25] = residuals[1];

		return true;
	}

	// 分别表示：编码器 X轴四元数、Z轴四元数、三轴平移修正量、360个角度修正值关联前后两个角度值
	static ceres::CostFunction* Create(const MotorCalibMatchPair& pair,
		const PoseD& sPose, const PoseD& tPose, const double weight) {
		return new ceres::AutoDiffCostFunction<LidarMotorCalibFactor4geosun,
			2, 1, 1, 3>(new LidarMotorCalibFactor4geosun(pair, sPose, tPose,  weight));
	}

	MotorCalibMatchPair matchPair_;			// 匹配对，点坐标位于编码器global系
	PoseD sPose_;							// source点原始pose的逆，用于将编码器global系转到local系
	PoseD tPose_;							// target点原始pose的逆，用于将编码器global系转到local系					// target点内插360个角度修正值的比例
	double weight_;							// 权重
};


struct LidarMotorCalibFactor4geosun2 {
	LidarMotorCalibFactor4geosun2(const MotorCalibMatchPair& pair, const PoseD& sPose,
		const PoseD& tPose, const double sRatio, const double tRatio,
		const std::vector<int>& idVec,  const double weight)
		: matchPair_(pair), sPose_(sPose), tPose_(tPose),
		sRatio_(sRatio), tRatio_(tRatio), idVec_(idVec), weight_(weight) {
		
	}

	template <typename T>
	bool operator()(const T* alphaX, const T* alphaY, const T* dp, const T* angleCorrectVec,
		T* residuals) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using Mat3T = Eigen::Matrix<T, 3, 3>;
		using QuaT = Eigen::Quaternion<T>;
		QuaT qAlphaX(Eigen::AngleAxis<T>(alphaX[0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaY(Eigen::AngleAxis<T>(alphaY[0], Vec3T::UnitY()));		// 待优化编码器Z轴修正角转四元数
		QuaT sAlphaX, sAlphaY;
		QuaT tAlphaX, tAlphaY;
		Mat3T matrixXY;
		T cx = cos(alphaX[0]), sx = sin(alphaX[0]);
		T cy = cos(alphaY[0]), sy = sin(alphaY[0]);
		T cz = cos(T(0)), sz = sin(T(0));
		matrixXY << cy * cz, cy* sz, -sy,
			-cx * sz + sx * sy * cz, cx* cz + sx * sy * sz, sx* cy,
			sx* sz + cx * sy * cz, -sx * cz + cx * sy * sx, cx* cy;
		
		//std::cout << matrixXY << std::endl;
		Eigen::Map<const Vec3T> dP(dp);
	

		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sFrontCorrectValue, sBackCorrectValue;
		T tFrontCorrectValue, tBackCorrectValue;
		sFrontCorrectValue = angleCorrectVec[idVec_[0]];
		sBackCorrectValue = angleCorrectVec[idVec_[1]];
		
		T sDeltAngle = (T(1.0) - T(sRatio_)) * sFrontCorrectValue + T(sRatio_) * sBackCorrectValue;

		T sCorrectAngle = T(-1.0)*T(DEG2RAD) * (T(matchPair_.sPt.angle) - sDeltAngle);


		T sCcz = cos(sCorrectAngle), sCsz = sin(sCorrectAngle);
		Mat3T smatrixZ;
		smatrixZ << sCcz, sCsz, T(0),  -sCsz, sCcz, T(0), T(0), T(0), T(1);
		sAlphaX = smatrixZ * matrixXY;
		sAlphaY = smatrixZ;
		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		
		tFrontCorrectValue = angleCorrectVec[idVec_[2]];
		tBackCorrectValue = angleCorrectVec[idVec_[3]];
		

		T tDeltAngle = (T(1.0) - T(tRatio_)) * tFrontCorrectValue + T(tRatio_) * tBackCorrectValue;
		T tCorrectAngle = T(-1.0)*T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		CeresHelper<T>::CorrectPoint4geosun(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), sAlphaX, sAlphaY, dP, sCorrectAngle, sPose_);

		T tCcz = cos(tCorrectAngle), tCsz = sin(tCorrectAngle);
		Mat3T tmatrixZ;
		tmatrixZ << tCcz, tCsz, T(0), -tCsz, tCcz, T(0), T(0), T(0), T(1);
		tAlphaX = tmatrixZ * matrixXY;
		tAlphaY = tmatrixZ;
		CeresHelper<T>::CorrectPoint4geosun(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), tAlphaX, tAlphaY, dP, tCorrectAngle, tPose_);

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);
		return true;
	}

	template <typename T>
	bool operator()(T const* const* parameters, T* residuals, T* datas) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		QuaT qAlphaX(Eigen::AngleAxis<T>(parameters[0][0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaY(Eigen::AngleAxis<T>(parameters[1][0], Vec3T::UnitY()));
		QuaT qAlphaZ(Eigen::AngleAxis<T>(parameters[2][0], Vec3T::UnitZ()));		// 待优化编码器Z轴修正角转四元数
		Eigen::Map<const Vec3T> dP(parameters[3]);									// 待优化编码器三轴平移修正量

		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle));

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle));

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		// 这里在debug优化禁用编译时，会报错
		CeresHelper<T>::CorrectPoint4geosun(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaY, sCorrectAngle, sPose_);


		CeresHelper<T>::CorrectPoint4geosun(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP, tCorrectAngle, tPose_);


		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);

		datas[0] = sGlobalPt[0];
		datas[1] = sGlobalPt[1];
		datas[2] = sGlobalPt[2];
		datas[3] = sGlobalPtN[0];
		datas[4] = sGlobalPtN[1];
		datas[5] = sGlobalPtN[2];

		datas[6] = tGlobalPt[0];
		datas[7] = tGlobalPt[1];
		datas[8] = tGlobalPt[2];
		datas[9] = tGlobalPtN[0];
		datas[10] = tGlobalPtN[1];
		datas[11] = tGlobalPtN[2];


		datas[12] = sGlobalPtCorrect[0];
		datas[13] = sGlobalPtCorrect[1];
		datas[14] = sGlobalPtCorrect[2];
		datas[15] = sGlobalPtNCorrect[0];
		datas[16] = sGlobalPtNCorrect[1];
		datas[17] = sGlobalPtNCorrect[2];

		datas[18] = tGlobalPtCorrect[0];
		datas[19] = tGlobalPtCorrect[1];
		datas[20] = tGlobalPtCorrect[2];
		datas[21] = tGlobalPtNCorrect[0];
		datas[22] = tGlobalPtNCorrect[1];
		datas[23] = tGlobalPtNCorrect[2];

		datas[24] = residuals[0];
		datas[25] = residuals[1];

		return true;
	}

	// 分别表示：编码器 X轴四元数、Z轴四元数、三轴平移修正量、360个角度修正值关联前后两个角度值
	static ceres::CostFunction* Create(const MotorCalibMatchPair& pair,
		const PoseD& sPose, const PoseD& tPose, const double sRatio, const double tRatio,
		const std::vector<int>& idVec, const double weight) {
		return new ceres::AutoDiffCostFunction<LidarMotorCalibFactor4geosun2,
			2, 1, 1, 3, 360>(new LidarMotorCalibFactor4geosun2(pair, sPose, tPose, sRatio, tRatio, idVec, weight));
	}


	MotorCalibMatchPair matchPair_;			// 匹配对，点坐标位于编码器global系
	PoseD sPose_;							// source点原始pose的逆，用于将编码器global系转到local系
	PoseD tPose_;							// target点原始pose的逆，用于将编码器global系转到local系
	double sRatio_;							// source点内插360个角度修正值的比例
	double tRatio_;							// target点内插360个角度修正值的比例
	double weight_;							// 权重
	std::vector<int> idVec_;				// 编码器角度插值id，4个，分别为source点front、back id 和 target点front、back id
	
};


struct LidarMotorCalibFactor4geosun4 {
	LidarMotorCalibFactor4geosun4(const MotorCalibMatchPair& pair, const PoseD& sPose,
		const PoseD& tPose, const double sRatio, const double tRatio,
		//const float sPtInfo[3], const float tPtInfo[3],
		const std::vector<int>& idVec, const double weight)
		: matchPair_(pair), sPose_(sPose), tPose_(tPose),
		sRatio_(sRatio), tRatio_(tRatio), idVec_(idVec), weight_(weight) {
		//std::memcpy(sPtInfo_, sPtInfo, 3 * sizeof(float));
		//std::memcpy(tPtInfo_, tPtInfo, 3 * sizeof(float));
	}


	template <typename T>
	bool operator()(const T* alphaX, const T* alphaY, const T* dp, const T* angleCorrectVec, const T* lidarhorizonCorrectVec,
		T* residuals) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using Mat3T = Eigen::Matrix<T, 3, 3>;
		using QuaT = Eigen::Quaternion<T>;
		QuaT qAlphaX(Eigen::AngleAxis<T>(alphaX[0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaY(Eigen::AngleAxis<T>(alphaY[0], Vec3T::UnitY()));	// 待优化编码器Z轴修正角转四元数
		Mat3T matrixXY;
		T cx = cos(alphaX[0]), sx = sin(alphaX[0]);
		T cy = cos(alphaY[0]), sy = sin(alphaY[0]);
		T cz = cos(T(0)), sz = sin(T(0));
		matrixXY << cy * cz, cy* sz, -sy,
			-cx * sz + sx * sy * cz, cx* cz + sx * sy * sz, sx* cy,
			sx* sz + cx * sy * cz, -sx * cz + cx * sy * sx, cx* cy;
		qAlphaX = matrixXY;


		Eigen::Map<const Vec3T> dP(dp);


		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sDeltAngle = (T(1.0) - T(sRatio_)) * angleCorrectVec[idVec_[0]] + T(sRatio_) * angleCorrectVec[idVec_[1]];

		T sCorrectAngle = T(DEG2RAD) * (T(-1*matchPair_.sPt.angle) - sDeltAngle);
		T sCorrectLidar = lidarhorizonCorrectVec[idVec_[4]];
		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tDeltAngle = (T(1.0) - T(tRatio_)) * angleCorrectVec[idVec_[2]] + T(tRatio_) * angleCorrectVec[idVec_[3]];
		T tCorrectAngle = T(-1.0)*T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);
		T tCorrectLidar = lidarhorizonCorrectVec[idVec_[5]];
		T sCcz = cos(tCorrectAngle), sCsz = sin(tCorrectAngle);
		Mat3T matrixZ;
		matrixZ << sCcz, sCsz, 0,
			-sCsz, sCcz, 0,
			0, 0, 1;
		qAlphaY = matrixZ;
		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;


		CeresHelper<T>::CorrectPoint4geosun4(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP,  sCorrectAngle, lidarhorizonCorrectVec[idVec_[4]], sPose_);

		T tCcz = cos(tCorrectAngle), tCsz = sin(tCorrectAngle);
		matrixZ << sCcz, sCsz, 0,
			-sCsz, sCcz, 0,
			0, 0, 1;
		qAlphaY = matrixZ;

		CeresHelper<T>::CorrectPoint4geosun4(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP,  tCorrectAngle, lidarhorizonCorrectVec[idVec_[5]], tPose_);

		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);
		return true;
	}

	template <typename T>
	bool operator()(T const* const* parameters, T* residuals, T* datas) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using QuaT = Eigen::Quaternion<T>;

		QuaT qAlphaX(Eigen::AngleAxis<T>(parameters[0][0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaY(Eigen::AngleAxis<T>(parameters[1][0], Vec3T::UnitY()));
		QuaT qAlphaZ(Eigen::AngleAxis<T>(parameters[2][0], Vec3T::UnitZ()));		// 待优化编码器Z轴修正角转四元数
		Eigen::Map<const Vec3T> dP(parameters[3]);									// 待优化编码器三轴平移修正量

		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle));

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle));

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		// 这里在debug优化禁用编译时，会报错
		CeresHelper<T>::CorrectPoint4geosun(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaY, sCorrectAngle, sPose_);


		CeresHelper<T>::CorrectPoint4geosun(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP, tCorrectAngle, tPose_);


		residuals[0] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(sGlobalPtNCorrect);
		residuals[1] = T(weight_) * (sGlobalPtCorrect - tGlobalPtCorrect).dot(tGlobalPtNCorrect);

		datas[0] = sGlobalPt[0];
		datas[1] = sGlobalPt[1];
		datas[2] = sGlobalPt[2];
		datas[3] = sGlobalPtN[0];
		datas[4] = sGlobalPtN[1];
		datas[5] = sGlobalPtN[2];

		datas[6] = tGlobalPt[0];
		datas[7] = tGlobalPt[1];
		datas[8] = tGlobalPt[2];
		datas[9] = tGlobalPtN[0];
		datas[10] = tGlobalPtN[1];
		datas[11] = tGlobalPtN[2];


		datas[12] = sGlobalPtCorrect[0];
		datas[13] = sGlobalPtCorrect[1];
		datas[14] = sGlobalPtCorrect[2];
		datas[15] = sGlobalPtNCorrect[0];
		datas[16] = sGlobalPtNCorrect[1];
		datas[17] = sGlobalPtNCorrect[2];

		datas[18] = tGlobalPtCorrect[0];
		datas[19] = tGlobalPtCorrect[1];
		datas[20] = tGlobalPtCorrect[2];
		datas[21] = tGlobalPtNCorrect[0];
		datas[22] = tGlobalPtNCorrect[1];
		datas[23] = tGlobalPtNCorrect[2];

		datas[24] = residuals[0];
		datas[25] = residuals[1];

		return true;
	}

	// 分别表示：编码器 X轴四元数、Z轴四元数、三轴平移修正量、360个角度修正值关联前后两个角度值
	static ceres::CostFunction* Create(const MotorCalibMatchPair& pair,
		const PoseD& sPose, const PoseD& tPose, const double sRatio, const double tRatio,
		const std::vector<int>& idVec, const double weight) {
		return new ceres::AutoDiffCostFunction<LidarMotorCalibFactor4geosun4,
			2, 1, 1, 3, 360, 16>(new LidarMotorCalibFactor4geosun4(pair, sPose, tPose, sRatio, tRatio, idVec, weight));
	}


	MotorCalibMatchPair matchPair_;			// 匹配对，点坐标位于编码器global系
	PoseD sPose_;							// source点原始pose的逆，用于将编码器global系转到local系
	PoseD tPose_;							// target点原始pose的逆，用于将编码器global系转到local系
	double sRatio_;							// source点内插360个角度修正值的比例
	double tRatio_;							// target点内插360个角度修正值的比例
	double weight_;							// 权重
	std::vector<int> idVec_;				// 编码器角度插值id，4个，分别为source点front、back id 和 target点front、back id

	
};


template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};


struct TError
{
	TError(double t_x, double t_y, double t_z, double var)
		:t_x(t_x), t_y(t_y), t_z(t_z), var(var) {}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);
		residuals[2] = (tj[2] - T(t_z)) / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var)
	{
		return (new ceres::AutoDiffCostFunction<
			TError, 3, 3>(
				new TError(t_x, t_y, t_z, var)));
	}

	double t_x, t_y, t_z, var;

};

struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z,
		double q_w, double q_x, double q_y, double q_z,
		double t_var, double q_var)
		:t_x(t_x), t_y(t_y), t_z(t_z),
		q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
		t_var(t_var), q_var(q_var) {}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T i_q_w[4];
		QuaternionInverse(w_q_i, i_q_w);

		T t_i_ij[3];
		ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		T relative_q[4];
		relative_q[0] = T(q_w);
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T q_i_j[4];
		ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

		residuals[3] = T(2) * error_q[1] / T(q_var);
		residuals[4] = T(2) * error_q[2] / T(q_var);
		residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
		const double q_w, const double q_x, const double q_y, const double q_z,
		const double t_var, const double q_var)
	{
		return (new ceres::AutoDiffCostFunction<
			RelativeRTError, 6, 4, 3, 4, 3>(
				new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};























struct LidarCameraCalibFactor {
	LidarCameraCalibFactor(const PnPData& data, const TrajectoryManager::Ptr& traj)
		: data_(data), lidarTraj_(traj) {}

	template <typename T>
	bool operator()(const T *q, const T *t, const T *dt, const T *intrisic, T *residual) const {
		using Vec3T = Eigen::Matrix<T, 3, 1>;
		using Mat3T = Eigen::Matrix<T, 3, 3>;
		using QuaT = Eigen::Quaternion<T>;

		Eigen::Map<const QuaT> qC2I(q);					// imu 到 camera 的转换
		Eigen::Map<const Vec3T> tC2I(t);				// 
		Eigen::Map<const Mat3T> camIntrMat(intrisic);	// 相机内参

		T inputImageTime = T(data_.time);
		T correctImageTime = inputImageTime + dt[0];	// 修正时间偏移

		QuaT qW2I;
		Vec3T tW2I;
		lidarTraj_->slerp2(correctImageTime, tW2I, qW2I);

		QuaT qI2W = qW2I.inverse();
		Vec3T tI2W = T(-1) * (qI2W * tW2I);

		Vec3T pt_in_I = qI2W * data_.pt3D.cast<T>() + tI2W;

		Vec3T ptInImg = qC2I * pt_in_I + tC2I;

		T fx = T(camIntrMat(0, 0));
		T fy = T(camIntrMat(1, 1));
		T cx = T(camIntrMat(0, 2));
		T cy = T(camIntrMat(1, 2));
		T scale = T(1);
		T u = (ptInImg[0] * fx / ptInImg[2] + cx) * scale;
		T v = (ptInImg[1] * fy / ptInImg[2] + cy) * scale;

		T deltU = (u - T(data_.uv[0]));
		T deltV = (v - T(data_.uv[1]));

		residual[0] = deltU * deltU + deltV * deltV;

		return true;
	}

	static ceres::CostFunction *Create(const PnPData& data, const TrajectoryManager::Ptr& traj) {
		LidarCameraCalibFactor* factor = new LidarCameraCalibFactor(data, traj);
		return new ceres::AutoDiffCostFunction<LidarCameraCalibFactor, 1, 4, 3, 1, 9>(factor);
	}

	PnPData data_;
	TrajectoryManager::Ptr lidarTraj_;
};

template <typename T> inline
void TranPoint(const T t[3], T points[3],  T result[3])
{
	result[0] = points[0] + t[0];
	result[1] = points[1] + t[1];
	result[2] = points[2] + t[2];
};


struct LidarCaliBoardFactor {
	LidarCaliBoardFactor(const Eigen::Vector3d & planeNormal, const Eigen::Vector3d& planePoint, const std::vector<double> distancevec):planeNormal(planeNormal), planePoint(planePoint), distancevec(distancevec) {}

	template <typename T>
	bool operator()(const T* points, const T* r, const T* t, const T* residual)
	{
		using Vec3T = Eigen::Matrix<T, 3, 1>;

		Vec3T pointA, pointB, pointC, pointD;
		pointA[0] = points[0]; pointA[1] = points[1]; pointA[2] = points[2];
		pointB[0] = points[3]; pointB[1] = points[4]; pointB[2] = points[5];
		pointC[0] = points[6]; pointC[1] = points[7]; pointC[2] = points[8];
		pointD[0] = points[9]; pointD[1] = points[10]; pointD[2] = points[11];

		Vec3T cameraA, cameraB, cameraC, cameraD;
		cameraA[0] = camPtA[0]; cameraA[1] = camPtA[1]; cameraA[2] = camPtA[2];
		cameraB[0] = camPtB[0]; cameraB[1] = camPtB[1]; cameraB[2] = camPtB[2];
		cameraC[0] = camPtC[0]; cameraC[1] = camPtC[1]; cameraC[2] = camPtC[2];
		cameraD[0] = camPtD[0]; cameraD[1] = camPtD[1]; cameraD[2] = camPtD[2];
		//将相机归一化平面的点转到雷达坐标系下
		T camlidarA, camlidarB, camlidarC, camlidarD;
		ceres::QuaternionRotatePoint(q, cameraA, camlidarA);
		TranPoint(t, camlidarA, camlidarA);
		ceres::QuaternionRotatePoint(q, cameraB, camlidarB);
		TranPoint(t, camlidarB, camlidarB);
		ceres::QuaternionRotatePoint(q, cameraC, camlidarC);
		TranPoint(t, camlidarC, camlidarC);
		ceres::QuaternionRotatePoint(q, cameraD, camlidarD);
		TranPoint(t, camlidarD, camlidarD);


		//通过点云拟合的面和转换后的归一化平面得到交点坐标
		T t = T(planePoint[0]) * T(planeNormal[0]) + T(planePoint[1]) * T(planeNormal[1]) + T(planePoint[2]) * T(planeNormal[2]);
		cameraA = t * cameraA; cameraB = t * cameraB; cameraC = t * cameraC; cameraD = t * cameraD;



		//点-点的残差用于估计R T；
		residual[0] = (PointA - camlidarA) * (PointA - camlidarA);
		residual[1] = (PointB - camlidarB) * (PointB - camlidarB);
		residual[2] = (PointC - camlidarC) * (PointC - camlidarC);
		residual[3] = (PointD - camlidarD) * (PointD - camlidarD);

		T planept[3]; planept[0] = planePoint[0]; planept[1] = planePoint[1]; planept[2] = planePoint[2];
		//点-面的距离公式 用于估计3D点
		residual[4] = (PointA - planept) * Vec3T(planeNormal[0], planeNormal[1], planeNormal[2]);
		residual[5] = (PointA - planept) * Vec3T(planeNormal[0], planeNormal[1], planeNormal[2]);
		residual[6] = (PointA - planept) * Vec3T(planeNormal[0], planeNormal[1], planeNormal[2]);
		residual[7] = (PointA - planept) * Vec3T(planeNormal[0], planeNormal[1], planeNormal[2]);













		

	}


	Eigen::Vector3d planeNormal;    //平面法向量
	Eigen::Vector3d planePoint;     //平面中的一点
	std::vector<double> distancevec;   // 方向向量 用于固定旋转
	Eigen::Vector3d camPtA, camPtB, camPtC, camPtD;    //相机坐标系归一化平面四个点

};
}// namespace sc
