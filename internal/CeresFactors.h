#pragma once

#include "internal/Structs.h"
#include "internal/PoseD.h"
#include "internal/Utils.h"
#include "internal/CeresHelper.h"
#include "internal/TrajectoryManager.hpp"
#include "module/LidarCameraCalibration.h"
#include <ceres/ceres.h>

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
		using QuaT = Eigen::Quaternion<T>;
		QuaT qAlphaX(Eigen::AngleAxis<T>(alphaX[0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaY(Eigen::AngleAxis<T>(alphaY[0], Vec3T::UnitY()));		// 待优化编码器Z轴修正角转四元数

		Eigen::Map<const Vec3T> dP(dp);
	

		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sDeltAngle = (T(1.0) - T(sRatio_)) * angleCorrectVec[idVec_[0]] + T(sRatio_) * angleCorrectVec[idVec_[1]];

		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle) - sDeltAngle);

		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tDeltAngle = (T(1.0) - T(tRatio_)) * angleCorrectVec[idVec_[2]] + T(tRatio_) * angleCorrectVec[idVec_[3]];
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);

		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;

		CeresHelper<T>::CorrectPoint4geosun(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP, sCorrectAngle, sPose_);

		CeresHelper<T>::CorrectPoint4geosun(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP, tCorrectAngle, tPose_);

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
		using QuaT = Eigen::Quaternion<T>;
		QuaT qAlphaX(Eigen::AngleAxis<T>(alphaX[0], Vec3T::UnitX()));		// 待优化编码器X轴修正角转四元数
		QuaT qAlphaY(Eigen::AngleAxis<T>(alphaY[0], Vec3T::UnitY()));		// 待优化编码器Z轴修正角转四元数

		Eigen::Map<const Vec3T> dP(dp);


		// source点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T sGlobalPt = matchPair_.sPt.getVector3fMap().cast<T>();
		Vec3T sGlobalPtN = matchPair_.sPt.getNormalVector3fMap().cast<T>();
		T sDeltAngle = (T(1.0) - T(sRatio_)) * angleCorrectVec[idVec_[0]] + T(sRatio_) * angleCorrectVec[idVec_[1]];

		T sCorrectAngle = T(DEG2RAD) * (T(matchPair_.sPt.angle) - sDeltAngle);
		T sCorrectLidar = lidarhorizonCorrectVec[idVec_[4]];
		// target点global系下的点坐标、法向，编码器角度修正值、结合原始编码器角度后的修正角度
		Vec3T tGlobalPt = matchPair_.tPt.getVector3fMap().cast<T>();
		Vec3T tGlobalPtN = matchPair_.tPt.getNormalVector3fMap().cast<T>();
		T tDeltAngle = (T(1.0) - T(tRatio_)) * angleCorrectVec[idVec_[2]] + T(tRatio_) * angleCorrectVec[idVec_[3]];
		T tCorrectAngle = T(DEG2RAD) * (T(matchPair_.tPt.angle) - tDeltAngle);
		T tCorrectLidar = lidarhorizonCorrectVec[idVec_[5]];
		
		Vec3T sGlobalPtCorrect, sGlobalPtNCorrect;
		Vec3T tGlobalPtCorrect, tGlobalPtNCorrect;


		CeresHelper<T>::CorrectPoint4geosun4(sGlobalPt.data(), sGlobalPtN.data(), sGlobalPtCorrect.data(),
			sGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP, sCorrectAngle, lidarhorizonCorrectVec[idVec_[4]], sPose_);

		CeresHelper<T>::CorrectPoint4geosun4(tGlobalPt.data(), tGlobalPtN.data(), tGlobalPtCorrect.data(),
			tGlobalPtNCorrect.data(), qAlphaX, qAlphaY, dP, tCorrectAngle, lidarhorizonCorrectVec[idVec_[5]], tPose_);

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

}// namespace sc
