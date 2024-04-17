#pragma once

#include "internal/PoseD.h"

// namespace of sensor calibration
namespace sc {

template <typename T>
class CeresHelper {
public:
	typedef Eigen::Matrix<T, 3, 1>	Vec3T;
	typedef Eigen::Quaternion<T>	QuaT;
	typedef Eigen::Matrix<T, 3, 3>  Mat3T;
	CeresHelper() = default;
	~CeresHelper() = default;
	
	// ���������ݱ궨��ı������ڲκͱ������Ƕ�����ֵ��������ǰglobalϵ�µĵ������뷨��
	// pointGlobal��������globalϵ�µ�
	// pointGlobalNormal��������globalϵ�µ�ķ���
	// pointGlobalCorrect�����ݱ궨���������globalϵ�µĵ�
	// pointGlobalNormalCorrect�����ݱ궨���������globalϵ�µĵ�ķ���
	// qAlphaX���������ڲΣ���X��������ת��Ԫ��
	// qAlphaZ���������ڲΣ���Z��������ת��Ԫ��
	// dP���������ڲΣ��������ƽ��������
	// angleCorrect����ǰ��ʱ�̶�Ӧ�������Ƕ�ֵ + ��Ӧ��ʱ��360���Ƕ�����ֵ����������ı������Ƕȹ۲�ֵ
	// pose����ǰ��ʱ�̱�����globalϵתlocalϵ��pose
	static void CorrectPoint(const T* pointGlobal, const T* pointGlobalNormal,
							 T* pointGlobalCorrect, T* pointGlobalNormalCorrect, 
							 const QuaT& qAlphaX, const QuaT& qAlphaZ, const Vec3T& dP, 
							 const T angleCorrect, const PoseD& pose) {
		Eigen::Map<const Vec3T> ptGlobal(pointGlobal);
		Eigen::Map<const Vec3T> ptGlobalN(pointGlobalNormal);
		Eigen::Map<Vec3T> ptGlobalCorrect(pointGlobalCorrect);
		Eigen::Map<Vec3T> ptGlobalNCorrect(pointGlobalNormalCorrect);
		QuaT qAngleY(Eigen::AngleAxis<T>(angleCorrect, Vec3T::UnitY()));

		Vec3T ptLocal = pose.q.cast<T>() * ptGlobal + pose.p.cast<T>();
		Vec3T ptLocalN = pose.q.cast<T>() * ptGlobalN;
		
		// ����ԭ����д��
		//ptGlobalCorrect = qAngleY * (qAlphaZ * qAlphaX * ptLocal + dP);

		QuaT dQ = qAngleY * qAlphaZ * qAlphaX;
		ptGlobalCorrect = dQ * ptLocal + qAngleY * dP;
		ptGlobalNCorrect = dQ * ptLocalN;
	}

	static void CorrectPoint4geosun2(const T* pointGlobal, const T* pointGlobalNormal,
		T* pointGlobalCorrect, T* pointGlobalNormalCorrect,
		const QuaT& qAlphaX, const QuaT& qAlphaY, const Vec3T& dP,
		const T angleCorrect, const T* lidarIntrAngle, const PoseD& pose) {
		Eigen::Map<const Vec3T> ptGlobal(pointGlobal);
		Eigen::Map<const Vec3T> ptGlobalN(pointGlobalNormal);
		Eigen::Map<Vec3T> ptGlobalCorrect(pointGlobalCorrect);
		Eigen::Map<Vec3T> ptGlobalNCorrect(pointGlobalNormalCorrect);
		QuaT qAngleZ(Eigen::AngleAxis<T>(angleCorrect, Vec3T::UnitZ()));

		Vec3T ptLocal = pose.q.cast<T>() * ptGlobal + pose.p.cast<T>();
		Vec3T ptLocalN = pose.q.cast<T>() * ptGlobalN;
		Vec3T ptLocalInfo, ptLocalCorrect;

		GetPointInfo(ptLocal.data(), ptLocalInfo.data());
		CorrectPointUseLidarIntrParam(ptLocalInfo.data(), lidarIntrAngle, ptLocalCorrect.data());

		// ����ԭ����д��
		//ptGlobalCorrect = qAngleY * (qAlphaZ * qAlphaX * ptLocalCorrect + dP);

		QuaT dQ = qAngleZ * qAlphaY * qAlphaX;
		ptGlobalCorrect = dQ * ptLocalCorrect + qAngleZ * dP;
		ptGlobalNCorrect = dQ * ptLocalN;
	}



	static void CorrectPoint4geosun(const T* pointGlobal, const T* pointGlobalNormal,
		T* pointGlobalCorrect, T* pointGlobalNormalCorrect,
		const QuaT& qAlphaX, const QuaT& qAlphaY,  const Vec3T& dP,
		const T angleCorrect, const PoseD& pose) {
		Eigen::Map<const Vec3T> ptGlobal(pointGlobal);
		Eigen::Map<const Vec3T> ptGlobalN(pointGlobalNormal);
		Eigen::Map<Vec3T> ptGlobalCorrect(pointGlobalCorrect);
		Eigen::Map<Vec3T> ptGlobalNCorrect(pointGlobalNormalCorrect);
		QuaT qAngleZ(Eigen::AngleAxis<T>(angleCorrect, Vec3T::UnitZ()));

		Vec3T ptLocal = pose.q.cast<T>() * ptGlobal + pose.p.cast<T>();
		Vec3T ptLocalN = pose.q.cast<T>() * ptGlobalN;

		// ����ԭ����д��
		//ptGlobalCorrect = qAngleY * (qAlphaZ * qAlphaX * ptLocal + dP);

		QuaT dQ = qAlphaY * qAlphaX;
		ptGlobalCorrect = qAlphaX * ptLocal + qAlphaY * dP;
		ptGlobalNCorrect = qAlphaX * ptLocalN;
	}


	static void CorrectPoint4geosun4(const T* pointGlobal, const T* pointGlobalNormal,
		T* pointGlobalCorrect, T* pointGlobalNormalCorrect,
		const QuaT& qAlphaX, const QuaT& qAlphaY, const Vec3T& dP,
		const T angleCorrect, const T horizonCorrect, const PoseD& pose) {
		Eigen::Map<const Vec3T> ptGlobal(pointGlobal);
		Eigen::Map<const Vec3T> ptGlobalN(pointGlobalNormal);
		Eigen::Map<Vec3T> ptGlobalCorrect(pointGlobalCorrect);
		Eigen::Map<Vec3T> ptGlobalNCorrect(pointGlobalNormalCorrect);
		

		QuaT qAngleZ(Eigen::AngleAxis<T>(angleCorrect, Vec3T::UnitZ()));
		
		Vec3T ptLocal = pose.q.cast<T>() * ptGlobal + pose.p.cast<T>();
		Vec3T ptLocalN = pose.q.cast<T>() * ptGlobalN;
		Vec3T ptLocalInfo, ptLocalCorrect;

		GetPointInfo(ptLocal.data(), ptLocalInfo.data());
		CorrectPointUseLidarIntrParam(ptLocalInfo.data(), &horizonCorrect, ptLocalCorrect.data());
		
		QuaT dQ = qAlphaY * qAlphaX;
		ptGlobalCorrect = dQ * ptLocalCorrect + qAlphaY * dP;
		ptGlobalNCorrect = dQ * ptLocalN;

		// ����ԭ����д��
		//ptGlobalCorrect = qAngleY * (qAlphaZ * qAlphaX * ptLocal + dP);

		/*QuaT dQ = qAngleZ * qAlphaY * qAlphaX;
		ptGlobalCorrect = dQ * ptLocalCorrect + qAngleZ * dP;
		ptGlobalNCorrect = dQ * ptLocalN;*/
	}


	static void CorrectPoint2(const T* pointGlobal, const T* pointGlobalNormal,
		T* pointGlobalCorrect, T* pointGlobalNormalCorrect,
		const QuaT& qAlphaX, const QuaT& qAlphaY, const Vec3T& dP,
		const T angleCorrect, const T* lidarIntrAngle, const PoseD& pose) {
		Eigen::Map<const Vec3T> ptGlobal(pointGlobal);
		Eigen::Map<const Vec3T> ptGlobalN(pointGlobalNormal);
		Eigen::Map<Vec3T> ptGlobalCorrect(pointGlobalCorrect);
		Eigen::Map<Vec3T> ptGlobalNCorrect(pointGlobalNormalCorrect);
		QuaT qAngleZ(Eigen::AngleAxis<T>(angleCorrect, Vec3T::UnitZ()));

		Vec3T ptLocal = pose.q.cast<T>() * ptGlobal + pose.p.cast<T>();
		Vec3T ptLocalN = pose.q.cast<T>() * ptGlobalN;
		Vec3T ptLocalInfo, ptLocalCorrect;

		GetPointInfo(ptLocal.data(), ptLocalInfo.data());
		CorrectPointUseLidarIntrParam(ptLocalInfo.data(), lidarIntrAngle, ptLocalCorrect.data());

		// ����ԭ����д��
		//ptGlobalCorrect = qAngleY * (qAlphaZ * qAlphaX * ptLocalCorrect + dP);

		QuaT dQ = qAngleZ * qAlphaY * qAlphaX;
		ptGlobalCorrect = dQ * ptLocalCorrect + qAngleZ * dP;
		ptGlobalNCorrect = dQ * ptLocalN;
	}

	// Ŀǰ���Ż�ˮƽ�Ǻ;���������������
	static void CorrectPointUseLidarIntrParam(const T* pointInfo, const T* laserParam, 
											  T* pointLocalCorrect) {
		T correctInfo[3];
		correctInfo[0] = pointInfo[0] + laserParam[0];
		//correctInfo[1] = pointInfo[1] + laserParam[1];
		//correctInfo[2] = pointInfo[2] + laserParam[2];
		correctInfo[1] = pointInfo[1];
		correctInfo[2] = pointInfo[2];

		PointParse(correctInfo[0], correctInfo[1], correctInfo[2], pointLocalCorrect);
	}

	// ԭʼVLP16���뷽ʽ��
	// omega����ֱ��
	// alpha��ˮƽ��
	// R������

	// X = R * cos(omega) * sin(alpha)
	// Y = R * cos(omega) * cos(alpha)
	// Z = R * sin(omega)

	// ������ݵ����꣬�����Ļ�����Ϣ��omega��alpha��R
	// pointInfo���洢˳�� omega��alpha��R  --->  ��ֱ�ǡ�ˮƽ�ǡ�����
	static bool GetPointInfo(const T* pointLocal, T* pointInfo) {
		Eigen::Map<const Vec3T> ptLocal(pointLocal);
		T pointCorrect[3];

		T R = ptLocal.norm();
		T omega = ceres::asin(pointLocal[2] / R);
		T alpha = ceres::acos(pointLocal[1] / (R * ceres::cos(omega)));

		PointParse(omega, alpha, R, pointCorrect);

		if (ceres::abs(pointCorrect[0] - pointLocal[0]) > 1e-2 ||
			ceres::abs(pointCorrect[1] - pointLocal[1]) > 1e-2 ||
			ceres::abs(pointCorrect[2] - pointLocal[2]) > 1e-2) {
			alpha = T(-1) * alpha;
			pointCorrect[0] = R * ceres::cos(omega) * ceres::sin(alpha);
			pointCorrect[1] = R * ceres::cos(omega) * ceres::cos(alpha);
		}

		pointInfo[0] = omega;
		pointInfo[1] = alpha;
		pointInfo[2] = R;

		if (ceres::abs(pointCorrect[0] - pointLocal[0]) > 1e-2 ||
			ceres::abs(pointCorrect[1] - pointLocal[1]) > 1e-2 ||
			ceres::abs(pointCorrect[2] - pointLocal[2]) > 1e-2) {
			std::cout << "Get point info error: " << pointLocal[0] << " " 
					  << pointLocal[1] << " " << pointInfo[2] << " -> " 
					  << pointCorrect[0] << " " << pointCorrect[1] 
					  << " " << pointCorrect[2] << std::endl;
			return false;
		}
		/*std::cout << ptLocal[0] << " " << ptLocal[1] << " " << ptLocal[2] << " "   << pointCorrect[0] << " " << pointCorrect[1] << " " << pointCorrect[2] << " " << pointInfo[0] <<
			" " << pointInfo[1] << " " << pointInfo[2]  <<  std::endl;*/
		return true;
	}

	static void PointParse(const T& omega, const T& alpha, const T& R, T* point) {
		point[0] = R * ceres::cos(omega) * ceres::sin(alpha);
		point[1] = R * ceres::cos(omega) * ceres::cos(alpha);
		point[2] = R * ceres::sin(omega);
	}
};
}// namespace sc
