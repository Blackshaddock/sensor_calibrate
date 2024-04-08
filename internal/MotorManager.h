#pragma once
#include "Bag2File.h"
#include "internal/PoseD.h"
#include "internal/Parameter.hpp"
#include <memory>
#include <vector>
#include <fstream>

// namespace of sensor calibration
namespace sc {

// �������궨������ԭʼconfig��������
struct MotorCalibrationParam : public ParamterBase {
	typedef std::shared_ptr<MotorCalibrationParam> Ptr;

	double alpha1;			// ������ X �������ǣ���
	double alpha2;			// ������ Y �������ǣ���
	double alpha3;			// ������ z��������
	Eigen::Vector3d dP;		// ����������ƽ��������
	double startAngle;		// ��ʼʱ�̵ĽǶȣ����ֵ��֪��������ã�ԭ����

	MotorCalibrationParam() : alpha1(0.), alpha2(0.), alpha3(0.),
							  startAngle(0.) {}
};

// �������궨��������ͨ��MotorCalibrationParamת���õ�������궨ʹ��
struct MotorCalibrationParam2 {
	typedef std::shared_ptr<MotorCalibrationParam2> Ptr;

	Eigen::Quaterniond qAlphaX;		// ������ X �������ǣ�ת��Ԫ��
	Eigen::Quaterniond qAlphaY;		// geosun Z��ɹ� ת��֮��
	Eigen::Quaterniond qAlphaZ;	    // ������ Z �������ǣ�ת��Ԫ�� ����Y��ɹ� ת��֮��
	
	Eigen::Vector3d dP;				// ������ƽ��������
	
	double startAngle;				// ��ʼʱ�̵ĽǶȣ����ֵ��֪��������ã�ԭ����
};

struct MotorData {
	double time;
	double angle;	// ��

	MotorData() : time(0.), angle(0.) {}

	bool operator<(const MotorData& other) const {
		return this->time < other.time;
	}
};
typedef std::vector<MotorData> MotorDatas;

// ������360���Ƕ�����ֵ�������궨ʹ��
struct MotorAngleCorrectParam {
public:
	typedef std::shared_ptr<MotorAngleCorrectParam> Ptr;

	MotorAngleCorrectParam() {
		angles.resize(360, 0.);
		lidarhorizons.resize(16, 0.);
	}

	double* GetAngles() { 
		std::cout <<"angles num: " <<  angles.size() << std::endl;
		return angles.data(); }

	double* GetHorizons() {
		std::cout << "lidarhorizons num: " << lidarhorizons.size() << std::endl;
		return lidarhorizons.data();
	}

	double Slerp(const double angle, int& frontId, int& backId, double& ratio) const {
		double absAngle = std::abs(angle);

		frontId = int(absAngle) % 360;
		backId  = int(absAngle + 1) % 360;

		double frontAngle = angles[frontId];
		double backAngle = angles[backId];

		ratio = absAngle - int(absAngle);

		return (1 - ratio) * frontAngle + ratio * backAngle;
	}

	std::string& GetAngleCorrectionFile() { return angleCorrectionFile; }

	std::string& GetLidarCorrectionFile() { return lidarCorrectionFile; }

	bool LoadlidarCorrectionFile() {
		std::ifstream ifs(lidarCorrectionFile);
		if (!ifs.is_open()) {
			return false;
		}

		int countId = 0;
		std::string line;
		while (std::getline(ifs, line)) {
			angles[countId] = std::stod(line);
			countId++;
		}

		return countId == 360;
	}


	bool LoadAngleCorrectionFile() {
		std::ifstream ifs(angleCorrectionFile);
		if (!ifs.is_open()) {
			return false;
		}

		int countId = 0;
		std::string line;
		while (std::getline(ifs, line)) {
			angles[countId] = std::stod(line);
			countId++;
		}

		return countId == 16;
	}



private:
	std::vector<double> angles;			// ��
	std::string angleCorrectionFile;	// �����ļ�����

	std::vector<double> lidarhorizons;
	std::string lidarCorrectionFile;    
};

class MotorManager {
public:
	typedef std::shared_ptr<MotorManager> Ptr;

	MotorManager();

	~MotorManager();

	// bag��������������
	bool ParseMotorData(BagFileReader* bagReader);

	// ��������������
	bool SaveMotorData(const std::string& path);

	// ����궨����������������ڲ��ļ� �� ������360���Ƕ������ļ�
	bool SaveCalibrationResults(const std::string& rootDir);

	// ����궨����������������ڲ��ļ� �� ������360���Ƕ������ļ�
	bool SaveCalibrationResults2(const std::string& rootDir);

	// �������������
	bool LoadMotorData(const std::string& path);


	//
	void SetMotorParameter(const MotorCalibrationParam::Ptr& intrinsicParam, const MotorAngleCorrectParam::Ptr& angleParam);
	// ���ñ������ڲΣ����궨ֵ
	void SetMotorParameter(const MotorCalibrationParam::Ptr& intrinsicParam, 
						   const MotorAngleCorrectParam::Ptr& angleParam,
						   const double bagStartTime);

	// ����ʱ������ڲ�������Ƕ�����
	bool Slerp(const double time, double& motorAngle);

	// ����ʱ������ڲ�������Ƕȣ���ϱ������ڲΣ�����pose��
	// ��pose����ת����������������ϵ�����ݵ�������ȫ������ϵ��
	bool GetMotorPose(const double time, PoseD& pose, double& angle);

	//����ÿ����ĽǶȣ���ȡ��Ӧ��pose
	bool GetMotorPose4geosun(double angle, PoseD& pose, bool UseMotorAngleCorr);

	bool GetMotorPoseInfo4geosun(double angle, PoseD& pose, int& frontId, int& backId, double& ratio, bool UseMotorAngleCorr);

	bool GetMotorPoseInfo(const double time, PoseD& pose, double& angle, 
						  int& frontId, int& backId, double& ratio);

	bool GetMotorPoseInfoOrigin(const double time, PoseD& pose);

	MotorDatas& GetMotorDatas();

	MotorCalibrationParam::Ptr& GetMotorIntrinsicParamPtr();

	MotorCalibrationParam2::Ptr& GetMotorIntrinsicParam2Ptr();

	MotorAngleCorrectParam::Ptr& GetMotorAngleCorrectParamPtr();

private:
	MotorDatas motorDatas_;

	// �������ڲΣ�ԭʼconfig�����ʽ
	MotorCalibrationParam::Ptr motorParamPtr_;

	// �������ڲΣ��������Ƕ�ֵתΪ��Ԫ��������ʹ�ã�ֵ��motorParamPtr_һ��
	MotorCalibrationParam2::Ptr motorParam2Ptr_;

	// ������360���Ƕ�����ֵ
	MotorAngleCorrectParam::Ptr motorAngleCorrectParamPtr_;

	// ���ڵ���ʱʹ�ã�����motorParam2Ptr_��motorAngleCorrectParamPtr_
	// ԭʼ����ֵ����Ϊ�������������Ż���ᱻ��д�����ﱣ��ԭʼֵ�����Ƿ������
	MotorCalibrationParam2::Ptr motorParam2PtrOri_;
	MotorAngleCorrectParam::Ptr motorAngleCorrectParamPtrOri_;

	bool debugFlag_;
};

}// namespace sc
