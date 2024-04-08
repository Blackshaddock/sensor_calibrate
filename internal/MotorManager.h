#pragma once
#include "Bag2File.h"
#include "internal/PoseD.h"
#include "internal/Parameter.hpp"
#include <memory>
#include <vector>
#include <fstream>

// namespace of sensor calibration
namespace sc {

// 编码器标定参数，原始config输入类型
struct MotorCalibrationParam : public ParamterBase {
	typedef std::shared_ptr<MotorCalibrationParam> Ptr;

	double alpha1;			// 编码器 X 轴修正角，度
	double alpha2;			// 编码器 Y 轴修正角，度
	double alpha3;			// 编码器 z轴修正角
	Eigen::Vector3d dP;		// 编码器三轴平移修正量
	double startAngle;		// 开始时刻的角度，这个值不知道哪里会用，原码有

	MotorCalibrationParam() : alpha1(0.), alpha2(0.), alpha3(0.),
							  startAngle(0.) {}
};

// 编码器标定参数，是通过MotorCalibrationParam转换得到，方便标定使用
struct MotorCalibrationParam2 {
	typedef std::shared_ptr<MotorCalibrationParam2> Ptr;

	Eigen::Quaterniond qAlphaX;		// 编码器 X 轴修正角，转四元数
	Eigen::Quaterniond qAlphaY;		// geosun Z轴可观 转换之后
	Eigen::Quaterniond qAlphaZ;	    // 编码器 Z 轴修正角，转四元数 绿土Y轴可观 转换之后
	
	Eigen::Vector3d dP;				// 编码器平移修正量
	
	double startAngle;				// 开始时刻的角度，这个值不知道哪里会用，原码有
};

struct MotorData {
	double time;
	double angle;	// 度

	MotorData() : time(0.), angle(0.) {}

	bool operator<(const MotorData& other) const {
		return this->time < other.time;
	}
};
typedef std::vector<MotorData> MotorDatas;

// 编码器360个角度修正值参数，标定使用
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
	std::vector<double> angles;			// 度
	std::string angleCorrectionFile;	// 参数文件传入

	std::vector<double> lidarhorizons;
	std::string lidarCorrectionFile;    
};

class MotorManager {
public:
	typedef std::shared_ptr<MotorManager> Ptr;

	MotorManager();

	~MotorManager();

	// bag解析编码器数据
	bool ParseMotorData(BagFileReader* bagReader);

	// 导出编码器数据
	bool SaveMotorData(const std::string& path);

	// 保存标定结果，包括编码器内参文件 和 编码器360个角度修正文件
	bool SaveCalibrationResults(const std::string& rootDir);

	// 保存标定结果，包括编码器内参文件 和 编码器360个角度修正文件
	bool SaveCalibrationResults2(const std::string& rootDir);

	// 读入编码器数据
	bool LoadMotorData(const std::string& path);


	//
	void SetMotorParameter(const MotorCalibrationParam::Ptr& intrinsicParam, const MotorAngleCorrectParam::Ptr& angleParam);
	// 设置编码器内参，即标定值
	void SetMotorParameter(const MotorCalibrationParam::Ptr& intrinsicParam, 
						   const MotorAngleCorrectParam::Ptr& angleParam,
						   const double bagStartTime);

	// 根据时间戳，内插编码器角度数据
	bool Slerp(const double time, double& motorAngle);

	// 根据时间戳，内插编码器角度，结合编码器内参，返回pose，
	// 该pose用于转换编码器独立坐标系下数据到编码器全局坐标系下
	bool GetMotorPose(const double time, PoseD& pose, double& angle);

	//根据每个点的角度，获取对应胡pose
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

	// 编码器内参，原始config读入格式
	MotorCalibrationParam::Ptr motorParamPtr_;

	// 编码器内参，将两个角度值转为四元数，方便使用，值和motorParamPtr_一样
	MotorCalibrationParam2::Ptr motorParam2Ptr_;

	// 编码器360个角度修正值
	MotorAngleCorrectParam::Ptr motorAngleCorrectParamPtr_;

	// 仅在调试时使用，保留motorParam2Ptr_和motorAngleCorrectParamPtr_
	// 原始传入值，因为这两个参数在优化后会被改写，这里保留原始值备份是方便调试
	MotorCalibrationParam2::Ptr motorParam2PtrOri_;
	MotorAngleCorrectParam::Ptr motorAngleCorrectParamPtrOri_;

	bool debugFlag_;
};

}// namespace sc
