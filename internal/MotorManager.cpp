#include "internal/MotorManager.h"
#include "internal/Utils.h"
#include <algorithm>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <iomanip>
#include <cmath>
// namespace of sensor calibration
namespace sc {

	MotorManager::MotorManager() : debugFlag_(true) {}

	MotorManager::~MotorManager() {}

	void MotorManager::SetMotorParameter(const MotorCalibrationParam::Ptr& intrinsicParam, const MotorAngleCorrectParam::Ptr& angleParam)
	{
		// ��config����ı������ڲνǶ�ת����
		motorParamPtr_ = intrinsicParam;
		motorParamPtr_->alpha1 *= DEG2RAD;
		motorParamPtr_->alpha2 *= DEG2RAD;
		double alpha3 = 0;
		double cx = cos(motorParamPtr_->alpha1), sx = sin(motorParamPtr_->alpha1);
		double cy = cos(motorParamPtr_->alpha2), sy = sin(motorParamPtr_->alpha2);
		double cz = cos(alpha3), sz = sin(alpha3);
		Eigen::Matrix3d a;
		a<< cy*cz,             cy*sz,             -sy,
									-cx*sz + sx*sy*cz, cx*cz + sx*sy*sz, sx*cy,
									sx*sz + cx*sy*cz,  -sx*cz + cx*sy*sx, cx*cy;
		std::cout << a << std::endl;
		motorParam2Ptr_ = std::make_shared<MotorCalibrationParam2>();
		motorParam2Ptr_->mtrixXY = a;
		// motorParam2Ptr_�ǽ��������ڲνǶ�תΪ��Ԫ�����������ʹ�ã�ֵ��һ����
		
		motorParam2Ptr_->qAlphaX = Eigen::Quaterniond(Eigen::AngleAxisd(intrinsicParam->alpha1, Eigen::Vector3d::UnitX()));
		motorParam2Ptr_->qAlphaY = Eigen::Quaterniond(Eigen::AngleAxisd(intrinsicParam->alpha2, Eigen::Vector3d::UnitY()));
		motorParam2Ptr_->qAlphaZ = Eigen::Quaterniond(Eigen::AngleAxisd(intrinsicParam->alpha3, Eigen::Vector3d::UnitZ()));
		motorParam2Ptr_->dP = intrinsicParam->dP;
		motorAngleCorrectParamPtr_ = angleParam;
		motorAngleCorrectParamPtr_->LoadAngleCorrectionFile();
		motorAngleCorrectParamPtr_->LoadlidarCorrectionFile();
		//motorAngleCorrectParamPtr_.reset(new MotorAngleCorrectParam);
	}

void MotorManager::SetMotorParameter(const MotorCalibrationParam::Ptr& intrinsicParam,
									 const MotorAngleCorrectParam::Ptr& angleParam,
									 const double bagStartTime) {
	static double deg2Rad = M_PI / 180.0;

	// ��config����ı������ڲνǶ�ת����
	motorParamPtr_ = intrinsicParam;
	motorParamPtr_->alpha1 *= deg2Rad;
	motorParamPtr_->alpha2 *= deg2Rad;

	// motorParam2Ptr_�ǽ��������ڲνǶ�תΪ��Ԫ�����������ʹ�ã�ֵ��һ����
	motorParam2Ptr_ = std::make_shared<MotorCalibrationParam2>();
	motorParam2Ptr_->qAlphaX = Eigen::Quaterniond(Eigen::AngleAxisd(-intrinsicParam->alpha1, Eigen::Vector3d::UnitX()));
	motorParam2Ptr_->qAlphaY = Eigen::Quaterniond(Eigen::AngleAxisd(-intrinsicParam->alpha2, Eigen::Vector3d::UnitY()));
	motorParam2Ptr_->qAlphaZ = Eigen::Quaterniond(Eigen::AngleAxisd(intrinsicParam->alpha2, Eigen::Vector3d::UnitZ()));
	motorParam2Ptr_->dP = intrinsicParam->dP;
	motorParam2Ptr_->startAngle = intrinsicParam->startAngle;

	// ���ݲ����ļ�ָ���ĽǶ������ļ�������360���Ƕ�����ֵ
	motorAngleCorrectParamPtr_ = angleParam;
	motorAngleCorrectParamPtr_->LoadAngleCorrectionFile();

	// �ͼ���������ʱ�������ͬ��
	for (MotorData& d : motorDatas_) {
		d.time -= bagStartTime;
	}

	// ���Ϊ����ģʽ�����������ڲκ�360���Ƕ�����ֵ����һ�ݱ���
	// ������ԣ������Ż�ʱ���޸�ԭֵ
	if (motorParamPtr_->debugFlag) {
		motorParam2PtrOri_ = std::make_shared<MotorCalibrationParam2>(*motorParam2Ptr_);
		motorAngleCorrectParamPtrOri_ = std::make_shared<MotorAngleCorrectParam>(*motorAngleCorrectParamPtr_);
	}
}

bool MotorManager::ParseMotorData(BagFileReader* bagReader) {
	if (bagReader == nullptr) {
		return false;
	}

	// ��������ʼ��Ϣ
	BagMotorCfg motorConfig;
	if (bagReader->GetMotorConfig(motorConfig) && debugFlag_) {
		//std::cout << "Motor start angle: " << motorConfig.motorvalue
		//		  << "\nhorizon angle: " << motorConfig.startvalue
		//		  << "\nvertical angle: " << motorConfig.angledeg
		//		  << "\nstart gps time: " << motorConfig.gpstime
		//		  << "\nstart ros time: " << motorConfig.rostime << std::endl;
	}

	// ����ת�����������ݲ�ת����ʽ�������������
	BagMotorData bagMotorData;
	if (bagReader->GetMotorData(bagMotorData)) {
		motorDatas_.reserve(bagMotorData.num);
		for (int i = 0; i < bagMotorData.num; ++i) {
			if (bagMotorData.motorvalue[i] == 0)
				continue;
			
			MotorData data;
			data.time = bagMotorData.motortime[i];
			data.angle = bagMotorData.motorvalue[i];
			motorDatas_.emplace_back(data);
		}
	}

	return !motorDatas_.empty();
}

bool MotorManager::Slerp(const double time, double& motorAngle) {
	MotorData motorTmp;
	motorTmp.time = time;

	auto upperIter = std::upper_bound(motorDatas_.begin(), motorDatas_.end(), motorTmp);
	auto lowerIter = upperIter - 1;
	//int upperId = upperIter - motorDatas_.begin();	// ����

	if (upperIter == motorDatas_.end()) {
		motorAngle = motorDatas_.back().angle;
		return false;
	}
	else if (upperIter == motorDatas_.begin()) {
		motorAngle = motorDatas_.front().angle;
		return false;
	}

	double ratio = (time - lowerIter->time) / (upperIter->time - lowerIter->time);
	ratio = std::min(ratio, 1.);
	ratio = std::max(ratio, 0.);

	if (std::abs(upperIter->angle - lowerIter->angle) > 180.0) {
		if (upperIter->angle > lowerIter->angle) {
			lowerIter->angle += 360.0;
		}
		else {
			upperIter->angle += 360.0;
		}
	}

	motorAngle = (1 - ratio) * lowerIter->angle + ratio * upperIter->angle;

	return true;
}

// Pl����������������ϵ�µ�����
// Pg��������ȫ������ϵ�µ�����
// Qx���������ڲ�alpha1��ӦX�������Ƕ�ת��Ԫ��
// Qz���������ڲ�alpha2��ӦZ�������Ƕ�ת��Ԫ��
// Dp(dx, dy, dz)���������ڲΣ��ֱ��Ӧ����������ϵ��XYZ��ƽ��������
// Qy���������۲�ֵ����������Y��ת���Ƕ�ת��Ԫ��

// ����ԭ����÷�
// Pg = Qy * (Qz * Qx * Pl + Dp)
// Pg = Qy * Qz * Qx * Pl + Qy * Dp

// ���Ǳ��㷨���÷�
// Dq' = Qy * Qz * Qx
// Dp' = Qy * Dp
// Pg = Dq' * Pl + Dp'

// Dq' �� Dp' ��ͬ���ɱ���������pose���������ʹ�ã����Ҳ�Ǹú����ķ���ֵ
bool MotorManager::GetMotorPose(const double time, PoseD& pose, double& angle) {
	// ��ǰʱ�̱������۲�Ƕȣ���Y��ĽǶ�ֵ
	if (!Slerp(time, angle)) {
		return false;
	}

	// �����ԭ���д������֪����ʲô����
	angle = angle + (360.0 - motorParam2Ptr_->startAngle);
	angle *= -1.0;

	static double deg2Rad = M_PI / 180.0;

	// ���ݵ�ǰ�������Ƕȣ��ڲ�360���Ƕ���������Ӧֵ����Ϊ�������Ƕ�������
	int frontId, backId;
	double ratio;
	double angleCorrect = motorAngleCorrectParamPtr_->Slerp(angle, frontId, backId, ratio);
	double angleUse = angle - angleCorrect;
	
	angle *= deg2Rad;
	angleUse *= deg2Rad;
	Eigen::Quaterniond qY(Eigen::AngleAxisd(angleUse, Eigen::Vector3d::UnitY()));
	pose.q = qY * motorParam2Ptr_->qAlphaZ * motorParam2Ptr_->qAlphaX;
	pose.p = qY * motorParam2Ptr_->dP;
	return true;
}
// ���Ǳ��㷨���÷�
// Dq' = Qy * Qz * Qx
// Dp' = Qy * Dp
// Pg = Dq' * Pl + Dp'
bool MotorManager::GetMotorPose4geosun(double angle,PoseD& pose, bool UseMotorAngleCorr)
{
	double angleUse;
	if (UseMotorAngleCorr)
	{
		int frontId, backId;
		double ratio;
		double angleCorrect = motorAngleCorrectParamPtr_->Slerp(angle, frontId, backId, ratio);
		angleUse = angle - angleCorrect;
	}
	else
	{
		angleUse = angle;
	}
	angleUse = -angleUse * DEG2RAD;
	double cz = std::cos(angleUse), sz = std::sin(angleUse);
	Eigen::Quaterniond qZ(Eigen::AngleAxisd(-angleUse, Eigen::Vector3d::UnitZ()));
	Eigen::Matrix3d b;
	b << cz, sz, 0,
		 -sz, cz, 0,
		  0, 0, 1;
	//pose.q = motorParam2Ptr_->qAlphaX* motorParam2Ptr_->qAlphaY*qZ;
	pose.q = b* motorParam2Ptr_->mtrixXY ;
	pose.p = b * motorParam2Ptr_->dP;
	return true;
}

bool MotorManager::GetMotorPoseInfo4geosun(double angle, PoseD& pose, int& frontId, int& backId, double& ratio, bool UseMotorAngleCorr)
{
	double angleUse;
	if (UseMotorAngleCorr)
	{
		double angleCorrect = motorAngleCorrectParamPtr_->Slerp(angle, frontId, backId, ratio);
		
		angleUse = angle - angleCorrect;
	}
	else
	{
		angleUse = angle;
	}
	angleUse = -angleUse * DEG2RAD;
	double cz = std::cos(angleUse), sz = std::sin(angleUse);
	Eigen::Quaterniond qZ(Eigen::AngleAxisd(angleUse, Eigen::Vector3d::UnitZ()));
	Eigen::Matrix3d b;
	b << cz, sz, 0,
		-sz, cz, 0,
		0, 0, 1;
	
	pose.q = b * motorParam2Ptr_->mtrixXY;
	pose.p = b * motorParam2Ptr_->dP;
	return true;
}


bool MotorManager::GetMotorPoseInfo(const double time, PoseD& pose, double& angle, 
									int& frontId, int& backId, double& ratio) {
	// ��ǰʱ�̱������۲�Ƕȣ���Y��ĽǶ�ֵ
	if (!Slerp(time, angle)) {
		return false;
	}

	// �����ԭ���д������֪����ʲô����
	angle = angle + (360.0 - motorParam2Ptr_->startAngle);
	angle *= -1.0;

	static double deg2Rad = M_PI / 180.0;

	// ���ݵ�ǰ�������Ƕȣ��ڲ�360���Ƕ���������Ӧֵ����Ϊ�������Ƕ�������
	double angleCorrect = motorAngleCorrectParamPtr_->Slerp(angle, frontId, backId, ratio);
	double angleUse = angle - angleCorrect;

	angle *= deg2Rad;
	angleUse *= deg2Rad;
	Eigen::Quaterniond qY(Eigen::AngleAxisd(angleUse, Eigen::Vector3d::UnitY()));
	pose.q = qY * motorParam2Ptr_->qAlphaZ * motorParam2Ptr_->qAlphaX;
	pose.p = qY * motorParam2Ptr_->dP;
	return true;
}

bool MotorManager::GetMotorPoseInfoOrigin(const double time, PoseD& pose) {
	// ��ǰʱ�̱������۲�Ƕȣ���Y��ĽǶ�ֵ
	double angle;
	if (!Slerp(time, angle)) {
		return false;
	}

	// �����ԭ���д������֪����ʲô����
	angle = angle + (360.0 - motorParam2PtrOri_->startAngle);
	angle *= -1.0;

	static double deg2Rad = M_PI / 180.0;

	// ���ݵ�ǰ�������Ƕȣ��ڲ�360���Ƕ���������Ӧֵ����Ϊ�������Ƕ�������
	int frontId, backId;
	double ratio;
	double angleCorrect = motorAngleCorrectParamPtrOri_->Slerp(angle, frontId, backId, ratio);
	double angleUse = angle - angleCorrect;

	angle *= deg2Rad;
	angleUse *= deg2Rad;
	Eigen::Quaterniond qY(Eigen::AngleAxisd(angleUse, Eigen::Vector3d::UnitY()));
	pose.q = qY * motorParam2PtrOri_->qAlphaZ * motorParam2PtrOri_->qAlphaX;
	pose.p = qY * motorParam2PtrOri_->dP;
	return true;
}

bool MotorManager::SaveMotorData(const std::string& path) {
	std::ofstream ofs(path);
	if (!ofs.is_open()) {
		LOG(INFO) << "Can not open: " << path;
		return false;
	}

	if (motorDatas_.empty()) {
		LOG(INFO) << "No motor datas.";
		return false;
	}

	ofs << "time, angle" << std::endl;
	ofs << std::fixed << std::setprecision(6);

	for (const MotorData& d : motorDatas_) {
		ofs << d.time << ", " << d.angle << std::endl;
	}
	ofs.close();
	return true;
}

bool MotorManager::LoadMotorData(const std::string& path) {
	std::ifstream ifs(path);
	if (!ifs.is_open()) {
		LOG(INFO) << "Can not open: " << path;
		return false;
	}

	std::string line, elem;
	std::getline(ifs, line);

	motorDatas_.clear();
	while (std::getline(ifs, line)) {
		std::stringstream ss(line);
		std::vector<std::string> elems;

		while (std::getline(ss, elem, ',')) {
			elems.push_back(elem);
		}

		if (elems.size() != 2) {
			continue;
		}

		MotorData data;
		data.time = std::stod(elems[0]);
		data.angle = std::stod(elems[1]);
		motorDatas_.emplace_back(std::move(data));
	}
	return !motorDatas_.empty();
}

bool MotorManager::SaveCalibrationResults(const std::string& rootDir) {
	std::string motorIntriPath = rootDir + "motor_intrinsic_param.txt";
	std::string motorAnglePath = rootDir + "motor_angle_correction_param.txt";

	std::ofstream ifs1(motorIntriPath), ifs2(motorAnglePath);
	if (!ifs1.is_open() || !ifs2.is_open()) {
		LOG(INFO) << "Save calibration result failed: " << rootDir;
		return false;
	}

	double alphaX = std::asin(-motorParam2Ptr_->qAlphaX.toRotationMatrix()(1, 2));
	double alphaZ = std::asin(-motorParam2Ptr_->qAlphaZ.toRotationMatrix()(0, 1));

	Eigen::Quaterniond qX(Eigen::AngleAxisd(alphaX, Eigen::Vector3d::UnitX()));
	Eigen::Quaterniond qZ(Eigen::AngleAxisd(alphaZ, Eigen::Vector3d::UnitZ()));
	std::cout << "originX\n" << motorParam2Ptr_->qAlphaX.toRotationMatrix() << std::endl;
	std::cout << "originZ\n" << motorParam2Ptr_->qAlphaZ.toRotationMatrix() << std::endl;

	std::cout << "optimizeX\n" << qX.toRotationMatrix() << std::endl;
	std::cout << "optimizeZ\n" << qZ.toRotationMatrix() << std::endl;

	ifs1 << std::fixed << std::setprecision(6);
	ifs2 << std::fixed << std::setprecision(6);
	ifs1 << "alpha1: " << alphaX * RAD2DEG << std::endl;
	ifs1 << "alpha1: " << alphaX * RAD2DEG << std::endl;
	ifs1 << "dp: " << motorParam2Ptr_->dP[0] << " " << motorParam2Ptr_->dP[1]
				   << " " << motorParam2Ptr_->dP[2] << std::endl;

	ifs2 << "id, angle" << std::endl;
	double* angles = motorAngleCorrectParamPtr_->GetAngles();
	for (int i = 0; i < 360; i++) {
		ifs2 << i << ", " << angles[i] << std::endl;
	}

	ifs1.close(), ifs2.close();
	return true;
}


bool MotorManager::SaveCalibrationResults2(const std::string& rootDir) {
	std::string motorIntriPath = rootDir + "motor_intrinsic_param.txt";
	std::string motorAnglePath = rootDir + "motor_angle_correction_param.txt";
	std::string lidarHorizonPath = rootDir + "lidar_intrisic_param.txt";
	std::ofstream ifs1(motorIntriPath), ifs2(motorAnglePath), ifs3(lidarHorizonPath);
	if (!ifs1.is_open() || !ifs2.is_open() || !ifs3.is_open()) {
		LOG(INFO) << "Save calibration result failed: " << rootDir;
		return false;
	}

	// �궨���ڲ���motorParamPtr_�е������ǶȺ�3��ƽ������
	// Ҫ�ѱ궨���ͬ�����µ�motorParam2Ptr_��������Ԫ��������ƽ����
	motorParam2Ptr_->qAlphaX = Eigen::Quaterniond(Eigen::AngleAxisd(motorParamPtr_->alpha1, Eigen::Vector3d::UnitX()));
	motorParam2Ptr_->qAlphaY = Eigen::Quaterniond(Eigen::AngleAxisd(motorParamPtr_->alpha2, Eigen::Vector3d::UnitY()));
	motorParam2Ptr_->qAlphaZ = Eigen::Quaterniond(Eigen::AngleAxisd(motorParamPtr_->alpha3, Eigen::Vector3d::UnitZ()));
	motorParam2Ptr_->dP = motorParamPtr_->dP;
	ifs1 << std::fixed << std::setprecision(6);
	ifs2 << std::fixed << std::setprecision(6);
	ifs1 << "alpha1: " << motorParamPtr_->alpha1 * RAD2DEG << std::endl;
	ifs1 << "alpha2: " << motorParamPtr_->alpha2 * RAD2DEG << std::endl;
	ifs1 << "alpha3: " << motorParamPtr_->alpha3 * RAD2DEG << std::endl;
	ifs1 << "dp: [" << motorParamPtr_->dP[0] << ", " << motorParamPtr_->dP[1]
		<< ", " << motorParamPtr_->dP[2] << "]" << std::endl;

	//ifs2 << "id, angle" << std::endl;
	double* angles = motorAngleCorrectParamPtr_->GetAngles();
	for (int i = 0; i < 360; i++) {
		ifs2 << angles[i] << std::endl;
		//ifs2 << i << ", " << angles[i] << std::endl;
	}
	double* lidarhorizons = motorAngleCorrectParamPtr_->GetHorizons();
	for (int i = 0; i < 16; i++)
	{
		ifs3 << lidarhorizons[i] << std::endl;
	}
	ifs1.close(), ifs2.close(), ifs3.close();
	return true;
}


MotorDatas& MotorManager::GetMotorDatas() { return motorDatas_; }

MotorCalibrationParam::Ptr& MotorManager::GetMotorIntrinsicParamPtr() {
	return motorParamPtr_; 
}

MotorCalibrationParam2::Ptr& MotorManager::GetMotorIntrinsicParam2Ptr() {
	return motorParam2Ptr_;
}

MotorAngleCorrectParam::Ptr& MotorManager::GetMotorAngleCorrectParamPtr() {
	return motorAngleCorrectParamPtr_; 
}
}// namespace sc
