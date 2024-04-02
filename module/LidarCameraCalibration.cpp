#include "module/LidarCameraCalibration.h"
#include "internal/Utils.h"
#include "internal/CeresOptimizer.h"
#include "internal/PlyIo.hpp"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace sc {

LidarCameraCalibrationConfig::Ptr LidarCameraCalibration::configPtr_;
std::vector<std::string> LidarCameraCalibration::texts_;
std::vector<cv::Point> LidarCameraCalibration::points_;
int LidarCameraCalibration::countId_ = 0;
std::string LidarCameraCalibration::curImageId_;

LidarCameraCalibration::LidarCameraCalibration(LidarCameraCalibrationConfig::Ptr& config) {
	configPtr_ = config;
}

LidarCameraCalibration::~LidarCameraCalibration() {}

bool LidarCameraCalibration::Run() {
	// 根据参数设置，读入雷达轨迹、相机轨迹时间戳
	if (!InitWithConfig()) {
		return false;
	}

	if (configPtr_->mode == 1) {
		// 模式1为照片选点，为标定提供数据
		if (!MarkImagePoint()) {
			return false;
		}
	}
	else if (configPtr_->mode == 2) {
		// 模式2为雷达-相机选点标定
		// 读入pnp数据
		if (!LoadObservation()) {
			return false;
		}

		// 根据设置，可以选择标定相机外参、时间戳、相机内参
		if (!RunCalibration()) {
			return false;
		}

		// 保存结果
		if (!SaveOptimizeResult()) {
			return false;
		}

		// 赋色验证标定结果
		CheckRecolor();
	}
	else {
		return false;
	}

	return true;
}

bool LidarCameraCalibration::InitWithConfig() {
	lidarTrajPtr_ = std::make_shared<TrajectoryManager>();
	cameraTrajPtr_ = std::make_shared<TrajectoryManager>();

    // 读入camera时间戳
    if (!cameraTrajPtr_->loadTimeList(configPtr_->cameraTimeFilePath)) {
        LOG(INFO) << "Load file error: " << configPtr_->cameraTimeFilePath << std::endl;
        return false;
    }

    // 读入lidar轨迹pose
	if (!lidarTrajPtr_->loadTrajectory(configPtr_->lidarTrajPath)) {
		LOG(INFO) << "Load file error: " << configPtr_->lidarTrajPath << std::endl;
        return false;
    }

	configPtr_->debugFlag = true;

	configPtr_->debugRootDir = configPtr_->lidarCloudPath;
	if (configPtr_->debugFlag) {
		configPtr_->debugRootDir = GetRootDirectory(configPtr_->lidarTrajPath) + "/calibration/";
		if (!CreateDir(configPtr_->debugRootDir)) {
			configPtr_->debugRootDir = configPtr_->lidarCloudPath;
		}
	}

    return true;
}

bool LidarCameraCalibration::MarkImagePoint() {
	int markPointNum = 0;
	for (const int id : configPtr_->calibImgIdVec) {
		countId_ = 0;
		curImageId_ = std::to_string(id);
		texts_.clear();
		points_.clear();

		cv::Mat img = cv::imread(configPtr_->cameraPicPath + curImageId_ + ".jpg");
		cv::namedWindow(curImageId_, CV_WINDOW_NORMAL);
		cv::setMouseCallback(curImageId_, HandlerMouseEvent, &img);
		HandlerMouseEvent(0, 0, 0, 0, &img);

		cv::imshow(curImageId_, img);
		cv::waitKey();
		cvDestroyWindow(curImageId_.c_str());

		for (int i = 0; i < points_.size(); i++) {
			cv::putText(img, texts_[i], points_[i], 0, 2.0, cv::Scalar(0, 0, 255), 3, 8, 0);
			cv::circle(img, points_[i], 4, cv::Scalar(0, 255, 0), 3, 8, 0);
		}
		markPointNum += countId_;
		cv::imwrite(configPtr_->debugRootDir + curImageId_ + "_mark.jpg", img);
	}

	return markPointNum > 3;
}


void LidarCameraCalibration::HandlerMouseEvent(int event, int x, int y, int flags, void* p) {
	cv::Mat img = (*static_cast<cv::Mat*>(p)); //获取当前图像
	cv::Mat img_clone = img.clone();

	static std::string text;
	static cv::Point point;	

	if (event == cv::EVENT_LBUTTONDOWN) {
		point = cv::Point(x, y);
		int blue = img_clone.at<cv::Vec3b>(y, x)[0];
		int green = img_clone.at<cv::Vec3b>(y, x)[1];
		int red = img_clone.at<cv::Vec3b>(y, x)[2];

		//格式:（u, v）（b, g, r）
		std::stringstream ss;
		ss << countId_ << " (" << x << ", " << y << ") " 
					   << " (" << blue << "," << green << "," << red << ")";
		text = ss.str();
		
		for (int i = 0; i < points_.size(); i++) {
			cv::putText(img_clone, texts_[i], points_[i], 0, 2.0, cv::Scalar(0, 0, 255), 3, 8, 0);
			cv::circle(img_clone, points_[i], 4, cv::Scalar(0, 255, 0), 3, 8, 0);
		}

		cv::putText(img_clone, text, point, 0, 2.0, cv::Scalar(0, 0, 255), 3, 8);
		cv::circle(img_clone, point, 4, cv::Scalar(0, 255, 0), 3, 8, 0);

		cv::imshow(curImageId_, img_clone);

	}
	else if (event == cv::EVENT_RBUTTONDOWN) {
		std::string filename = configPtr_->debugRootDir + "pixel_coord.txt";
		std::ofstream ofs(filename, std::ios::app);
		if (!ofs.is_open()) {
			LOG(INFO) << "Open file error: " << configPtr_->debugRootDir + "pixel_coord.txt";
			system("pause");
			return;
		}

		std::stringstream ss;
		ss << countId_ << " " << curImageId_ << " " << point.x << " " << point.y << " ";
		ofs << ss.str() << std::endl;
		texts_.push_back(text);
		points_.push_back(point);
		countId_++;
	}
}

bool LidarCameraCalibration::LoadObservation() {
	std::string calibFilePath = configPtr_->debugRootDir + "pixel_coord.txt";
	std::ifstream ifs(calibFilePath);
	if (!ifs.is_open()) {
		LOG(INFO) << "No calibration file: " << calibFilePath;
		system("pause");
		return false;
	}

	configPtr_->calibTimeVec.clear();
	pnpDatas_.clear();

	std::string line, elem;	
	std::getline(ifs, line);
	std::stringstream ss0(line);

	while (std::getline(ss0, elem, ' ')) {
		configPtr_->calibTimeVec[std::stoi(elem)] = 0;
	}

	const auto cameraPoses = cameraTrajPtr_->getPoints();

	while (std::getline(ifs, line)) {
		std::vector<std::string> elems;
		std::stringstream ss(line);

		while (std::getline(ss, elem, ' ')) {
			if (elem != " " && !elem.empty())
				elems.push_back(elem);
		}

		if (elems.size() != 7) {
			continue;
		}

		PnPData d;
		d.imageId = std::stoi(elems[1]);
		d.uv[0] = std::stoi(elems[2]);
		d.uv[1] = std::stoi(elems[3]);
		d.pt3D[0] = std::stof(elems[4]);
		d.pt3D[1] = std::stof(elems[5]);
		d.pt3D[2] = std::stof(elems[6]);
		d.time = cameraPoses[d.imageId].time;
		d.useFlag = configPtr_->calibTimeVec.find(d.imageId) != configPtr_->calibTimeVec.end();
		pnpDatas_.push_back(d);
	}
	return pnpDatas_.size() > 3;
}

bool LidarCameraCalibration::RunCalibration() {
	// 设置优化参数
	CeresOptimizerConfig::Ptr option = std::make_shared<CeresOptimizerConfig>();
	option->optimizeLidarIntrinsicParam = configPtr_->optExtParam;
	option->optimizeTimeDelayOnlyOne = configPtr_->optTimeDelayOnlyOne;
	option->optimizeTimeDelaySeparate = configPtr_->optTimeDelaySeparate;

	CeresOptimizer optimizer(option);

	optimizer.SetLidarCameraCalibParam(configPtr_);

	optimizer.AddLidarCameraCalibObservation(pnpDatas_, lidarTrajPtr_);

	// 解算
	optimizer.Solve(false);

	return true;
}

bool LidarCameraCalibration::SaveOptimizeResult() {
	std::ofstream ofs(configPtr_->debugRootDir + "calibration_result.txt");
	if (!ofs.is_open()) {
		LOG(INFO) << "Can not open: " << configPtr_->debugRootDir + "calibration_result.txt";
		return false;
	}

	std::stringstream ss;

	ss << std::fixed << std::setprecision(6);
	double pose[6];
	const auto& qOpt = configPtr_->extPoseCam2Imu.q;
	const auto& pOpt = configPtr_->extPoseCam2Imu.p;
	configPtr_->extPoseCam2Imu.GetXYZRPY(pose);

	ss << "\nimu to camera external parameter:\n\n";
	ss << "quaternion(wxyz) : " << qOpt.w() << "\t"
		<< qOpt.x() << "\t" << qOpt.y() << "\t" << qOpt.z() << std::endl;
	ss << "angles(rpy)      : " << pose[3] * RAD2DEG << "\t"
	   << pose[4] * RAD2DEG << "\t" << pose[5] * RAD2DEG << std::endl;
	ss << "position         : " << pose[0] << "\t"
	   << pose[1] << "\t" << pose[2] << std::endl << std::endl;

	ss << "rotation matrix: \n[";
	Eigen::Matrix3d rotMat = qOpt.toRotationMatrix();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ss << rotMat(i, j);
			if (i != 2 || j != 2) ss << ", ";
		}
		if (i != 2) ss << std::endl;
	}
	ss << "]\n\ntime delay: \n";
	for (const auto& t : configPtr_->calibTimeVec) {
		if (configPtr_->optTimeDelayOnlyOne) {
			ss << "id: " << t.first << ", delay: "
				<< configPtr_->calibTimeVec.begin()->second << std::endl;
		}
		else {
			ss << "id: " << t.first << ", delay: "
				<< t.second << std::endl;
		}
	}

	std::cout << std::fixed << std::setprecision(6);
	std::cout << ss.str() << std::endl << std::endl;
	ofs << ss.str() << std::endl;

	// 输出精度统计信息
	if (configPtr_->debugFlag) {
		const auto& cameraPoses = cameraTrajPtr_->getPoints();
		float fx = configPtr_->camIntrinsicMat(0, 0);
		float fy = configPtr_->camIntrinsicMat(1, 1);
		float cx = configPtr_->camIntrinsicMat(0, 2);
		float cy = configPtr_->camIntrinsicMat(1, 2);
		float scale = 1.0;
		int sumError = 0;
		int lastId = -1;

		for (auto& d : pnpDatas_) {			
			if (lastId != d.imageId) {
				std::cout << std::endl;
			}

			lastId = d.imageId;
			double timeCorrect;
			if (configPtr_->optTimeDelayOnlyOne) {
				timeCorrect = configPtr_->calibTimeVec.begin()->second;
			}
			else {
				timeCorrect = configPtr_->calibTimeVec[d.imageId];
			}

			double correctTime = cameraPoses[d.imageId].time + timeCorrect;
			PoseD poseInverse = lidarTrajPtr_->slerp(correctTime).pose.Inverse();

			Eigen::Vector3f ptOpt = poseInverse.q.cast<float>() * d.pt3D + poseInverse.p.cast<float>();

			Eigen::Vector3d ptInImg = qOpt * ptOpt.cast<double>() + pOpt;

			int u = (ptInImg[0] * fx / ptInImg[2] + cx) * scale;
			int v = (ptInImg[1] * fy / ptInImg[2] + cy) * scale;

			double res;
			LidarCameraCalibFactor factor(d, lidarTrajPtr_);
			factor.operator()(qOpt.coeffs().data(), pOpt.data(), 
							  &timeCorrect,
							   configPtr_->camIntrinsicMat.data(), &res);
			res = std::sqrt(res);

			std::cout << "image id: " << d.imageId << std::setw(20) << "   | input uv: "
				<< std::setw(7) << d.uv[0] << std::setw(7) << d.uv[1] << std::setw(3) << "   | optimize uv: "
				<< std::setw(10) << u << std::setw(10) << v << std::setw(10)
				<< "   | error:  " << std::setw(5) << int(res) << std::setw(10)
				<< "   | time delay: " << timeCorrect << std::endl;
			sumError += res;
		}
		std::cout << "\nCalibration error: " << sumError << std::endl;
	}

	return true;
}

void LidarCameraCalibration::CheckRecolor() {
	SetCameraPoses(lidarTrajPtr_);

	BaseCloudPtr fullCloud(new BaseCloud);
	PlyIo::LoadPLYFile(configPtr_->lidarCloudPath, *fullCloud);
	if (fullCloud->empty()) {
		return;
	}

	const auto& cameraPoses = cameraTrajPtr_->getPoints();

	const float surroundRadius = 20;
	for (const auto id : configPtr_->calibTimeVec) {
		LOG(INFO) << "Recolor id: " << id.first;

		BaseCloudPtr originCloud(new BaseCloud);
		ColorCloudPtr colorCloud(new ColorCloud);
		Eigen::Vector3f cameraCenter = cameraPoses[id.first].pose.p.cast<float>();

		for (const BasePoint& pt : fullCloud->points) {
			if ((pt.getVector3fMap() - cameraCenter).norm() < surroundRadius) {
				originCloud->push_back(pt);
			}
		}

		CloudProjection(originCloud, id.first, colorCloud);

		if (configPtr_->debugFlag) {
			PlyIo::SavePLYFileBinary(configPtr_->debugRootDir + std::to_string(id.first) + 
									 "_split_cloud.ply", *originCloud);
			PlyIo::SavePLYFileBinary(configPtr_->debugRootDir + std::to_string(id.first) +
									 "_color_cloud.ply", *colorCloud);
		}
	}
}

void LidarCameraCalibration::SetCameraPoses(const TrajectoryManager::Ptr& globalTraj) {
	auto& cameraPoses = cameraTrajPtr_->getPoints();

	double timeDelayAvg = 0;
	if (configPtr_->optTimeDelayOnlyOne) {
		timeDelayAvg = configPtr_->calibTimeVec.begin()->second;
	}
	else {
		for (const auto& t : configPtr_->calibTimeVec) {
			timeDelayAvg += t.second;
		}
		timeDelayAvg /= configPtr_->calibTimeVec.size();
	}
	LOG(INFO) << "average time delay: " << timeDelayAvg;

	BasePoint pt;
	BaseCloud camTrajCloud;

	PoseD poseC2I = configPtr_->extPoseCam2Imu;
	PoseD poseI2C = poseC2I.Inverse();

	for (auto& camPose : cameraPoses) {
		int curId = camPose.frameId;
		auto poseW2I = globalTraj->slerp(camPose.time + timeDelayAvg);

		// poseW2C = poseW2I * poseI2C;
		camPose.pose = poseW2I.pose * poseI2C;

		camPose.frameId = curId;
		pt.getVector3fMap() = camPose.pose.p.cast<float>();
		pt.intensity = camPose.frameId;
		camTrajCloud.push_back(pt);
	}

	PlyIo::SavePLYFileBinary(configPtr_->debugRootDir + "camera_traj.ply", camTrajCloud);
}


void LidarCameraCalibration::CloudProjection(const BaseCloudPtr& cloudIn, const int imageId,
											 ColorCloudPtr& cloudOut) {
	// 注意：这里是因为有一组数据，照片id和pose id 错位1个
	cv::Mat curImg = cv::imread(configPtr_->cameraPicPath + std::to_string(imageId - 1) + ".jpg");

	const auto& cameraPoses = cameraTrajPtr_->getPoints();
	const auto& poseW2C = cameraPoses[imageId];
	PoseD poseC2W = poseW2C.pose.Inverse();

	float fx = configPtr_->camIntrinsicMat(0, 0);
	float fy = configPtr_->camIntrinsicMat(1, 1);
	float cx = configPtr_->camIntrinsicMat(0, 2);
	float cy = configPtr_->camIntrinsicMat(1, 2);
	float scale = 1.0;

	ColorPoint ptTmp;
	for (const BasePoint& pt : cloudIn->points) {
		Eigen::Vector3d ptVec(pt.x, pt.y, pt.z), ptCam;

		ptCam = poseC2W.q * ptVec + poseC2W.p;
		if (ptCam.z() < 0) {
			continue;
		}

		int u = (ptCam[0] * fx / ptCam[2] + cx) * scale;
		int v = (ptCam[1] * fy / ptCam[2] + cy) * scale;

		if (u < 3840 && u > 0 && v < 2160 && v > 0) {
			ptTmp.getVector3fMap() = pt.getVector3fMap();
			ptTmp.r = curImg.at<cv::Vec3b>(v, u)[2];
			ptTmp.g = curImg.at<cv::Vec3b>(v, u)[1];
			ptTmp.b = curImg.at<cv::Vec3b>(v, u)[0];
			ptTmp.intensity = pt.intensity;
			cloudOut->push_back(ptTmp);
		}
	}
}


template <typename T>
bool LidarCameraCalibrationConfig::GetData(const YAML::Node& node, T& value) {
	if (!node.IsDefined()) {
		return false;
	}
	value = node.as<T>();
	return true;
}

bool LidarCameraCalibrationConfig::LoadConfigFile(const std::string& path) {
	YAML::Node rootNode;

	try {
		rootNode = YAML::LoadFile(path);

		const auto& calibNode = rootNode["calibration_config"];
		GetData(calibNode["mode"], mode);
		GetData(calibNode["calibrate_id"], calibImgIdVec);
		GetData(calibNode["optimize_external_param"], optExtParam);
		GetData(calibNode["optimize_time_delay_separate"], optTimeDelaySeparate);
		GetData(calibNode["optimize_time_delay_only_one"], optTimeDelayOnlyOne);

		const auto& dataNode = rootNode["data_path"];
		GetData(dataNode["laser_cloud_path"], lidarCloudPath);
		GetData(dataNode["laser_trajectory_path"], lidarTrajPath);
		GetData(dataNode["camera_time_file_path"], cameraTimeFilePath);
		GetData(dataNode["camera_data_path"], cameraPicPath);

		lidarCloudPath = UtfToGbk(lidarCloudPath);
		lidarTrajPath = UtfToGbk(lidarTrajPath);
		cameraTimeFilePath = UtfToGbk(cameraTimeFilePath);
		cameraPicPath = UtfToGbk(cameraPicPath);

		const auto& paramNode = rootNode["camera_parameter"];
		std::vector<double> cameraIntrParam, cameraDistCoeffs, cameraExtRot, cameraExtPos;
		GetData(paramNode["camera_intrinsic"], cameraIntrParam);
		GetData(paramNode["camera_dist_coeffs"], cameraDistCoeffs);
		GetData(paramNode["camera_to_imu_R"], cameraExtRot);
		GetData(paramNode["camera_to_imu_t"], cameraExtPos);
		GetData(paramNode["time_delay"], timeDelay);

		extRotCam2Imu = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cameraExtRot.data());
		camIntrinsicMat = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cameraIntrParam.data());

		camDistCoeffs = Eigen::Map<Eigen::Matrix<double, 4, 1>>(cameraDistCoeffs.data());
		extPosCam2Imu = Eigen::Map<Eigen::Matrix<double, 3, 1>>(cameraExtPos.data());

		extPoseCam2Imu.p = extPosCam2Imu;
		extPoseCam2Imu.q = extRotCam2Imu;
	}
	catch (std::exception& exc) {
		LOG(INFO) << "Load calibration file failed: " 
			<< path << ", message: " << exc.what();
		return false;
	}

	return true;
}

}