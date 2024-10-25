#include "color_point.h"




std::vector<std::string> ColoredPoint::m_vText;
std::vector<cv::Point> ColoredPoint::m_vPoint;
int ColoredPoint::m_iCount = 0;
string ColoredPoint::m_sCurImgId;
string ColoredPoint::m_sLogPath;

ColoredPoint::ColoredPoint()
{
	m_pConf = std::make_shared<slamColorPointOptionsCfg>();
	m_pPoints.reset(new BaseCloud);
	m_surMap.reset(new colorPointCloudT);
	m_pNdtOmp = pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
	//m_pNdtOmp.reset(new pclomp::NormalDistributionsTransform<BasePoint, BasePoint>);
	NdtInit(m_pConf->s_dNdtResolution);
	initColorList();
	
	
}

void ColoredPoint::LoadPoints()
{
	if (m_pConf->s_sLidarFilePath.empty())
	{
		LOG(INFO) << "The input lidar path is empty!" << std::endl;
		system("pause");
		return;
	}
	pcl::PointCloud<pcl::PointXYZI> tmp;
	pcl::io::loadPLYFile(m_pConf->s_sLidarFilePath, tmp);
	
	//pcl::copyPointCloud(*m_pPoints, tmp);
	//m_pPoints->clear();
	m_pNdtOmp->setInputTarget(tmp.makeShared());
	Eigen::Vector3i counter(0, 0, 0);
	/*auto a = m_pNdtOmp->getTargetCells();
	a.getLeaves();*/
	std::cout << m_pNdtOmp->getTargetCells().getLeaves().size() << std::endl;

	for (const auto &v : m_pNdtOmp->getTargetCells().getLeaves())
	{
		//std::cout << v.first << " " << v.second.nr_points <<" " << tmp.size() <<   std::endl;
		auto leaf = v.second;
		if (leaf.nr_points < 10)
		{
			continue;
		}
		//论文中判断面 和 柱的公式
		int plane_type = checkPlaneType(leaf.getEvals(), leaf.getEvecs(), m_pConf->s_dPlaneThreshold);
		if (plane_type < 0)
			continue;
		Eigen::Vector4d surfCoeff;
		BaseCloudPtr cloud_inliers(new BaseCloud());
		
		if (!fitPlane(leaf.pointList_.makeShared(), surfCoeff, cloud_inliers))
			continue;

		counter(plane_type) += 1;
		SurfelPlane surfplane;
		pcl::copyPointCloud(leaf.pointList_, surfplane.cloud);
		//surfplane.cloud = leaf.pointList_;
		surfplane.cloud_inlier = *cloud_inliers;
		surfplane.p4 = surfCoeff;
		surfplane.Pi = -surfCoeff(3) * surfCoeff.head<3>();
		BasePoint min, max;
		pcl::getMinMax3D(surfplane.cloud, min, max);
		surfplane.boxMin = Eigen::Vector3d(min.x, min.y, min.z);
		surfplane.boxMax = Eigen::Vector3d(max.x, max.y, max.z);

		m_surPlanes.push_back(surfplane);
	}
	m_surMap->clear();
	{
		int idx = 0;
		for (const auto& v : m_surPlanes) {
			colorPointCloudT cloud_rgb;
			pcl::copyPointCloud(v.cloud_inlier, cloud_rgb);

			size_t colorType = (idx++) % m_colorList.size();
			for (auto& p : cloud_rgb) {
				p.rgba = m_colorList[colorType];
			}
			*m_surMap += cloud_rgb;
		}
	}

	PlyIo::SavePLYFileBinary(m_sLogPath + "sourceColor.ply", *m_surMap);

	

}


int ColoredPoint::checkPlaneType(const Eigen::Vector3d& eigen_value,
	const Eigen::Matrix3d& eigen_vector,
	const double& p_lambda) {
	Eigen::Vector3d sorted_vec;
	Eigen::Vector3i ind;
	sort_vec(eigen_value, sorted_vec, ind);

	double p = 2 * (sorted_vec[1] - sorted_vec[2]) /
		(sorted_vec[2] + sorted_vec[1] + sorted_vec[0]);

	if (p < p_lambda) {
		return -1;
	}

	int min_idx = ind[2];
	Eigen::Vector3d plane_normal = eigen_vector.block<3, 1>(0, min_idx);
	plane_normal = plane_normal.array().abs();

	sort_vec(plane_normal, sorted_vec, ind);
	return ind[2];
}


bool ColoredPoint::fitPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	Eigen::Vector4d& coeffs,
	BaseCloudPtr cloud_inliers) {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZI> seg;    /// Create the segmentation object
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.05);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() < 20) {
		return false;
	}

	for (int i = 0; i < 4; i++) {
		coeffs(i) = coefficients->values[i];
	}

	pcl::copyPointCloud<pcl::PointXYZI>(*cloud, *inliers, *cloud_inliers);
	return true;
}

void ColoredPoint::initColorList()
{
	m_colorList.set_capacity(6);
	m_colorList.push_back(0xFF0000);
	m_colorList.push_back(0xFF00FF);
	m_colorList.push_back(0x436EEE);
	m_colorList.push_back(0xBF3EFF);
	m_colorList.push_back(0xB4EEB4);
	m_colorList.push_back(0xFFE7BA);
}


void ColoredPoint::LoadImages()
{
	if (m_pConf->s_sImagePath.empty())
	{
		LOG(INFO) << "The input image path is empty!" << std::endl;
		system("pause");
		return;
	}
	if (IsDir(m_pConf->s_sImagePath))
	{
		std::vector<string> filenames;
		std::vector<string> resultstr;
		ImageFrame temp_imgframe;
		LOG(INFO) << "Read Img Start!!!" << std::endl;
//#pragma omp parallel for 
		for (int i = 0; i < m_pConf->s_iCamNum; i++)
		{
			string imgpath = m_pConf->s_sImagePath + "\\C" + std::to_string(i);
			if (!IsDir(imgpath))
			{
				LOG(INFO) << "Can not find the path: " << imgpath << std::endl;
			}
			filenames = GetFileNamesFromDir(imgpath);
	//#pragma omp parallel for 
			for (auto filename : filenames)
			{
				temp_imgframe.image = cv::imread(filename);
				resultstr = split(GetFileName(filename), '-');
				temp_imgframe.imgtype = stoi(resultstr[0]);
				temp_imgframe.time = stoi(resultstr[2]) + stoi(resultstr[3]) * 0.001;
				temp_imgframe.frameId = m_vImgFrame.size();
				m_vImgFrame.emplace_back(temp_imgframe);
			}
		}
		LOG(INFO) << "Read Img Finish!!!" << std::endl;
		
	}
}

void ColoredPoint::LoadTraj()
{
	if (m_pConf->s_sPosFilePath.empty())
	{
		LOG(INFO) << "Can not find pos path!" << std::endl;
		system("pause");
		return;
	}
	std::string tmp;
	PoseD temp_pos;
	ImuFrame temp_imu;
	std::fstream fd(m_pConf->s_sPosFilePath);
	for (int i = 0; /*i < 2007500*/; i++)
	{
		std::getline(fd, tmp);
		if (tmp == "")
		{
			break;
		}
		sscanf(tmp.c_str(), "%lf %lf %f %f %f %f %f %f", &temp_pos.time, &temp_imu.time, &temp_imu.acc[0], &temp_imu.acc[1], &temp_imu.acc[2], &temp_imu.gyr[0], &temp_imu.gyr[1], &temp_imu.gyr[2]);
		temp_pos.q.w() = temp_imu.time;
		temp_pos.q.x() = temp_imu.acc[0];
		temp_pos.q.y() = temp_imu.acc[1];
		temp_pos.q.z() = temp_imu.acc[2];
		temp_pos.p.x() = temp_imu.gyr[0];
		temp_pos.p.y() = temp_imu.gyr[1];
		temp_pos.p.z() = temp_imu.gyr[2];
		m_vPosFrame.push_back(temp_pos);
	}
	if (m_vImgFrame.empty())
	{
		LOG(INFO) << "Can not find image!" << std::endl;
		return;
	}
	int idx = 0;
	for (auto &imagepose : m_vImgFrame)
	{
		imagepose.pose.time = imagepose.time;
		if (imagepose.pose.time < m_vPosFrame.begin()->time)
		{
			imagepose.pose.time = 0;
			imagepose.pose = m_vPosFrame.front();
			continue;
		}
		const auto& upperIter = std::upper_bound(m_vPosFrame.begin(), m_vPosFrame.end(), imagepose.pose);
		const auto& lowerIter = upperIter - 1;
		if (upperIter == m_vPosFrame.end()) {
			imagepose.pose.time = 1;
			continue;
		}
		if (upperIter == m_vPosFrame.begin()) {
			imagepose.pose = m_vPosFrame.front();
			continue;
		}
		double ratio = (imagepose.pose.time - lowerIter->time) / (upperIter->time - lowerIter->time);
		ratio = std::min(ratio, 1.0);
		ratio = std::max(ratio, 0.0);
		imagepose.pose = lowerIter->Slerp(ratio, *upperIter);
	}
}

void ColoredPoint::LoadConf(const std::string str_in)
{
	if (!IsExists(str_in))
	{
		LOG(INFO) << "The input config file is not exist!" << std::endl;
		system("pause");
		return;
	}
	m_pConf = std::make_shared<slamColorPointOptionsCfg>();
	YAML::Node rootNode;

	try
	{
		rootNode = YAML::LoadFile(str_in);
	}
	catch (std::exception& exc)
	{
		LOG(INFO) << "Load config file failed: "
			<< exc.what() << ", path: " << str_in;
		return ;
	}
	auto ColorNode = rootNode["color_params"];
	GetData(ColorNode["image_path"], m_pConf->s_sImagePath);
	GetData(ColorNode["use_time"], m_pConf->s_bUseTime);
	GetData(ColorNode["lidar_file_path"], m_pConf->s_sLidarFilePath);
	GetData(ColorNode["pos_file_path"], m_pConf->s_sPosFilePath);
	GetData(ColorNode["surround_radius"], m_pConf->s_dRadius);



	std::vector<double> dp(3);
	ColorNode = rootNode["cam_params"];
	GetData(ColorNode["Rcam2lid"], dp);

	m_pConf->s_eRCam2Lid  =  GetRFromZYX(dp[0], dp[1], dp[2]);
	dp.clear();
	GetData(ColorNode["Tcam2lid"], dp);
	m_pConf->s_eTCam2Lid << dp[0], dp[1], dp[2];
	dp.clear();
	GetData(ColorNode["Rcam2lid0"], dp);
	m_pConf->s_eRCam2Lid0 = GetRFromZYX(dp[0], dp[1], dp[2]);
		
	dp.clear();
	GetData(ColorNode["Tcam2lid0"], dp);
	m_pConf->s_eTCam2Lid0 << dp[0], dp[1], dp[2];

	dp.clear();
	GetData(ColorNode["Rcam2lid2"], dp);
	m_pConf->s_eRCam2Lid2 = GetRFromZYX(dp[0], dp[1], dp[2]);
		
	dp.clear();
	GetData(ColorNode["Tcam2lid2"], dp);
	m_pConf->s_eTCam2Lid2 << dp[0], dp[1], dp[2];
	dp.clear();


	GetData(ColorNode["camIntrisic"], dp);
	m_pConf->s_eCamIntrisic << dp[0], 0, dp[2],
		                       0, dp[1], dp[3],
		                       0, 0, 0;

	dp.clear();
	GetData(ColorNode["camIntrisic0"], dp);
	m_pConf->s_eCamIntrisic0 << dp[0], 0, dp[2],
		0, dp[1], dp[3],
		0, 0, 0;

	dp.clear();
	GetData(ColorNode["camIntrisic2"], dp);
	m_pConf->s_eCamIntrisic2 << dp[0], 0, dp[2],
		0, dp[1], dp[3],
		0, 0, 0;
	
	dp.clear();
	GetData(ColorNode["camNum"], m_pConf->s_iCamNum);
	m_sLogPath = GetRootDirectory(m_pConf->s_sLidarFilePath) + "/Process/";
	if (!CreateDir(m_sLogPath)) {
		m_sLogPath = GetRootDirectory(m_pConf->s_sLidarFilePath);
	}
}

void ColoredPoint::Run()
{
	//1. 获取点云数据/图片数据
	LoadPoints();
	//VtkDisplay();
	LoadImages();
	//2. 获取POS信息，并插值获取对应相机的POS
	LoadTraj();
	//3. 按照距离对点云进行附色
	PointsColorful();


	//MarkImagePoint();


}

void ColoredPoint::PointsColorful()
{
	if (m_pPoints->empty())
	{
		LOG(INFO) << "The input points is empty!!!" << std::endl;
	}
	BaseCloudPtr originCloud(new BaseCloud);
	ColorCloudPtr colorCloud(new ColorCloud);
	Eigen::Vector3f cameraCenter = m_vImgFrame[83].pose.p.cast<float>();
	for (auto& pt : m_pPoints->points)
	{
		if ((pt.getVector3fMap() - cameraCenter).norm() < m_pConf->s_dRadius) {
			
			originCloud->push_back(pt);
		}
	}
	PlyIo::SavePLYFileBinary(m_sLogPath + "source_cloud_extract.ply", *originCloud);
 	CloudProjection(originCloud, m_vImgFrame[83], colorCloud);
	PlyIo::SavePLYFileBinary(m_sLogPath + "source_color_extract.ply", *colorCloud);
}

void ColoredPoint::CloudProjection(BaseCloudPtr cloudIn, ImageFrame imgFrame, ColorCloudPtr cloudout)
{
	
	PoseD poseC2W = imgFrame.pose.Inverse();

	float fx = m_pConf->s_eCamIntrisic(0, 0);
	float fy = m_pConf->s_eCamIntrisic(1, 1);
	float cx = m_pConf->s_eCamIntrisic(0, 2);
	float cy = m_pConf->s_eCamIntrisic(1, 2);
	float x0 = imgFrame.image.cols/2;
	float y0 = imgFrame.image.rows/2;
	float scale = 1.0;
	/*Eigen::Matrix3d tmp_r;
	tmp_r << -1, 0, 0, 0, -1, 0, 0, 0, -1;*/
	ColorPoint ptTmp;
	int num = 0;
	std::ofstream testout;
	testout.open(m_sLogPath + "compare.txt");
	for (const BasePoint& pt : cloudIn->points) {
		Eigen::Vector3d ptVec(pt.x, pt.y, pt.z), ptCam;

		ptCam = poseC2W.q * ptVec + poseC2W.p;
		
		
		/*ptCam = tmp_r * ptCam;
		ptCam = m_pConf->s_eRCam2Lid0.inverse() * ptCam - m_pConf->s_eRCam2Lid0.inverse() * m_pConf->s_eTCam2Lid0;
		ptTmp.x = ptCam[0];
		ptTmp.y = ptCam[1];
		ptTmp.z = ptCam[2];*/
		
		//cloudout->push_back(ptTmp);
		ptCam = m_pConf->s_eRCam2Lid * ptCam + m_pConf->s_eTCam2Lid;
		//ptCam = m_pConf->s_eRCam2Lid.inverse() * ptCam - m_pConf->s_eRCam2Lid.inverse() * m_pConf->s_eTCam2Lid;
		if (ptCam.z() < 0) {
			continue;
		}
		ptTmp.x = ptCam[0];
		ptTmp.y = ptCam[1];
		ptTmp.z = ptCam[2]; 
		//cloudout->push_back(ptTmp);
		int u = (ptCam[0] * fx / ptCam[2] + cx) * scale+ x0;
		int v = (ptCam[1] * fy / ptCam[2] + cy) * scale+ y0;
		

		if (u < 2592 && u > 0 && v < 1944 && v > 0) {
			//ptTmp.getVector3fMap() = pt.getVector3fMap();
			ptTmp.r = imgFrame.image.at<cv::Vec3b>(v, u)[2];
			ptTmp.g = imgFrame.image.at<cv::Vec3b>(v, u)[1];
			ptTmp.b = imgFrame.image.at<cv::Vec3b>(v, u)[0];
			ptTmp.intensity = pt.intensity;
			ptTmp.id = num;
			ptTmp.u = u;
			ptTmp.v = v;

			num++;
			Color_LOC position((int)u, (int)v);
			auto iter = m_uColor.find(position);
			//int distance = std::distance(m_uColor.begin(), m_uColor.end());
			if (iter == m_uColor.end()) {
				m_uColor[position] = ptTmp;
				ptTmp.intensity = 0;

			}
			else {
				if (m_uColor[position].z < ptTmp.z )
				{
					if (abs(m_uColor[position].z - ptTmp.z) < 0.10)
					{
					}
					else {
						ptTmp.intensity = 255;
						ptTmp.r = 255;
						ptTmp.g = 255;
						ptTmp.b = 255;
					}
					
				}
				else {
					ColorPoint tmpColorpt;
					tmpColorpt = m_uColor[position];
					tmpColorpt.id = ptTmp.id;
					ptTmp.id = m_uColor[position].id;
					cloudout->points[m_uColor[position].id] = ptTmp;
					ptTmp = tmpColorpt;
				}
			}
			cloudout->push_back(ptTmp);
			ptTmp.u = ptTmp.u * 10000 + ptTmp.v;
			testout << ptTmp.x << " " << ptTmp.y << " " << ptTmp.z << " " << ptTmp.intensity << " " << ptTmp.u  << std::endl;
		}

	}
	testout.close();
}




bool ColoredPoint::MarkImagePoint() {
	int markPointNum = 0;
	m_iCount = 0;
	for (auto imgframe : m_vImgFrame) {
		
		m_vText.clear();
		m_vPoint.clear();
		m_sCurImgId = std::to_string(imgframe.imgtype) + "_" + std::to_string(imgframe.frameId);
		
		cv::namedWindow(m_sCurImgId, CV_WINDOW_NORMAL);
		cv::setMouseCallback(m_sCurImgId, MouseEvent, &imgframe.image);
		MouseEvent(0, 0, 0, 0, &imgframe.image);

		cv::imshow(m_sCurImgId, imgframe.image);
		cv::waitKey();
		cvDestroyWindow(m_sCurImgId.c_str());

		for (int i = 0; i < m_vPoint.size(); i++) {
			cv::putText(imgframe.image, m_vText[i], m_vPoint[i], 0, 2.0, cv::Scalar(0, 0, 255), 3, 8, 0);
			cv::circle(imgframe.image, m_vPoint[i], 4, cv::Scalar(0, 255, 0), 3, 8, 0);
		}
		markPointNum += m_iCount;
		cv::imwrite(m_sLogPath + m_sCurImgId + "_mark.jpg", imgframe.image);
	}

	return markPointNum > 3;
}


void ColoredPoint::MouseEvent(int event, int x, int y, int flags, void* p) {
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
		ss << m_iCount << " (" << x << ", " << y << ") "
			<< " (" << blue << "," << green << "," << red << ")";
		text = ss.str();

		for (int i = 0; i < m_vPoint.size(); i++) {
			cv::putText(img_clone, m_vText[i], m_vPoint[i], 0, 2.0, cv::Scalar(0, 0, 255), 3, 8, 0);
			cv::circle(img_clone, m_vPoint[i], 4, cv::Scalar(0, 255, 0), 3, 8, 0);
		}

		cv::putText(img_clone, text, point, 0, 2.0, cv::Scalar(0, 0, 255), 3, 8);
		cv::circle(img_clone, point, 4, cv::Scalar(0, 255, 0), 3, 8, 0);

		cv::imshow(m_sCurImgId, img_clone);

	}
	else if (event == cv::EVENT_RBUTTONDOWN) {
		std::string filename = m_sLogPath + "pixel_coord.txt";
		std::ofstream ofs(filename, std::ios::app);
		if (!ofs.is_open()) {
			LOG(INFO) << "Open file error: " << m_sLogPath + "pixel_coord.txt";
			system("pause");
			return;
		}

		std::stringstream ss;
		ss << m_iCount << " " << m_sCurImgId << " " << point.x << " " << point.y << " ";
		ofs << ss.str() << std::endl;
		m_vText.push_back(text);
		m_vPoint.push_back(point);
		m_iCount++;
	}
}

void ColoredPoint::VtkDisplay()
{
	pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer);
	int v1;
	view->createViewPort(0., 0., 1.0, 1.0, v1);
	view->setBackgroundColor(0, 0, 0, v1);
	view->addPointCloud<BasePoint>(m_pPoints, "sample cloud", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud", v1);

	// 启动交互式选点
	while (!view->wasStopped()) {
		view->spinOnce(100);
		// 在这里可以添加处理用户选择点的代码
	}
}

void ColoredPoint::NdtInit(double ndtResolution)
{
	m_pNdtOmp->setResolution(ndtResolution);
	m_pNdtOmp->setNumThreads(4);
	m_pNdtOmp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
	m_pNdtOmp->setTransformationEpsilon(1e-3);
	m_pNdtOmp->setStepSize(0.01);
	m_pNdtOmp->setMaximumIterations(50);
}





