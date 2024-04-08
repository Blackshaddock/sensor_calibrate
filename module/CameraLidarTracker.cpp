#include "module/CameraLidarTracker.h"
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace clt {
	using namespace sc;
	CameraLidarTrackerConfig::Ptr CameraLidarTracker::configPtr_;
	CameraLidarTracker::CameraLidarTracker(CameraLidarTrackerConfig::Ptr& config)
	{
		configPtr_ = config;
	}

	CameraLidarTracker::~CameraLidarTracker()
	{
	}

	bool CameraLidarTracker::Run()
	{

		//cv::VideoCapture capture(0); // 打开摄像头，如果是视频文件，可以指定文件路径
		//if (!capture.isOpened())
		//{
		//	std::cout << "无法打开摄像头或视频文件" << std::endl;
		//	return -1;
		//}
		//cv::Mat frame;
		//capture.read(frame); // 读取第一帧图像
		//cv::Rect2d bbox = cv::selectROI(frame, false); // 选择目标对象的初始位置
		//cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create(); // 创建KCF跟踪器
		//tracker->init(frame, bbox); // 初始化跟踪器
		//while (capture.read(frame))
		//{
		//	bool ok = tracker->update(frame, bbox); // 更新跟踪器
		//	if (ok)
		//	{
		//		cv::rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2, 1); // 绘制跟踪框
		//	}
		//	else
		//	{
		//		cv::putText(frame, "跟踪失败", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
		//	}
		//	cv::imshow("跟踪", frame);
		//	if (cv::waitKey(1) == 27) // 按下ESC键退出
		//	{
		//		break;
		//	}
		//}
		//capture.release();




		return true;
	}

	bool CameraLidarTracker::InitWithConfig()
	{
		return true;
	}



	bool CameraLidarTrackerConfig::LoadConfigFile(const std::string& path)
	{
		return true;
	}

	template <typename T>
	bool CameraLidarTrackerConfig::GetData(const YAML::Node& node, T& value) {
		if (!node.IsDefined()) {
			return false;
		}
		value = node.as<T>();
		return true;
	}

}


