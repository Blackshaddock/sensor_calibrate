#include "LidarSlam.h"
namespace ls {
	using namespace scanframe;
    using namespace sc;
	bool LidarSlamConfig::LoadConfigFile(const std::string& path)
	{
        YAML::Node rootNode;

        try
        {
            rootNode = YAML::LoadFile(path);
        }
        catch (std::exception& exc)
        {
            LOG(INFO) << "Load config file failed: "
                << exc.what() << ", path: " << path;
            return false;
        }
        scanFrameConfigPtr = std::make_shared<ScanFrameConfig>();
        sensorParamsConfigPtr = std::make_shared<SenSorParamsConfig>();

        try
        {
            
            //获取文件路径
            auto bagNode = rootNode["lid_decode"];
            IsDir("abc");
            GetData(bagNode["lidar_file_path"], sensorParamsConfigPtr->LaserFilePath);
            GetData(bagNode["motor_file_path"], sensorParamsConfigPtr->MotorFilePath);
            GetData(bagNode["imu_file_path"], sensorParamsConfigPtr->ImuFilePath);
            GetData(bagNode["camera_file_path"], sensorParamsConfigPtr->CameraFilePath);
            

            //获取点云参数设置
            bagNode = rootNode["lid_params"];
            GetData(bagNode["min_detect_distance"], sensorParamsConfigPtr->minDetectDist);
            GetData(bagNode["max_detect_distance"], sensorParamsConfigPtr->maxDetectDist);

            GetData(bagNode["start_process_time"], sensorParamsConfigPtr->startProcessTime);
            GetData(bagNode["end_process_time"], sensorParamsConfigPtr->endProcessTime);

            GetData(bagNode["start_process_id"], sensorParamsConfigPtr->startProcessId);
            GetData(bagNode["end_process_id"], sensorParamsConfigPtr->endProcessId);

            GetData(bagNode["lidar_send_duration"], sensorParamsConfigPtr->lidarSendDuration);
            GetData(bagNode["process_duration"], sensorParamsConfigPtr->processDuration);
            std::cout << sensorParamsConfigPtr << std::endl;

            //获取标定参数(alpha X, alpha Y, dp, rotLidar2Motor)
            bagNode = rootNode["motor_intrinsic_parameter"];
            std::vector<double> dp;
            GetData(bagNode["alpha1"], scanFrameConfigPtr->alphaX);
            GetData(bagNode["alpha2"], scanFrameConfigPtr->alphaY);
            GetData(bagNode["alpha3"], scanFrameConfigPtr->alphaZ);
            GetData(bagNode["dp"], dp);
            
            scanFrameConfigPtr->dp << dp[0], dp[1], dp[2];
            dp.clear();
            GetData(bagNode["rotLidar2Motor"], dp);
            scanFrameConfigPtr->rotLidar2Motor << dp[0], dp[1], dp[2], dp[3], dp[4], dp[5], dp[6], dp[7], dp[8], dp[9], dp[10], dp[11], dp[12],
                dp[13], dp[14], dp[15];

            bagNode = rootNode["motor_angle_calibration_parameter"];
            GetData(bagNode["angle_correction_file"], scanFrameConfigPtr->correctAngleFile);

        }
        catch (std::exception& exc)
        {
            LOG(INFO) << "Load calibration file failed: " << exc.what();
            return false;
        }

		return true;
	}

	LidarSlam::LidarSlam()
	{
	}

	LidarSlam::LidarSlam(const LidarSlamConfig::Ptr lidarSlamConfig)
	{
		
		scanFrame_.reset(new ScanFrame(lidarSlamConfig->sensorParamsConfigPtr,lidarSlamConfig->scanFrameConfigPtr));
	}

	bool LidarSlam::Run()
	{
		scanFrame_->Run();
		return true;
	}

}