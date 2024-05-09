#include "ReadPts.h"

namespace sc
{
    SenSorParamsConfig::Ptr SendataIo::configPtr_;


    SendataIo::~SendataIo()
    {
    }
    

    SendataIo::SendataIo(const SenSorParamsConfig::Ptr& config)
    {
        configPtr_ = config;
        enable_imu_process_thr_ = false;
        enable_lidar_process_thr_ = false;
        lidar_start_time_ = -1;
        lidar_end_time_ = -1;
        lidarframe_id_ = -1;
    }

    bool SendataIo::InitWithConfig()
    {
        return true;
    }

    bool SendataIo::Start(boost::function<void(LidarFrame)> lidar_callback, boost::function<void(ImuFrame)> imu_callback)
    {
        //1. 确保路径是否存在
        if (IsExists(configPtr_->LaserFilePath))
            enable_lidar_process_thr_ = true;
        else
        {
            LOG(INFO) << "The input laser file is not exist! " << configPtr_->LaserFilePath;
        }
        if (IsExists(configPtr_->ImuFilePath))
            enable_imu_process_thr_ = true;
        else
        {
            LOG(INFO) << "The input Imu file is not exist! " << configPtr_->ImuFilePath;
        }
        std::cout << "Imu Path: " << configPtr_->ImuFilePath << " Laser Path: " << configPtr_->LaserFilePath << std::endl;
        if (enable_lidar_process_thr_ == false && enable_imu_process_thr_ == false)
        {
            return false;
        }


        //2. 线程初始化
        //2.1 激光的数据，imu的数据解析分别存放在一个线程,imu解析完直接回调，激光器解析完需要打包处理不用再做处理
        //2.2 激光和imu的数据传输放入不同的线程
        if (enable_lidar_process_thr_)
        {
            lidarframe_callback_ = lidar_callback;
            lidar_send_thr_ = new boost::thread(boost::bind(&SendataIo::SendLidarData, this));
            lidar_process_thr_ = new boost::thread(boost::bind(&SendataIo::HandLidarData, this));

        }
        if (enable_imu_process_thr_)
        {
            imuframe_callback_ = imu_callback;
            imu_process_send_thr_ = new boost::thread(boost::bind(&SendataIo::HandImuData, this));


        }
        return true;
    }

    bool SendataIo::SetSenDataParams()
    {

        return false;
    }

    bool SendataIo::GetLidarFrame(LidarFrame& frame)
    {
        return true;
    }
    bool SendataIo::HandLidarData()
    {
        if (configPtr_ == nullptr || configPtr_->LaserFilePath.empty())
        {
            LOG(INFO) << "Can not find raw file path to parse the lidar data!";
        }
        // 若为文件夹, 将所有文件全部进行处理，否则只处理该文件
        if (IsDir(configPtr_->LaserFilePath))
        {
            GetFileNamesFromDir(configPtr_->LaserFilePath, configPtr_->LaserFiles);
        }
        else
        {
            configPtr_->LaserFiles.push_back(configPtr_->LaserFilePath);
        }
        LOG(INFO) << "Process Laser ======>";
        LidarFrame msg;
        double fst1pt = -1;
        for (auto& lidarfile : configPtr_->LaserFiles)
        {
            std::fstream fd(lidarfile);

            BasePoint tmp_pt;
            std::string tmp;
            std::vector<float> splitstr;
            for (int i = 0; ; i++)
            {
                std::getline(fd, tmp);
                if (tmp == "")
                {
                    break;
                }

                sscanf(tmp.c_str(), "%f %f %f %f %lf %d %f", &tmp_pt.x, &tmp_pt.y, &tmp_pt.z, &tmp_pt.intensity, &tmp_pt.time, &tmp_pt.ring, &tmp_pt.angle);
                if (lidar_start_time_ < 0)
                {
                    lidar_start_time_ = tmp_pt.time;
                    if (configPtr_->startProcessTime > -DBL_MAX)
                        configPtr_->startProcessTime += lidar_start_time_;
                    if (configPtr_->startProcessTime < DBL_MAX)
                        configPtr_->endProcessTime += lidar_start_time_;
                }
                BasePoint tmp_0;
                tmp_0.x = 0.; tmp_0.y = 0.; tmp_0.z = 0.;
                auto distance = pcl::euclideanDistance(tmp_0, tmp_pt);
                if (distance > configPtr_->maxDetectDist || distance < configPtr_->minDetectDist || tmp_pt.time < configPtr_->startProcessTime || tmp_pt.time >configPtr_->endProcessTime)
                {
                    continue;
                }

                if (msg.cloudPtr->points.size() == 0)
                {
                    msg.startTime = tmp_pt.time;
                    tmp_pt.time -= msg.startTime;
                    msg.cloudPtr->points.emplace_back(std::move(tmp_pt));
                    continue;
                }
                if (tmp_pt.time - msg.cloudPtr->points[0].time <= configPtr_->lidarSendDuration)
                {
                    tmp_pt.time -= msg.startTime;
                    msg.cloudPtr->points.emplace_back(std::move(tmp_pt));
                    continue;
                }
                else
                {
                    msg.endTime = tmp_pt.time;
                    lidarframe_id_++;
                    msg.frameId = lidarframe_id_;
                    msg.cloudPtr->width = (uint32_t)msg.cloudPtr->points.size();
                    msg.cloudPtr->height = 1;
                    lidarframe_callback_(msg);
                    msg.cloudPtr->clear();
                }


            }
        }

        return true;
    }
    bool SendataIo::HandMotorData()
    {
        return true;
    }
    bool SendataIo::HandImuData()
    {
        LOG(INFO) << "Process IMU 2=====>";
        // //判断文件是否存在
        if (configPtr_ == nullptr || configPtr_->ImuFilePath.empty())
        {
            LOG(INFO) << "Can not find raw file path to parse the IMU data!";
            return true;
        }
        //std::cout << configPtr_->ImuFilePath << std::endl;
        std::fstream fd(configPtr_->ImuFilePath);
        std::string tmp;
        ImuFrame tmp_imu;

        //数据带有头 需要过滤两行
        std::getline(fd, tmp);
        std::cout << tmp << std::endl;
        std::getline(fd, tmp);
        int week;
        for (int i = 0; /*i < 2007500*/; i++)
        {
            std::getline(fd, tmp);
            if (tmp == "")
            {
                break;
            }

            sscanf(tmp.c_str(), "%d %lf %f %f %f %f %f %f", &week, &tmp_imu.time, &tmp_imu.acc[0], &tmp_imu.acc[1], &tmp_imu.acc[2], &tmp_imu.gyr[0], &tmp_imu.gyr[1], &tmp_imu.gyr[2]);
            std::cout << std::setprecision(10) << tmp_imu.time << std::endl;
            imuframe_callback_(tmp_imu);

        }

        return true;


    }
    bool SendataIo::SendLidarData()
    {
        return true;
    }
} // namespace sdio
