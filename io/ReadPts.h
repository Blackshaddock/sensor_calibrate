#ifndef __ReadPts_H__
#define __ReadPts_H__

#include "../internal/Structs.h"
#include "float.h"
#include "../internal/Utils.h"
#include <mutex>
#include <boost/function.hpp>
#include <pcl/common/distances.h>
#include <boost/thread.hpp>
#include <thread>
#include <sstream>



namespace sc {
    
    struct SenSorParamsConfig
    {
        typedef std::shared_ptr<SenSorParamsConfig>	Ptr;
        std::string LaserFilePath;
        std::string MotorFilePath;
        std::string ImuFilePath;
        std::string CameraFilePath;
        float minDetectDist;						// 激光器最小扫描距离
        float maxDetectDist;						// 激光器最大扫描距离
        double startProcessTime;					// 开始解析以激光器的时间为基准，imu默认全解析
        double endProcessTime;						// 结束处理
        int startProcessId;							// 开始处理id
        int endProcessId;							// 结束处理id，这两个处理id范围和上面的处理时间范围都是为了调试某段数据使用，若不设置，默认全解
        double processDuration;	                    // 设置处理时间，从起始时间开始算
        double lidarSendDuration;                   //雷达按指定帧数处理
        std::vector<std::string> LaserFiles;        // 雷达文件 
        std::vector<std::string> ImuFiles;          // Imu文件
        std::vector<std::string> MotorFiles;        //编码器文件

        SenSorParamsConfig() : minDetectDist(0.), maxDetectDist(FLT_MAX),
            startProcessTime(-DBL_MAX), endProcessTime(DBL_MAX),
            startProcessId(INT_MIN), endProcessId(INT_MAX), processDuration(DBL_MAX), lidarSendDuration(0.1) {}

        friend std::ostream& operator<< (std::ostream& out, SenSorParamsConfig::Ptr & sensorparamsconfig) {
            out << "laser path: " << sensorparamsconfig->LaserFilePath << " , motor path: " << sensorparamsconfig->MotorFilePath << " , imufile path: " << sensorparamsconfig->ImuFilePath << " , camerafile path: " <<  sensorparamsconfig->CameraFilePath << " , min dist: " << sensorparamsconfig->minDetectDist << " , max dist: " << sensorparamsconfig->maxDetectDist << " , start time: " << sensorparamsconfig->startProcessTime << " , end time: " << sensorparamsconfig->endProcessTime << ", start id" << sensorparamsconfig->startProcessId << " , end id" << sensorparamsconfig->endProcessId << " ,process duration: " << sensorparamsconfig->processDuration << " ,lidar duration: " << sensorparamsconfig->lidarSendDuration;
            return out;


        }
    };



    class SendataIo {
    public:
        typedef std::shared_ptr<SendataIo> Ptr;



        ~SendataIo();
        SendataIo() {}

        SendataIo(const SenSorParamsConfig::Ptr& config);

        //参数初始化
        bool InitWithConfig();

        //数据解析开始 
        bool Start(boost::function<void(LidarFrame)> lidar_callback, boost::function<void(ImuFrame)>imu_callback);

        bool SetSenDataParams();

        //获取一帧雷达数据
        bool GetLidarFrame(LidarFrame& frame);

        //解析雷达数据
        bool HandLidarData();

        //解析编码器数据
        bool HandMotorData();

        //解析IMU数据
        bool HandImuData();

        //rosbag形式保存数据
        bool WriteData2RosBag();

        //将激光器数据打包发出
        bool SendLidarData();

        //获取文件路径
        std::string& GetParseFilePath()
        {
            return configPtr_->LaserFilePath;
        }







    private:
        static SenSorParamsConfig::Ptr configPtr_;

        std::deque<ImuFrame> rawImuDeq_;
        std::deque<BasePoint> rawLaserPointDeq_;
        std::deque<LidarFrame> rawLaserDeq_;

        //用于将数据callback回去
        boost::function<void(LidarFrame cld)> lidarframe_callback_;
        boost::function<void(ImuFrame cld)> imuframe_callback_;

        //处理数据线程
        bool enable_imu_process_thr_;
        bool enable_lidar_process_thr_;
        boost::thread* lidar_process_thr_;
        boost::thread* lidar_send_thr_;
        boost::thread* imu_process_send_thr_;

        std::mutex lidar_mutex_;
        std::mutex imu_mutex_;

        //激光的起始时间
        double lidar_start_time_;
        double lidar_end_time_;
        int lidarframe_id_;


    };
}

#endif
