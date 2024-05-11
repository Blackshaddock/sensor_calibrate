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
        float minDetectDist;						// ��������Сɨ�����
        float maxDetectDist;						// ���������ɨ�����
        double startProcessTime;					// ��ʼ�����Լ�������ʱ��Ϊ��׼��imuĬ��ȫ����
        double endProcessTime;						// ��������
        int startProcessId;							// ��ʼ����id
        int endProcessId;							// ��������id������������id��Χ������Ĵ���ʱ�䷶Χ����Ϊ�˵���ĳ������ʹ�ã��������ã�Ĭ��ȫ��
        double processDuration;	                    // ���ô���ʱ�䣬����ʼʱ�俪ʼ��
        double lidarSendDuration;                   //�״ﰴָ��֡������
        std::vector<std::string> LaserFiles;        // �״��ļ� 
        std::vector<std::string> ImuFiles;          // Imu�ļ�
        std::vector<std::string> MotorFiles;        //�������ļ�

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

        //������ʼ��
        bool InitWithConfig();

        //���ݽ�����ʼ 
        bool Start(boost::function<void(LidarFrame)> lidar_callback, boost::function<void(ImuFrame)>imu_callback);

        bool SetSenDataParams();

        //��ȡһ֡�״�����
        bool GetLidarFrame(LidarFrame& frame);

        //�����״�����
        bool HandLidarData();

        //��������������
        bool HandMotorData();

        //����IMU����
        bool HandImuData();

        //rosbag��ʽ��������
        bool WriteData2RosBag();

        //�����������ݴ������
        bool SendLidarData();

        //��ȡ�ļ�·��
        std::string& GetParseFilePath()
        {
            return configPtr_->LaserFilePath;
        }







    private:
        static SenSorParamsConfig::Ptr configPtr_;

        std::deque<ImuFrame> rawImuDeq_;
        std::deque<BasePoint> rawLaserPointDeq_;
        std::deque<LidarFrame> rawLaserDeq_;

        //���ڽ�����callback��ȥ
        boost::function<void(LidarFrame cld)> lidarframe_callback_;
        boost::function<void(ImuFrame cld)> imuframe_callback_;

        //���������߳�
        bool enable_imu_process_thr_;
        bool enable_lidar_process_thr_;
        boost::thread* lidar_process_thr_;
        boost::thread* lidar_send_thr_;
        boost::thread* imu_process_send_thr_;

        std::mutex lidar_mutex_;
        std::mutex imu_mutex_;

        //�������ʼʱ��
        double lidar_start_time_;
        double lidar_end_time_;
        int lidarframe_id_;


    };
}

#endif
