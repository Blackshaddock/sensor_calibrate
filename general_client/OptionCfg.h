#ifndef __OPTIONCFG_H__
#define __OPTIONCFG_H__
#include <iostream>
namespace geosun {
	struct LidarOptions
	{
		typedef std::shared_ptr<LidarOptions> Ptr;
		int                                   s_iLidFps;                       //雷达点频
		double                                s_dLidHeight;					   //行高
		double                                s_dLidFov;                       //视野
		double                                s_dLidSpeed;                     //扫描速度
		double                                s_dAngResolution;                //角分辨率
		std::string                           s_sLidStatus;					   //激光器状态
		LidarOptions() :s_iLidFps(100), s_dLidHeight(100.), s_dLidSpeed(0.0), s_dAngResolution(0.0), s_dLidFov(360.0) {}
	};
	
	struct CameraOptions
	{
		typedef std::shared_ptr<CameraOptions> Ptr;
		int                                    s_dCamFps;                      //相机帧率 预留
		std::string                            s_sCamProcessStatus;            //相机工作状态  Idle/Recording
		std::string                            s_sCamStatus;                   //相机状态
		CameraOptions() :s_dCamFps(1), s_sCamProcessStatus("idle"), s_sCamStatus("OFF") {}
	};
	
	struct GpsOptions
	{
		typedef std::shared_ptr<GpsOptions> Ptr;
		int                                    s_iGpsNum;                      //Gps搜星个数
		std::string                            s_sGpsSyncStatus;               //Gps同步状态
		std::string                            s_sGpsStatus;                   //Gps工作状态
		std::string                            s_sImuStatus;                   //imu工作状态
		GpsOptions() : s_iGpsNum(0), s_sGpsSyncStatus("single"), s_sGpsStatus("OFF") {}
	};

	struct ComputerOptions
	{
		typedef std::shared_ptr<ComputerOptions> Ptr;
		int                                      s_iRecordTime;                //采集时间
		std::string                              s_sGlobalStatus;              //采集状态     idle/recording/stopping
		std::string                              s_sVersion;                   //版本号
		std::string                              s_sDeviceSN;                  //设备SN号
		std::string                              s_sPrjName;                   //该组数据工程名
		std::string                              s_sTotalSize;                 //总的存储大小  单位G
		std::string                              s_sRemainSize;                //剩余存储空间  单位G
		std::string                              s_sDatSize;                   //单位兆
		int                                      s_iLidNum;                    //lid文件个数
		int                                      s_iPicNum;                    //照片数量
		ComputerOptions() :s_iRecordTime(0), s_sGlobalStatus("Ready"), s_sVersion(""), s_sDeviceSN(""), s_sPrjName("") {}
	};


	struct SensorOptions
	{
		typedef std::shared_ptr<SensorOptions> Ptr;
		LidarOptions::Ptr                      s_pLidCfg;                     //雷达信息
		CameraOptions::Ptr                     s_pCamCfg;                     //相机信息
		GpsOptions::Ptr                        s_pGpsCfg;					  //Gps信息
		ComputerOptions::Ptr                   s_pComputerCfg;                //设备信息状态
		SensorOptions()
		{
			s_pLidCfg = std::make_shared<LidarOptions>();                     
			s_pCamCfg = std::make_shared<CameraOptions>();
			s_pGpsCfg = std::make_shared<GpsOptions>();
			s_pComputerCfg = std::make_shared<ComputerOptions>();
		}
	};


}
#endif // !__OPTIONCFG_H__
