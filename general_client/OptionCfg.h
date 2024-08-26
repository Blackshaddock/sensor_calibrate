#ifndef __OPTIONCFG_H__
#define __OPTIONCFG_H__
#include <iostream>


namespace geosun {
	struct LidarOptions
	{
		typedef std::shared_ptr<LidarOptions> Ptr;
		int                                   s_iLidFps;                       //�״��Ƶ
		double                                s_dLidHeight;					   //�и�
		double                                s_dLidFov;                       //��Ұ
		std::string                           s_sLidStatus;					   //������״̬
		LidarOptions() :s_iLidFps(100), s_dLidHeight(100.), s_dLidFov(360.0) {}
	};
	
	struct CameraOptions
	{
		typedef std::shared_ptr<CameraOptions> Ptr;
		int                                    s_iPicNum;                      //��Ƭ��
		int                                    s_dCamFps;                      //���֡�� Ԥ��
		std::string                            s_sCamProcessStatus;            //�������״̬  Idle/Recording
		std::string                            s_sCamStatus;                   //���״̬
		CameraOptions() :s_iPicNum(0), s_dCamFps(1), s_sCamProcessStatus("idle"), s_sCamStatus("OFF") {}
	};
	
	struct GpsOptions
	{
		typedef std::shared_ptr<GpsOptions> Ptr;
		int                                    s_iGpsNum;                      //Gps���Ǹ���
		std::string                            s_sGpsSyncStatus;               //Gpsͬ��״̬
		std::string                            s_sGpsStatus;                   //Gps����״̬
		std::string                            s_sImuStatus;                   //imu����״̬
		GpsOptions() : s_iGpsNum(0), s_sGpsSyncStatus("single"), s_sGpsStatus("OFF") {}
	};

	struct ComputerOptions
	{
		typedef std::shared_ptr<ComputerOptions> Ptr;
		int                                      s_iRecordTime;                //�ɼ�ʱ��
		std::string                              s_sGlobalStatus;              //�ɼ�״̬     idle/recording/stopping
		std::string                              s_sVersion;                   //�汾��
		std::string                              s_sDeviceSN;                  //�豸SN��
		std::string                              s_sPrjName;                   //�������ݹ�����
		double                                   s_sTotalSize;                 //�ܵĴ洢��С  ��λG
		double                                   s_sRemainSize;                //ʣ��洢�ռ�  ��λG
		ComputerOptions() :s_iRecordTime(0), s_sGlobalStatus("Ready"), s_sVersion(""), s_sDeviceSN(""), s_sPrjName("") {}
	};


	struct SensorOptions
	{
		typedef std::shared_ptr<SensorOptions> Ptr;
		LidarOptions::Ptr                      s_pLidCfg;                     //�״���Ϣ
		CameraOptions::Ptr                     s_pCamCfg;                     //�����Ϣ
		GpsOptions::Ptr                        s_pGpsCfg;					  //Gps��Ϣ
		ComputerOptions::Ptr                   s_pComputerCfg;                //�豸��Ϣ״̬
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
