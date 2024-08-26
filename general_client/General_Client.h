#ifndef __GENERAL_CLIENT_H__
#define __GENERAL_CLIENT_H__

#include "WebSocketServer.h"
#include "EventLoop.h"
#include "htime.h"
#include "json.hpp"
#include <stdexcept>
#include "OptionCfg.h"
#include "HttpClient.h"
#include "Utils.h"
using namespace std;
using namespace hv;
namespace geosun {


	static const std::string base64_chars =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
		"abcdefghijklmnopqrstuvwxyz"
		"0123456789+/";


	struct SocketClientConfig
	{
		typedef std::shared_ptr<SocketClientConfig> Ptr;
		int             s_iPort;                       //端口号
		bool            s_bSingle;                     //是否支持多人操作
		SocketClientConfig() :s_iPort(8090), s_bSingle(true) {}
	};


	class SocketClient {
	public:
		typedef std::shared_ptr<SocketClient> Ptr;
		SocketClient();
		SocketClient(SocketClientConfig::Ptr config);

		bool Run();
		bool ApplicantApi();
		void CamMessageCallback(CameraOptions::Ptr camcfg);
		void GpsMessageCallback(GpsOptions::Ptr gpscfg);
		void LidMessageCallback(LidarOptions::Ptr lidcfg);

		//获取设备的存储空间  
		void ComputerTotalSize();

		//获取设备的版本号和SN号
		void GetDeviceInfo();

		//数据组织，用于socket通讯
		void CompressMessage();

		//base64编码
		string Base64Encode(const char* bytes, unsigned int length);
		
		//base64解码
		string Base64Decode(const std::string& encode_str);
		
		//获取图片判断是否正常
		void GetImgBase64(Json& jsonInOut, int picNum = 3);
		
		//设置工程的路径,Path需要带有/  代表已经在该目录下
		inline void SetPrjPath(const string path)
		{
			s_sPrjPath += path;
		}
		
		//获取工程路径
		inline string GetPrjPath() { return s_sPrjPath; }

		//清空接收/返回的数据，避免数据混淆
		inline void clear() {
			m_jReceiveValue.clear();
			m_jReturnValue.clear();
		};

		static inline bool is_base64(const char c)
		{
			return (isalnum(c) || (c == '+') || (c == '/'));
		}

		
		SensorOptions::Ptr                    m_pSensorCfg;
		SocketClientConfig::Ptr               m_pConf;

		
		std::vector<std::string>              m_vConnectName;          //与设备连接的客户端用户名
		websocket_server_t                    m_wServer;               //服务器
		HttpService                           m_hRouter;               //用于Http服务
		WebSocketService                      m_wWs;                   //用于socket服务
		Json								  m_jSenSorStatusValue;    //用于socket通讯 进行返回的操作
		Json                                  m_jComputerStatusValue;  //用于socket通讯， 返回设备相关信息
		Json                                  m_jCameraStatusValue;    //用于socket通讯， 返回相机相关信息
		Json                                  m_jGpsStatusValue;       //用于socket通讯， 返回gps相关信息
		Json                                  m_jLidStatusValue;       //用于socket通讯， 返回雷达相关信息
		Json                                  m_jSocketReturn;         //用于socket信息返回
		Json                                  m_jReturnValue;          //用于返回
		Json                                  m_jReceiveValue;         //用于接受传入参数
		std::mutex                            m_mLidMutex;             //雷达数据的锁
		std::mutex                            m_mCamMutex;             //相机数据的锁
		std::mutex                            m_mGpsMutex;             //gps数据的锁
		std::mutex                            m_mSensorMutex;          //传感器数据的锁
		bool                                  m_bStartFlag;            //判断设备是否启动
	private:
		std::string                           s_sPrjPath;              //工程的路径,默认为/mnt/sd/
		std::string                           s_sVerPath;              //版本号的路径
		std::string                           s_sDeviceSN;             //设备的SN号


	};
}
	


#endif // !__GENREAL_CLIENT_H__