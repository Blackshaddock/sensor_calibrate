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
		int             s_iPort;                       //�˿ں�
		bool            s_bSingle;                     //�Ƿ�֧�ֶ��˲���
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

		//��ȡ�豸�Ĵ洢�ռ�  
		void ComputerTotalSize();

		//��ȡ�豸�İ汾�ź�SN��
		void GetDeviceInfo();

		//������֯������socketͨѶ
		void CompressMessage();

		//base64����
		string Base64Encode(const char* bytes, unsigned int length);
		
		//base64����
		string Base64Decode(const std::string& encode_str);
		
		//��ȡͼƬ�ж��Ƿ�����
		void GetImgBase64(Json& jsonInOut, int picNum = 3);
		
		//���ù��̵�·��,Path��Ҫ����/  �����Ѿ��ڸ�Ŀ¼��
		inline void SetPrjPath(const string path)
		{
			s_sPrjPath += path;
		}
		
		//��ȡ����·��
		inline string GetPrjPath() { return s_sPrjPath; }

		//��ս���/���ص����ݣ��������ݻ���
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

		
		std::vector<std::string>              m_vConnectName;          //���豸���ӵĿͻ����û���
		websocket_server_t                    m_wServer;               //������
		HttpService                           m_hRouter;               //����Http����
		WebSocketService                      m_wWs;                   //����socket����
		Json								  m_jSenSorStatusValue;    //����socketͨѶ ���з��صĲ���
		Json                                  m_jComputerStatusValue;  //����socketͨѶ�� �����豸�����Ϣ
		Json                                  m_jCameraStatusValue;    //����socketͨѶ�� ������������Ϣ
		Json                                  m_jGpsStatusValue;       //����socketͨѶ�� ����gps�����Ϣ
		Json                                  m_jLidStatusValue;       //����socketͨѶ�� �����״������Ϣ
		Json                                  m_jSocketReturn;         //����socket��Ϣ����
		Json                                  m_jReturnValue;          //���ڷ���
		Json                                  m_jReceiveValue;         //���ڽ��ܴ������
		std::mutex                            m_mLidMutex;             //�״����ݵ���
		std::mutex                            m_mCamMutex;             //������ݵ���
		std::mutex                            m_mGpsMutex;             //gps���ݵ���
		std::mutex                            m_mSensorMutex;          //���������ݵ���
		bool                                  m_bStartFlag;            //�ж��豸�Ƿ�����
	private:
		std::string                           s_sPrjPath;              //���̵�·��,Ĭ��Ϊ/mnt/sd/
		std::string                           s_sVerPath;              //�汾�ŵ�·��
		std::string                           s_sDeviceSN;             //�豸��SN��


	};
}
	


#endif // !__GENREAL_CLIENT_H__