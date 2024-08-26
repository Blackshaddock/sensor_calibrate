#include "General_Client.h"
#include <fstream>
using namespace geosun;



SocketClient::SocketClient()
{
	m_pConf = std::make_shared<SocketClientConfig>();
    m_pSensorCfg = std::make_shared<SensorOptions>();
    s_sPrjPath = "E:\\data\\RTK\\RTK\\";
    m_bStartFlag = false;
    s_sVerPath = "";
}

SocketClient::SocketClient(SocketClientConfig::Ptr config) : m_pConf(config) {
    s_sPrjPath = "E:\\data\\RTK\\RTK\\";
    m_bStartFlag = false;
    s_sVerPath = "";
}

bool geosun::SocketClient::Run()
{
    m_wWs.onopen = [&](const WebSocketChannelPtr& channel, const HttpRequestPtr& req) {
        printf("onopen: GET %s\n", req->Path().c_str());
        // send(time) every 1s
        setInterval(1000, [&](TimerID id) {
            if (channel->isConnected()) {
                char str[DATETIME_FMT_BUFLEN] = { 0 };
                datetime_t dt = datetime_now();
                datetime_fmt(&dt, str);
                CompressMessage();
                channel->send(m_jSocketReturn.dump());
            }
            else {
                killTimer(id);
            }
            });
    };

    m_wWs.onmessage = [&](const WebSocketChannelPtr& channel, const std::string& msg) {
        if (find(m_vConnectName.begin(), m_vConnectName.end(), msg) == m_vConnectName.end())
        {
            m_vConnectName.emplace_back(msg);
        }
    };
    m_wWs.onclose = [&](const WebSocketChannelPtr& channel) {
        printf("onclose\n");
    };

    ApplicantApi();

    
    m_wServer.port = m_pConf->s_iPort;
    m_wServer.ws = &m_wWs;
    m_wServer.service = &m_hRouter;
    websocket_server_run(&m_wServer, 0);

	return true;
}

bool geosun::SocketClient::ApplicantApi()
{
    // 设备连接
    m_hRouter.POST("/GeoSun_Connect", [&](const HttpContextPtr& ctx)
        {
            //1. 判断是否已经有设备进行了链接,
            try {
                m_jReturnValue.clear();
                m_jReturnValue["status"] = true;
                if (!m_vConnectName.empty())
                {
                    if (m_pConf->s_bSingle)
                    {
                        m_jReturnValue["connect_flag"] = false;
                        return ctx->send(m_jReturnValue.dump(), ctx->type());
                    }
                }
                m_jReturnValue["connect_flag"] = true;
                if (IsExists(s_sVerPath))
                {
                    fstream fd(s_sVerPath);
                    std::getline(fd, m_pSensorCfg->s_pComputerCfg->s_sVersion);
                    m_jReturnValue["device_version"] = m_pSensorCfg->s_pComputerCfg->s_sVersion;
                }
                if (IsExists(s_sDeviceSN))
                {
                    fstream fd(s_sDeviceSN);
                    std::getline(fd, m_pSensorCfg->s_pComputerCfg->s_sDeviceSN);
                    m_jReturnValue["device_sn"] = m_pSensorCfg->s_pComputerCfg->s_sDeviceSN;
                }
                
            }
            catch (std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                m_jReturnValue["status"] = false;

            }
            catch (...)
            {
                std::cerr << "GeoSun_Connect Unknow error" << std::endl;
                m_jReturnValue["status"] = false;

            }
            return ctx->send(m_jReturnValue.dump(), ctx->type());
        });


    //断开设备连接
    m_hRouter.POST("/GeoSun_DisConnect", [&](const HttpContextPtr& ctx)
        {
            try {
                m_jReturnValue.clear();
                m_jReturnValue["status"] = true;
            }
            catch (std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                std::cerr << "GeoSun_DisConnect Unknow error" << std::endl;
                m_jReturnValue["status"] = false;
            }
            return ctx->send(m_jReturnValue.dump(), ctx->type());
        });

    //是否支持多设备连接
    m_hRouter.POST("/GeoSun_MultDevice", [&](const HttpContextPtr& ctx)
        {
            try {
                clear();
                m_jReceiveValue = ctx->json();
                if (m_jReceiveValue["input_status"] == "get")
                {
                    m_jReturnValue["single_flag"] = m_pConf->s_bSingle;
                }
                else if (m_jReceiveValue["input_status"] == "set")
                {
                    m_pConf->s_bSingle = m_jReceiveValue["single_flag"];
                    m_jReturnValue["single_flag"] = m_pConf->s_bSingle;
                }
                m_jReturnValue["status"] = true;
            }
            catch (std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                std::cerr << "GeoSun_MultDevice Unknow error" << std::endl;
                m_jReturnValue["status"] = false;
            }
            return ctx->send(m_jReturnValue.dump(), ctx->type());
        });


    //开始采集或停止采集
    m_hRouter.POST("/GeoSun_StartAndStop", [&](const HttpContextPtr& ctx)
        {
            try {
                clear();
                m_jReceiveValue = ctx->json();
                if (m_jReceiveValue["input_status"] == "start")
                {
                    m_pSensorCfg->s_pComputerCfg->s_sGlobalStatus = "Prepare";
                    //GetImgBase64(m_jReturnValue);
                    return ctx->send(m_jReturnValue.dump());
                }
                else if (m_jReceiveValue["input_status"] == "stop")
                {
                    m_pSensorCfg->s_pComputerCfg->s_sGlobalStatus = "Stopping";
                }


            }
            catch (exception& e)
            {
                std::cerr << e.what() << std::endl;
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                std::cerr << "GeoSun_StartAndStop Unknow error" << std::endl;
                m_jReturnValue["status"] = false;
            }
            return ctx->send(m_jReturnValue.dump());

        });


     //获取图片接口
    m_hRouter.POST("/GeoSun_GetImage", [&](const HttpContextPtr& ctx)
        {
            try {
                clear();
                m_jReceiveValue = ctx->json();
                GetImgBase64(m_jReturnValue, m_jReceiveValue["image_count"]);
            }
            catch (exception& e)
            {
                std::cerr << e.what() << std::endl;
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                std::cerr << "GeoSun_GetImage Unknow error" << std::endl;
                m_jReturnValue["status"] = false;
            }
            return ctx->send(m_jReturnValue.dump());
        });


    





    return true;
}

void geosun::SocketClient::CamMessageCallback(CameraOptions::Ptr camcfg)
{
    m_mSensorMutex.lock();
    m_pSensorCfg->s_pCamCfg = camcfg;
    m_mSensorMutex.unlock();
}

void geosun::SocketClient::GpsMessageCallback(GpsOptions::Ptr gpscfg)
{
    m_mSensorMutex.lock();
    m_pSensorCfg->s_pGpsCfg = gpscfg;
    m_mSensorMutex.unlock();
}

void geosun::SocketClient::LidMessageCallback(LidarOptions::Ptr lidcfg)
{
    m_mSensorMutex.lock();
    m_pSensorCfg->s_pLidCfg = lidcfg;
    m_mSensorMutex.unlock();
    std::cout << m_pSensorCfg->s_pLidCfg->s_dLidHeight << std::endl;
}

void geosun::SocketClient::CompressMessage()
{
    m_jSocketReturn.clear();
    //Sensor Status
    m_jSenSorStatusValue["lidar_status"] = m_pSensorCfg->s_pLidCfg->s_sLidStatus;
    m_jSenSorStatusValue["gps_status"] = m_pSensorCfg->s_pGpsCfg->s_sGpsStatus;
    m_jSenSorStatusValue["camera_status"] = m_pSensorCfg->s_pCamCfg->s_sCamStatus;
    m_jSenSorStatusValue["imu_status"] = m_pSensorCfg->s_pGpsCfg->s_sImuStatus;
    m_jSocketReturn["Sensor_Status"] = m_jSenSorStatusValue;

    //Computer Status
    m_jComputerStatusValue["computer_totalspace"] = m_pSensorCfg->s_pComputerCfg->s_sTotalSize;
    m_jComputerStatusValue["remain_space"] = m_pSensorCfg->s_pComputerCfg->s_sRemainSize;
    m_jComputerStatusValue["global_status"] = m_pSensorCfg->s_pComputerCfg->s_sGlobalStatus;
    m_jComputerStatusValue["project_name"] = m_pSensorCfg->s_pComputerCfg->s_sPrjName;
    m_jComputerStatusValue["record_time"] = m_pSensorCfg->s_pComputerCfg->s_iRecordTime;
    m_jSocketReturn["Computer_Status"] = m_jComputerStatusValue;

    //Lidar Status 预留由于在雷达的参数设置中已经存在这些配置
    m_jLidStatusValue["lidar_fps"] = m_pSensorCfg->s_pLidCfg->s_iLidFps;
    m_jLidStatusValue["lidar_height"] = m_pSensorCfg->s_pLidCfg->s_dLidHeight;
    m_jLidStatusValue["lidar_fov"] = m_pSensorCfg->s_pLidCfg->s_dLidFov;
    m_jSocketReturn["Lidar_Status"] = m_jLidStatusValue;

    //Gps Status
    m_jGpsStatusValue["star_num"] = m_pSensorCfg->s_pGpsCfg->s_iGpsNum;
    m_jGpsStatusValue["sync_status"] = m_pSensorCfg->s_pGpsCfg->s_sGpsSyncStatus;
    m_jSocketReturn["Gps_Status"] = m_jGpsStatusValue;

    //Camera Status
    m_jCameraStatusValue["pic_num"] = m_pSensorCfg->s_pCamCfg->s_iPicNum;
    m_jCameraStatusValue["camera_fps"] = m_pSensorCfg->s_pCamCfg->s_dCamFps;
    m_jCameraStatusValue["camera_process"] = m_pSensorCfg->s_pCamCfg->s_sCamProcessStatus;
    m_jSocketReturn["Camera_Status"] = m_jCameraStatusValue;



}

string geosun::SocketClient::Base64Encode(const char* bytes, unsigned int length)
{
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    while (length--)
    {
        char_array_3[i++] = *(bytes++);
        if (i == 3)
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for (i = 0; (i < 4); i++)
            {
                ret += base64_chars[char_array_4[i]];
            }
            i = 0;
        }
    }
    if (i)
    {
        for (j = i; j < 3; j++)
        {
            char_array_3[j] = '\0';
        }

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
        {
            ret += base64_chars[char_array_4[j]];
        }

        while ((i++ < 3))
        {
            ret += '=';
        }
    }
    return ret;

   
}

string geosun::SocketClient::Base64Decode(const std::string& encode_str)
{
    int in_len = (int)encode_str.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    std::string ret;

    while (in_len-- && (encode_str[in_] != '=') && is_base64(encode_str[in_])) {
        char_array_4[i++] = encode_str[in_]; in_++;
        if (i == 4) {
            for (i = 0; i < 4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                ret += char_array_3[i];
            i = 0;
        }
    }
    if (i) {
        for (j = i; j < 4; j++)
            char_array_4[j] = 0;

        for (j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
    }

    return ret;
}

void geosun::SocketClient::GetImgBase64(Json& jsonInOut, int picNum)
{
    std::string imgPath = s_sPrjPath + "Image\\";
    if (IsDir(imgPath))
    {
        std::vector<string>imgFileNames = GetFileNamesFromDir(imgPath);
        if (imgFileNames.empty())
        {
            jsonInOut["imgfile_flag"] = false;
        }
        else {
            jsonInOut["imgfile_flag"] = true;
            for (int i = 0 ; i < picNum; i++)
            {
                if (i > imgFileNames.size() -1 )
                {
                    break;
                }
                std::string imgfile = imgFileNames[i];
                ifstream in(imgfile, ifstream::in | ios::binary);
                in.seekg(0, in.end);
                int length = in.tellg();
                in.seekg(0, in.beg);
                char* buffer = new char[length];
                in.read(buffer, length);
                jsonInOut["img_str"].push_back(Base64Encode(buffer, length));
                in.close();

            }
            jsonInOut["status"] = true;

        }
    }
}

