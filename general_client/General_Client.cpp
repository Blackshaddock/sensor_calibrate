#include "General_Client.h"
#include <fstream>
using namespace geosun;



SocketClient::SocketClient()
{
    
	m_pConf = std::make_shared<SocketClientConfig>();
    m_pSensorCfg = std::make_shared<SensorOptions>();
    m_sPrjPath = "E:\\data\\RTK\\RTK\\";
    m_bStartFlag = false;
    m_sVerPath = "";
    m_pProcessClent = nullptr;
    m_pProcessAg310 = nullptr;
    m_pProcessLidar = nullptr;

}

SocketClient::SocketClient(SocketClientConfig::Ptr config) : m_pConf(config) {
    m_pSensorCfg = std::make_shared<SensorOptions>();
    m_sPrjPath = "E:\\data\\RTK\\RTK\\";
    m_bStartFlag = false;
    m_sVerPath = "";
    m_pProcessClent = nullptr;
    m_pProcessAg310 = nullptr;
    m_pProcessLidar = nullptr;
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
                ComputerSize();
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
    m_pProcessClent = new boost::thread(boost::bind(&SocketClient::ProcessClent, this));
    //m_pProcessClent->join();
    m_wServer.port = m_pConf->s_iPort;
    m_wServer.ws = &m_wWs;
    m_wServer.service = &m_hRouter;
    websocket_server_run(&m_wServer, 0);

	return true;
}

bool geosun::SocketClient::ApplicantApi()
{
    //LOG_INFO("Single thread test: Info log message");
    // 
    m_hRouter.POST("/GeoSun_Test", [&](const HttpContextPtr& ctx)
        {
            try {
                m_jReturnValue.clear();
                m_jReceiveValue = ctx->json();
                
            }
            catch (std::exception& e)
            {
                LOG_INFO("API GeoSun Test %s", e.what());
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                LOG_INFO("API GeoSun Test Unknow error ");
                m_jReturnValue["status"] = false;
            }
            return ctx->send(m_jReturnValue.dump(), ctx->type());
        });
    
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
                if (IsExists(m_sVerPath))
                {
                    std::fstream fd(m_sVerPath);
                    std::getline(fd, m_pSensorCfg->s_pComputerCfg->s_sVersion);
                    m_jReturnValue["device_version"] = m_pSensorCfg->s_pComputerCfg->s_sVersion;
                }
                if (IsExists(m_sDeviceSN))
                {
                    std::fstream fd(m_sDeviceSN);
                    std::getline(fd, m_pSensorCfg->s_pComputerCfg->s_sDeviceSN);
                    m_jReturnValue["device_sn"] = m_pSensorCfg->s_pComputerCfg->s_sDeviceSN;
                }
                
            }
            catch (std::exception& e)
            {
                LOG_INFO("API GeoSun Connect %s", e.what());
                m_jReturnValue["status"] = false;

            }
            catch (...)
            {
                LOG_INFO("API GeoSun Connect Unknow error ");
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
                LOG_INFO("API GeoSun DisConnect %s", e.what());
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                LOG_INFO("API GeoSun DisConnect Unknow error ");
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
                LOG_INFO("API GeoSun MultDevice %s", e.what());
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                LOG_INFO("API GeoSun MultDevice Unknow error ");
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
                    SendSocketData(m_pLidarSocket, std::to_string(1).c_str());
                    return ctx->send(m_jReturnValue.dump());
                }
                else if (m_jReceiveValue["input_status"] == "stop")
                {
                    m_pSensorCfg->s_pComputerCfg->s_sGlobalStatus = "Stopping";
                    SendSocketData(m_pAg310Socket, std::to_string(2).c_str());
                }


            }
            catch (std::exception& e)
            {
                LOG_INFO("API GeoSun StartAndStop %s", e.what());
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                LOG_INFO("API GeoSun StartAndStop Unknow error ");
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
            catch (std::exception& e)
            {
                LOG_INFO("API GeoSun GetImage %s", e.what());
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                LOG_INFO("API GeoSun GetImage Unknow error ");
                m_jReturnValue["status"] = false;
            }
            return ctx->send(m_jReturnValue.dump());
        });

    
    //设置相机的帧率
    m_hRouter.POST("/GeoSun_CamParams", [&](const HttpContextPtr& ctx)
        {
            try {
                m_jReturnValue.clear();
                m_jReceiveValue = ctx->json();
                if (m_jReceiveValue["input_status"] == "get")
                {
                    m_jReturnValue["status"] = true;
                    //TODO
                    
                    //

                    m_jReturnValue["camera_fps"] = m_pSensorCfg->s_pCamCfg->s_dCamFps;
                }
                else if (m_jReceiveValue["input_status"] == "set")
                {
                    m_pSensorCfg->s_pCamCfg->s_dCamFps = m_jReceiveValue["camera_fps"];
                    m_jReturnValue["status"] = true;
                    m_jReturnValue["camera_fps"] = m_pSensorCfg->s_pCamCfg->s_dCamFps;

                }
            }
            catch (std::exception& e)
            {
                LOG_INFO("API GeoSun Test %s", e.what());
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                LOG_INFO("API GeoSun Test Unknow error ");
                m_jReturnValue["status"] = false;
            }
            return ctx->send(m_jReturnValue.dump(), ctx->type());
        });

    //设置激光器行高 点频 扫描速度 视场角 角分辨率
    m_hRouter.POST("/GeoSun_LidarParams", [&](const HttpContextPtr& ctx)
        {
            try {
                m_jReturnValue.clear();
                m_jReceiveValue = ctx->json();
                Json j;
                std::ofstream out("D:\\test1.json");
                std::cout << m_jReceiveValue.dump() << std::endl;
                if (m_jReceiveValue["input_status"] == "get")
                {
                    if (IsExists("D:\\lidarparams.json"))
                    {
                        std::ifstream jfile("D:\\lidarparams.json");
                        
                        jfile >> j;
                        std::cout << j.at("data") << std::endl;
                       /* out << j.dump(4) << std::endl;
                        out.flush();
                        out.close();*/
                        m_jReturnValue["data"] = j.at("data");
                        m_jReturnValue["lidar_fov"] = m_pSensorCfg->s_pLidCfg->s_dLidFov;
                        m_jReturnValue["status"] = true;
                    }
                    //获取当前的激光器的设置结果，并返回
                }
                else if (m_jReceiveValue["input_status"] == "set")
                {

                    //设置当前激光器的参数，返回一个表
                }
                else if (m_jReceiveValue["input_status"] == "add")
                {
                    //添加自定义的参数，并需要将该自定义参数保存为一个表
                }

            }
            catch (std::exception& e)
            {
                LOG_INFO("API GeoSun Test %s", e.what());
                m_jReturnValue["status"] = false;
            }
            catch (...)
            {
                LOG_INFO("API GeoSun Test Unknow error ");
                m_jReturnValue["status"] = false;
            }
            return ctx->send(m_jReturnValue.dump(), ctx->type());
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
}

void geosun::SocketClient::ComputerSize()
{
    bfs::space_info si = DiskInfo("F:");
    m_pSensorCfg->s_pComputerCfg->s_sTotalSize = si.capacity / 1024. / 1024. / 1024.;
    m_pSensorCfg->s_pComputerCfg->s_sRemainSize = si.available / 1024. / 1024. / 1024.;
    if (IsDir(m_sPrjPath + "Image\\"))
        m_pSensorCfg->s_pComputerCfg->s_iPicNum = GetFileNamesFromDir(m_sPrjPath + "Image\\").size();
    
    if (IsDir(m_sPrjPath + "Lidar\\"))
        m_pSensorCfg->s_pComputerCfg->s_iLidNum = GetFileNamesFromDir(m_sPrjPath + "Lidar\\").size();

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
    m_jComputerStatusValue["pic_num"] = m_pSensorCfg->s_pComputerCfg->s_iPicNum;
    m_jComputerStatusValue["lid_num"] = m_pSensorCfg->s_pComputerCfg->s_iLidNum;
    m_jComputerStatusValue["dat_size"] = m_pSensorCfg->s_pComputerCfg->s_sDatSize;
    m_jSocketReturn["Computer_Status"] = m_jComputerStatusValue;

    //Lidar Status 预留由于在雷达的参数设置中已经存在这些配置
    m_jLidStatusValue["lidar_fps"] = m_pSensorCfg->s_pLidCfg->s_iLidFps;
    m_jLidStatusValue["lidar_height"] = m_pSensorCfg->s_pLidCfg->s_dLidHeight;
    m_jLidStatusValue["lidar_fov"] = m_pSensorCfg->s_pLidCfg->s_dLidFov;
    m_jLidStatusValue["lidar_speed"] = m_pSensorCfg->s_pLidCfg->s_dLidSpeed;
    m_jLidStatusValue["lidar_angscale"] = m_pSensorCfg->s_pLidCfg->s_dAngResolution;
    m_jSocketReturn["Lidar_Status"] = m_jLidStatusValue;

    //Gps Status
    m_jGpsStatusValue["star_num"] = m_pSensorCfg->s_pGpsCfg->s_iGpsNum;
    m_jGpsStatusValue["sync_status"] = m_pSensorCfg->s_pGpsCfg->s_sGpsSyncStatus;
    m_jSocketReturn["Gps_Status"] = m_jGpsStatusValue;

    //Camera Status
    
    m_jCameraStatusValue["camera_fps"] = m_pSensorCfg->s_pCamCfg->s_dCamFps;
    m_jCameraStatusValue["camera_process"] = m_pSensorCfg->s_pCamCfg->s_sCamProcessStatus;
    m_jSocketReturn["Camera_Status"] = m_jCameraStatusValue;



}

std::string geosun::SocketClient::Base64Encode(const char* bytes, unsigned int length)
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

std::string geosun::SocketClient::Base64Decode(const std::string& encode_str)
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
    std::string imgPath = m_sPrjPath + "Image\\";
    if (IsDir(imgPath))
    {
        std::vector<std::string>imgFileNames = GetFileNamesFromDir(imgPath);
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
                std::ifstream in(imgfile, std::ifstream::in | std::ios::binary);
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

void geosun::SocketClient::ProcessClent()
{
    CreateSocket();
    while (1)
    {
        std::cout << "1" << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
}

void geosun::SocketClient::CreateSocket()
{
#if WIN
    WORD sockVersion = MAKEWORD(2, 2);
    WSADATA wsaData;
    if (WSAStartup(sockVersion, &wsaData) != 0)
    {
        return ;
    }
#else
#endif
    m_pServerSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (m_pServerSocket <= 0)
    {
        LOG_INFO("Create Socket Error!");
        return;
    }
    ;
    m_sAddress.sin_family = AF_INET;
    m_sAddress.sin_port = htons(8888);
    m_sAddress.sin_addr.S_un.S_addr = INADDR_ANY;
    if (bind(m_pServerSocket, (LPSOCKADDR)&m_sAddress, sizeof(m_sAddress)) == SOCKET_ERROR)
    {
        LOG_INFO("Bind Error!");
        return;
    }
    if (listen(m_pServerSocket, 5) == SOCKET_ERROR)
    {
        LOG_INFO("Listen Error!");
        return ;
    }
    int nAddrlen = sizeof(m_sClientAddress);
    SOCKET sClient;
    char revData[255];
    while (true)
    {
        printf("等待连接...\n");
        sClient = accept(m_pServerSocket, (SOCKADDR*)&m_sClientAddress, &nAddrlen);
        if (sClient <= 0)
        {
            LOG_INFO("Accept Error!");
            continue;
        }
        LOG_INFO("Receive One Connect：%s \r\n", inet_ntoa(m_sClientAddress.sin_addr));
        int ret = recv(sClient, revData, 255, 0);
        if (ret > 0)
        {
            revData[ret] = 0x00;
            std::string tmp = revData;
            std::cout << tmp << std::endl;
            if (tmp == "Lidar")
            {
                std::cout << "aaa" << std::endl;
                m_pLidarSocket = sClient;
                m_pProcessLidar = new boost::thread(boost::bind(&SocketClient::ProcessLidar, this));
            }
            if (tmp == "Ag310")
            {
                m_pAg310Socket = sClient;
                m_pProcessAg310 = new boost::thread(boost::bind(&SocketClient::ProcessAg310, this));
            }
        }
    }
}

void geosun::SocketClient::ProcessLidar()
{
    int sendData = 1;
    SendSocketData(m_pLidarSocket, std::to_string(sendData).c_str());
    while (true)
    {
        char buf[64];
        if (ReceiveSocketData(m_pLidarSocket, buf))
        {
            printf("Receive Lidar:%s\n", buf);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
    }
    
}

void geosun::SocketClient::ProcessAg310()
{
    int sendData = 10000;
    sendData--;
    SendSocketData(m_pAg310Socket, std::to_string(sendData).c_str());
    while (1)
    {
        char buf[64];
        if (ReceiveSocketData(m_pAg310Socket, buf))
        {
            printf("Receive Ag310:%s\n", buf);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}

bool geosun::SocketClient::ReceiveSocketData(SOCKET socketIn, char* buf)
{
    int retVal = recv(socketIn, buf, 255, 0);
    if (retVal <= 0)
    {
        LOG_INFO("Receive Fail!");
        return false;
    }
    return true;
}

bool geosun::SocketClient::SendSocketData(SOCKET socketIn, const char* buf)
{
    int retVal = send(socketIn, buf, strlen(buf), 0);
    if (retVal <= 0)
    {
        LOG_INFO("SendData Fail!");
        return false;
    }
    return true;
}

