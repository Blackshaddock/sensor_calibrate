#include "General_Client.h"
#include <fstream>
using namespace geosun;



SocketClient::SocketClient()
{
    
	m_pConf = std::make_shared<SocketClientConfig>();
    m_pSensorCfg = std::make_shared<SensorOptions>();
#if WIN
    m_sRootPath = "E:\\data\\RTK\\RTK\\";
    Log::getInstance().setLogFilePath("D:\\code\\01.sensor_calibrate\\build\\app_debug.log");
    m_jSensorUseValue = GetJsonFromFile("C:\\Users\\admin\\Desktop\\App\\configuresensor.json");
#else
    m_sRootPath = "/mnt/sd/";
    Log::getInstance().setLogFilePath("/mnt/sd/app_debug.log");
    m_jSensorUseValue = GetJsonFromFile("/mnt/cfg/configuresensor.json");
#endif
    m_bStartFlag = false;
    m_bReturnFlag = false;
    m_sVerPath = "";
    m_pProcessClent = nullptr;
    m_pProcessPos = nullptr;
    m_pProcessLidar = nullptr;
    Log::getInstance().setLogLevel(INFO);
    Log::getInstance().setPrintToTerminal(true);
    Log::getInstance().setMaxLogFileSize(5 * 1024 * 1024); // 5KB for testing
    Log::getInstance().setMaxBackupFiles(5);
    Log::getInstance().setQueueMaxSize(100000);
    
    Log::getInstance().enablePerformanceMonitoring(true);
    
    if (m_jSensorUseValue.empty())
    {
        LOG_INFO("Read the Config In Device Empty!");
    }
    else {
        
        m_pSensorCfg->s_pCamCfg->s_dCamFps = m_jSensorUseValue["Camera_Params"]["time_bias"];
        m_pSensorCfg->s_pLidCfg->s_dLidFovMin =     m_jSensorUseValue["Riegl_Params"]["scanline_start"];
        m_pSensorCfg->s_pLidCfg->s_dLidFovMax =     m_jSensorUseValue["Riegl_Params"]["scanline_stop"];
        m_pSensorCfg->s_pLidCfg->s_dAngResolution = m_jSensorUseValue["Riegl_Params"]["scanline_increment"];
        m_pSensorCfg->s_pLidCfg->s_iLidFps = m_jSensorUseValue["Riegl_Params"]["meas_prr"];
        m_pSensorCfg->s_pLidCfg->s_dLidHeight =     m_jSensorUseValue["Riegl_Params"]["lidar_height"];
        m_pSensorCfg->s_pLidCfg->s_dLidSpeed =      m_jSensorUseValue["Riegl_Params"]["speed_rate"];
    }

}

SocketClient::SocketClient(SocketClientConfig::Ptr config) : m_pConf(config) {
    m_pSensorCfg = std::make_shared<SensorOptions>();
    m_sRootPath = "E:\\data\\RTK\\RTK\\";
    m_bStartFlag = false;
    m_bReturnFlag = false;
    m_sVerPath = "";
    m_pProcessClent = nullptr;
    m_pProcessPos = nullptr;
    m_pProcessLidar = nullptr;
    m_jSensorUseValue = GetJsonFromFile("C:\\Users\\admin\\Desktop\\App\\configuresensor.json");
    if (m_jSensorUseValue.empty())
    {
        LOG_INFO("Read the Config In Device Empty!");
    }
    else {
        LOG_INFO("Read the Config In Device Empty!");
        m_pSensorCfg->s_pCamCfg->s_dCamFps =        m_jSensorUseValue["Camera_Params"]["time_bias"];
        m_pSensorCfg->s_pLidCfg->s_dLidFovMin =     m_jSensorUseValue["Riegl_Params"]["scanline_start"];
        m_pSensorCfg->s_pLidCfg->s_dLidFovMax =     m_jSensorUseValue["Riegl_Params"]["scanline_stop"];
        m_pSensorCfg->s_pLidCfg->s_dAngResolution = m_jSensorUseValue["Riegl_Params"]["scanline_increment"];
        m_pSensorCfg->s_pLidCfg->s_iLidFps =        m_jSensorUseValue["Riegl_Params"]["meas_prr"];
        m_pSensorCfg->s_pLidCfg->s_dLidHeight =     m_jSensorUseValue["Riegl_Params"]["lidar_height"];
        m_pSensorCfg->s_pLidCfg->s_dLidSpeed = m_pSensorCfg->s_pLidCfg->s_dAngResolution * m_pSensorCfg->s_pLidCfg->s_iLidFps / 360.;
    }
}

bool geosun::SocketClient::Run()
{
    m_wWs.onopen = [&](const WebSocketChannelPtr& channel, const HttpRequestPtr& req) {
        printf("onopen: GET %s\n", req->Path().c_str());
        // send(time) every 1s
        try {
            setInterval(1000, [&](TimerID id)
                {

                    if (channel->isConnected()) {
                        char str[DATETIME_FMT_BUFLEN] = { 0 };
                        datetime_t dt = datetime_now();
                        datetime_fmt(&dt, str);
                        ComputerSize();
                        CompressMessage();
                        std::cout << m_jSocketReturn.dump() << std::endl;
                        channel->send(m_jSocketReturn.dump());
                    }
                    else {
                        killTimer(id);
                    }
                }

            );
        }
        catch (std::exception& e)
        {
            LOG_INFO("GeoSun Socket Test %s", e.what());
        }
        catch (...)
        {
            LOG_INFO("GeoSun Socket Unknow Error ");
        }
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
                    fd.close();
                }
                if (IsExists(m_sDeviceSN))
                {
                    std::fstream fd(m_sDeviceSN);
                    std::getline(fd, m_pSensorCfg->s_pComputerCfg->s_sDeviceSN);
                    m_jReturnValue["device_sn"] = m_pSensorCfg->s_pComputerCfg->s_sDeviceSN;
                    fd.close();
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
                m_jReturnValue["disconnect_status"] = true;
            }
            catch (std::exception& e)
            {
                LOG_INFO("API GeoSun DisConnect %s", e.what());
                m_jReturnValue["status"] = false;
                m_jReturnValue["disconnect_status"] = false;
            }
            catch (...)
            {
                LOG_INFO("API GeoSun DisConnect Unknow error ");
                m_jReturnValue["status"] = false;
                m_jReturnValue["disconnect_status"] = false;
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
                LOG_INFO("Receive App Cmd Start And Stop: %s\n", m_jReceiveValue.dump());
                m_jReturnValue["status"] = true;
                if (m_jReceiveValue["input_status"] == "start")
                {
                    if (m_jReceiveValue["prj_name"] == "")
                    {
                        std::time_t now = std::time(0);
                        std::tm* ltm = std::localtime(&now);
                        char buf[256];
                        sprintf(buf, "%d%02d%02d%02d%02d%02d", ltm->tm_year + 1900, ltm->tm_mon + 1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
                        m_pSensorCfg->s_pComputerCfg->s_sPrjName = buf;
                    }
                    else {
                        m_pSensorCfg->s_pComputerCfg->s_sPrjName = m_jReceiveValue["prj_name"];
                    }
                    std::ofstream fin;
#if WIN
                    fin.open("E:\\prj_name");
#else
                    fin.open("/mnt/cfg/prj_name.txt");
#endif
                    if (!fin.is_open())
                    {
                        LOG_INFO("Can Open The Prj_Name Path!!!");
                        m_jReturnValue["prjname_flag"] = false;
                        return ctx->send(m_jReturnValue.dump());
                    }
                    m_jReturnValue["prjname_flag"] = true;
                    fin << m_pSensorCfg->s_pComputerCfg->s_sPrjName << std::endl;
                    fin.close();
                    m_sPrjPath = m_sRootPath + m_pSensorCfg->s_pComputerCfg->s_sPrjName;
                    SendSocketData(m_pLidarSocket, std::string(m_jReceiveValue["input_status"]).c_str());
                    SendSocketData(m_pPosSocket, std::string(m_jReceiveValue["input_status"]).c_str());
                    //WaitForReturn();
                    return ctx->send(m_jReturnValue.dump());
                }
                else if (m_jReceiveValue["input_status"] == "stop")
                {
                    SendSocketData(m_pLidarSocket, std::string(m_jReceiveValue["input_status"]).c_str());
                    SendSocketData(m_pPosSocket, std::string(m_jReceiveValue["input_status"]).c_str());
                    //WaitForReturn();
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
            m_jReturnValue["status"] = true;
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
                    m_jReturnValue["camera_fps"] = m_pSensorCfg->s_pCamCfg->s_dCamFps;
                }
                else if (m_jReceiveValue["input_status"] == "set")
                {
                    std::string confPath;
#if WIN
                    confPath = "C:\\Users\\admin\\Desktop\\App\\configuresensor.json";                
#else
                    confPath = "/mnt/cfg/configuresensor.json";
#endif
                    Json tmp = GetJsonFromFile(confPath);
                    tmp["Camera_Params"]["time_bias"] = m_jReceiveValue["camera_fps"];
                    SaveJson2File(tmp, "C:\\Users\\admin\\Desktop\\App\\configuresensor.json");
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
                m_jReturnValue["status"] = true;
                Json j;
                if (m_jReceiveValue["input_status"] == "get")
                {
                    std::string lidParamsPath;
#if WIN 
                    lidParamsPath = "C:\\Users\\admin\\Desktop\\App\\configurelidar.json";
#else 
                    lidParamsPath = "/mnt/cfg/configurelidar.json";
#endif
                    if (IsExists(lidParamsPath))
                    {
                        m_jReturnValue = GetJsonFromFile(lidParamsPath);
                        if (m_jReturnValue.empty())
                            m_jReturnValue["config_flag"] = false;
                        m_jReturnValue["config_flag"] = true;
                    }
                    else
                    {
                        m_jReturnValue["config_flag"] = false;
                    }
                    m_jReturnValue["status"] = true;
                    //获取当前的激光器的设置结果，并返回
                }
                else if (m_jReceiveValue["input_status"] == "set")
                {
                    std::string sensorParamsPath;
#if WIN
                    sensorParamsPath = "C:\\Users\\admin\\Desktop\\App\\configuresensor.json";
#else
                    sensorParamsPath = "/mnt/cfg/configuresensor.json";
#endif
                    Json tmp = GetJsonFromFile(sensorParamsPath);
                    if (tmp.empty())
                    {
                        LOG_INFO("Read Json File %s Empty: %s\n", "/mnt/cfg/configuresensor.json", "GeoSun_LidarParams set");
                        m_jReturnValue["config_flag"] = false;
                        
                    }
                    else {
                        tmp["Riegl_Params"]["lidar_height"] = m_jReceiveValue["data"]["lidar_height"];
                        tmp["Riegl_Params"]["meas_prr"] = m_jReceiveValue["data"]["meas_prr"];
                        tmp["Riegl_Params"]["scanline_increment"] = m_jReceiveValue["data"]["scanline_increment"];
                        tmp["Riegl_Params"]["scanline_start"] = m_jReceiveValue["data"]["scanline_start"];
                        tmp["Riegl_Params"]["scanline_stop"] = m_jReceiveValue["data"]["scanline_stop"];


                        m_pSensorCfg->s_pLidCfg->s_dLidFovMin = tmp["Riegl_Params"]["scanline_start"];
                        m_pSensorCfg->s_pLidCfg->s_dLidFovMax = tmp["Riegl_Params"]["scanline_stop"];
                        m_pSensorCfg->s_pLidCfg->s_dAngResolution = tmp["Riegl_Params"]["scanline_increment"];
                        m_pSensorCfg->s_pLidCfg->s_iLidFps = tmp["Riegl_Params"]["meas_prr"];
                        m_pSensorCfg->s_pLidCfg->s_dLidHeight = tmp["Riegl_Params"]["lidar_height"];
                        m_pSensorCfg->s_pLidCfg->s_dLidSpeed = m_pSensorCfg->s_pLidCfg->s_dAngResolution * m_pSensorCfg->s_pLidCfg->s_iLidFps / 360.;
                        tmp["Riegl_Params"]["speed_rate"] = m_pSensorCfg->s_pLidCfg->s_dLidSpeed;
                        m_jReturnValue["config_flag"] = true;
                        SaveJson2File(tmp, sensorParamsPath);
                        m_jReturnValue["Lidar_Status"] = m_jReceiveValue["data"];
                        m_jReturnValue["speed_rate"] = m_pSensorCfg->s_pLidCfg->s_dLidSpeed;
                        
                    }
                }
                else if (m_jReceiveValue["input_status"] == "get_add")
                {
                    std::string customLidarPath; 
#if WIN
                    customLidarPath = "C:\\Users\\admin\\Desktop\\App\\customlidar.json";
#else
                    customLidarPath = "/mnt/cfg/customlidar.json";
#endif
                    //添加自定义的参数，并需要将该自定义参数保存为一个表
                    Json tmp = GetJsonFromFile(customLidarPath);
                    if (!tmp.empty())
                    {
                        //m_jReturnValue["speed_rate"] = double(tmp["scanline_increment"]) * int(tmp["meas_prr"]) / 360.;
                        m_jReturnValue = tmp;
                        m_jReturnValue["config_flag"] = true;   
                    }
                    else {
                        m_jReturnValue["config_flag"] = false;
                    }
                    m_jReturnValue["status"] = true;
                }
                else if(m_jReceiveValue["input_status"] == "set_add")
                {
                    std::string customLidarPath;
#if WIN
                    customLidarPath = "C:\\Users\\admin\\Desktop\\App\\customlidar.json";
#else
                    customLidarPath = "/mnt/cfg/customlidar.json";
#endif
                    Json tmp, write; // = GetJsonFromFile(customLidarPath);
                    m_jReturnValue["config_flag"] = true;
                    tmp["scanline_increment"] = m_jReceiveValue["data"]["scanline_increment"];
                    tmp["meas_prr"] = m_jReceiveValue["data"]["meas_prr"];
                    tmp["lidar_height"] = m_jReceiveValue["data"]["lidar_height"];
                    write["data"].push_back(tmp);
                    SaveJson2File(write, customLidarPath);
                    tmp.clear();
                    std::string sensorParamsPath;
#if WIN
                    sensorParamsPath = "C:\\Users\\admin\\Desktop\\App\\configuresensor.json";
#else
                    sensorParamsPath = "/mnt/cfg/configuresensor.cfg";
#endif
                    tmp = GetJsonFromFile(sensorParamsPath);
                    tmp["Riegl_Params"]["lidar_height"] = m_jReceiveValue["data"]["lidar_height"];
                    tmp["Riegl_Params"]["meas_prr"] = m_jReceiveValue["data"]["meas_prr"];
                    tmp["Riegl_Params"]["scanline_increment"] = m_jReceiveValue["data"]["scanline_increment"];
                    tmp["Riegl_Params"]["scanline_start"] = m_jReceiveValue["data"]["scanline_start"];
                    tmp["Riegl_Params"]["scanline_stop"] = m_jReceiveValue["data"]["scanline_stop"];
                    m_pSensorCfg->s_pLidCfg->s_dLidFovMin = tmp["Riegl_Params"]["scanline_start"];
                    m_pSensorCfg->s_pLidCfg->s_dLidFovMax = tmp["Riegl_Params"]["scanline_stop"];
                    m_pSensorCfg->s_pLidCfg->s_dAngResolution = tmp["Riegl_Params"]["scanline_increment"];
                    m_pSensorCfg->s_pLidCfg->s_iLidFps = tmp["Riegl_Params"]["meas_prr"];
                    m_pSensorCfg->s_pLidCfg->s_dLidHeight = tmp["Riegl_Params"]["lidar_height"];
                    tmp["Riegl_Params"]["speed_rate"] = m_pSensorCfg->s_pLidCfg->s_dLidSpeed;
                    SaveJson2File(tmp, sensorParamsPath);
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
            std::cout << "===" << m_jReturnValue.dump() << std::endl;
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
    //m_pSensorCfg->s_pLidCfg = lidcfg;
    m_mSensorMutex.unlock();
}

void geosun::SocketClient::ComputerSize()
{

    bfs::space_info si = DiskInfo(m_sRootPath);
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
    //std::cout << m_jSocketReturn["Sensor_Status"].dump() << std::endl;
    //Computer Status
    m_jComputerStatusValue["computer_totalspace"] = m_pSensorCfg->s_pComputerCfg->s_sTotalSize;
    m_jComputerStatusValue["remain_space"] = m_pSensorCfg->s_pComputerCfg->s_sRemainSize;
    m_jComputerStatusValue["global_status"] = m_pSensorCfg->s_pComputerCfg->s_sGlobalStatus;
    m_jComputerStatusValue["project_name"] = m_pSensorCfg->s_pComputerCfg->s_sPrjName;
    m_jComputerStatusValue["record_time"] = m_pSensorCfg->s_pComputerCfg->s_iRecordTime;
    m_jComputerStatusValue["pic_num"] = m_pSensorCfg->s_pComputerCfg->s_iPicNum;
    m_jComputerStatusValue["lid_num"] = m_pSensorCfg->s_pComputerCfg->s_iLidNum;
    m_jComputerStatusValue["dat_size"] = m_pSensorCfg->s_pComputerCfg->s_sDatSize;
    m_jComputerStatusValue["star_num"] = m_pSensorCfg->s_pGpsCfg->s_iGpsNum;
    m_jSocketReturn["Computer_Status"] = m_jComputerStatusValue;
    //std::cout << m_jSocketReturn["Computer_Status"].dump() << std::endl;
    //Lidar Status 预留由于在雷达的参数设置中已经存在这些配置
    m_jLidStatusValue["meas_prr"] = m_pSensorCfg->s_pLidCfg->s_iLidFps;
    m_jLidStatusValue["lidar_height"] = m_pSensorCfg->s_pLidCfg->s_dLidHeight;
    m_jLidStatusValue["scanline_start"] = m_pSensorCfg->s_pLidCfg->s_dLidFovMin;
    m_jLidStatusValue["scanline_stop"] = m_pSensorCfg->s_pLidCfg->s_dLidFovMax;
    m_jLidStatusValue["speed_rate"] = m_pSensorCfg->s_pLidCfg->s_dLidSpeed;
    m_jLidStatusValue["scanline_increment"] = m_pSensorCfg->s_pLidCfg->s_dAngResolution;
    m_jSocketReturn["Lidar_Status"] = m_jLidStatusValue;
    //std::cout << m_jSocketReturn["Lidar_Status"].dump() << std::endl;
    //Gps Status
    
    m_jGpsStatusValue["sync_status"] = m_pSensorCfg->s_pGpsCfg->s_sGpsSyncStatus;
    m_jSocketReturn["Gps_Status"] = m_jGpsStatusValue;
    //std::cout << m_jSocketReturn["Gps_Status"].dump() << std::endl;
    //Camera Status
    
    m_jCameraStatusValue["camera_fps"] = m_pSensorCfg->s_pCamCfg->s_dCamFps;
    m_jCameraStatusValue["camera_process"] = m_pSensorCfg->s_pCamCfg->s_sCamProcessStatus;
    m_jSocketReturn["Camera_Status"] = m_jCameraStatusValue;
    //std::cout << m_jSocketReturn["Camera_Status"].dump() << std::endl;


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
   # if WIN
    m_sAddress.sin_addr.S_un.S_addr = INADDR_ANY;
    #else
    m_sAddress.sin_addr.s_addr = INADDR_ANY;
    #endif
    if (bind(m_pServerSocket, (struct sockaddr *)&m_sAddress, sizeof(m_sAddress)) <0)
    {
        LOG_INFO("Bind Error!");
        return;
    }
    if (listen(m_pServerSocket, 5) <0)
    {
        LOG_INFO("Listen Error!");
        return ;
    }
    int nAddrlen = sizeof(m_sClientAddress);
    SOCKET sClient;
    char revData[255];
    while (true)
    {
        printf("wait connect...\n");
		#if WIN
        sClient = accept(m_pServerSocket, (SOCKADDR*)&m_sClientAddress, &nAddrlen);
		#else
		sClient = accept(m_pServerSocket, (struct sockaddr *)&m_sClientAddress, (socklen_t*)&nAddrlen);
		#endif
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
            //FilterReceiveData(tmp);
            std::cout << tmp << std::endl;
            if (tmp == "Lidar")
            {
                m_pLidarSocket = sClient;
                m_pProcessLidar = new boost::thread(boost::bind(&SocketClient::ProcessLidar, this));
            }
            if (tmp == "Pos")
            {
                m_pPosSocket = sClient;
                m_pProcessPos = new boost::thread(boost::bind(&SocketClient::ProcessPos, this));
            }
        }
    }
}

void geosun::SocketClient::ProcessPos()
{
    int sendData = 1;
    //std::string sendstr = m_pConf->s_sDataHeader + std::to_string(sendData);
    SendSocketData(m_pPosSocket, AddDataHeader(std::to_string(sendData)).c_str());
    while (true)
    {
        char buf[256];
        if (ReceiveSocketData(m_pPosSocket, buf))
        {
            printf("Receive Lidar:%s\n", buf);
        }
        else {
            LOG_INFO("Pos Socket DisConnect!!!");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
    }
    
}

void geosun::SocketClient::ProcessLidar()
{
    int sendData = 10000;
    sendData--;
    SendSocketData(m_pLidarSocket, std::to_string(sendData).c_str());
    char buf[256] = {0};
    while (1)
    {
        memset(buf, 0, sizeof(buf));
        if (ReceiveSocketData(m_pLidarSocket, buf))
        {
            //printf("Receive Pos:%s\n", buf);
            std::string tmpRecv = buf;
            if (tmpRecv == "123")
            {
                m_jReturnValue["start_flag"] = true;
                m_bReturnFlag = true;
                continue;
            }
            Json receiveLidar = Json::parse(tmpRecv);
            if (receiveLidar["process_type"] == "lidar_init")
            {
                LOG_INFO("Receive Lidar Init: %s\n", buf);
                m_pSensorCfg->s_pLidCfg->s_sLidStatus = "Ready";
            }
            else if (receiveLidar["process_type"] == "start_feedback")
            {
                LOG_INFO("Receive Start Callback: %s\n", buf);
                m_jReturnValue["start_flag"] = true;
                m_pSensorCfg->s_pComputerCfg->s_sGlobalStatus = "Recording";
            }
            else if (receiveLidar["process_type"] == "stop_feedback")
            {
                LOG_INFO("Receive Stop Callback: %s\n", buf);
                m_jReturnValue["stop_flag"] = true;
                m_pSensorCfg->s_pComputerCfg->s_sGlobalStatus = "Stopping";
            }
            else if (receiveLidar["process_type"] == "camera_callback")
            {
                LOG_INFO("Receive Camera Callback: %s\n", buf);
                m_jReturnValue["camera_flag"] = true;
            }
            else if (receiveLidar["process_type"] == "laser_callback")
            {
                LOG_INFO("Receive Lidar Callback: %s\n", buf);
                m_jReturnValue["laser_flag"] = true;
            }
        }
        else {
            LOG_INFO("Lidar Socket DisConnect!!!");
            break;
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

void geosun::SocketClient::FilterReceiveData(std::string &strInOut, int type)
{
        int length = strInOut.size();
        if ((strInOut.substr(0, 6) == "ffffff") && (strInOut.substr(length-2, 2 ) == "ff"))
        {
            strInOut = strInOut.substr(6, length - 8);
        }
        else {
            LOG_INFO("Receive Data Is Error! %s\n", strInOut);
            strInOut = "";
        }
    
    
}



bool geosun::SocketClient::SendSocketData(SOCKET socketIn, const char* buf)
{
    LOG_INFO("SendData to Server:%s\n", buf);
    int retVal = send(socketIn, buf, strlen(buf), 0);
    if (retVal <= 0)
    {
        LOG_INFO("SendData %sFail!\n", buf);
        return false;
    }
    return true;
}

