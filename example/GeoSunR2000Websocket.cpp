#include "WebSocketServer.h"
#include "EventLoop.h"
#include "htime.h"
#include "json.hpp"
#include "general_client/General_Client.h"
#include "boost/function.hpp"
#include "boost/bind.hpp"

using namespace geosun;
using namespace hv;

void a(int b)
{
}

void Start(boost::function<void(LidarOptions::Ptr)> lidar_callback)
{
    LidarOptions::Ptr temp = std::make_shared<LidarOptions> ();
    while (true)
    {
        temp->s_dLidHeight++;
        lidar_callback(temp);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        
        
    }
}

int main(int argc, char** argv) {


    SocketClient::Ptr pSocketClient = std::make_shared<SocketClient>();
    pSocketClient->Run();
    Start(boost::bind(&SocketClient::LidMessageCallback, pSocketClient, _1));
    
    return 0;
}
