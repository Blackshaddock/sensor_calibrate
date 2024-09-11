#include "general_client/Log.h"
#include "general_client/General_Client.h"
#include "boost/function.hpp"
#include "boost/bind.hpp"
#include <winsock2.h>

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
