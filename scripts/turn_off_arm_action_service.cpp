#include <unitree/robot/b2/robot_state/robot_state_client.hpp>
#include <unitree/common/time/time_tool.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

int main(int32_t argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: turn_off_arm_example [NetWorkInterface(eth0)]" << std::endl;
        exit(0);
    }

    std::string networkInterface = "eth0", serviceName = "g1_arm_example";

    if (argc > 1)
    {
        networkInterface = argv[1];
    }

    std::cout << "NetWorkInterface:" << networkInterface << std::endl;
    std::cout << "Switch ServiceName:" << serviceName << std::endl; 

    ChannelFactory::Instance()->Init(0, networkInterface);

    RobotStateClient rsc;
    rsc.SetTimeout(10.0f);
    rsc.Init();

    std::string clientApiVersion = rsc.GetApiVersion();
    std::string serverApiVersion = rsc.GetServerApiVersion();

    if (clientApiVersion != serverApiVersion)
    {
        std::cout << "client and server api versions are not equal." << std::endl;
    }
    int status = 0;
    while (1)
    {
        int ret = rsc.ServiceSwitch(serviceName, 0, status);
        if (ret == 0 && status != 0)
        {
            std::cout << "Call ServiceSwitch[" << serviceName << ",0] ret:" << ret << ", status:" << status << std::endl;
            break;
        }
    }
    ChannelFactory::Instance()->Release();
    return 0;
}