#include <yarp/os/Network.h>
#include <idynutils/yarp_single_chain_interface.h>
#include <idynutils/yarp_ft_interface.h>
#include <idynutils/yarp_IMU_interface.h>

using namespace walkman;
// TODO transform in unit test
// TODO check reading from interfaces is "fast"
//TEST_F(testInterface, checkTimings)
//{

//}
// TODO check that readings on interfaces when no data is yet available
//      result in the previous reading
//TEST_F(testInterface, checkReadingsAreContinuous)
//{

//}
int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
        return 0;}
        yarp.init();
 
    yarp_ft_interface d("right_arm","test_module","coman");
    yarp_single_chain_interface e("right_arm","test_module","coman");

    yarp_IMU_interface imu("test_module","");

    yarp::sig::Vector imuData = imu.sense();
    std::cout << "IMU data reading:"
              << std::endl
              << imuData.toString() << std::endl;
        
}
