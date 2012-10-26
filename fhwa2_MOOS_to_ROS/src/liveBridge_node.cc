/*
    Runs Moos as a ros node
*/
#include <fhwa2_MOOS_to_ROS/liveBridge.h>

using namespace liveBridge;

MOOS2RVIZ app;

int main(int argc, char** argv) {
    ros::init(argc, argv, "talker");
    std::cout << "liveBridge_cpp init\n";
    ros::NodeHandle nh;
    
    app.rospub = nh.advertise<fhwa2_MOOS_to_ROS::MOOSrosmsg>("/moos/incoming", 1);
    // app.m_Comms.SetOnConnectCallback(app.onConnectToServer);
    // app.m_Comms.SetOnMailCallBack(app.onNewMail);
    // app.onInit();

    // app.Run(.....) // else call in thread (boost)
    ros::spin();
    return 0;
}