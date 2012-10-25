/*
    Runs Moos as a ros node
*/
#include <fhwa2_MOOS_to_ROS/liveBridge.h>

using namespace liveBridge;

MOOS2RVIZ app;

int main(int argc, char** argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    
    app.rospub = nh.advertise<fhwa2_MOOS_to_ROS::MOOSrosmsg>("/moos/incoming", 1);
    app.SetOnConnectCallback(app.onConnect, void, param);
    app.SetOnMailCallBack(app.onMail, void param);

    // app.Run(.....) // else call in thread (boost)
    ros::spin();
    return 0;
}