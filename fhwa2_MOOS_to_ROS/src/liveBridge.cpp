/*
    Define the functions to populate ros msgs with moos info for sending to the 
    python script
*/
#include <fhwa2_MOOS_to_ROS/liveBridge.h>

using namespace liveBridge;

bool MOOS2RVIZ::onConnect() {
    bool registered = true;
    // populate desired variables
    // register desired variables
    return registered;
}

bool MOOS2RVIZ::onMail(MOOSMSG_LIST &NewMail) {
    MOOSMSG_LIST::iterator p;
    for (p=NewMail.begin(); p!=NewMail.end(); p++) {
        CMOOSMsg &rMsg = *p;
        // send no matter what
        this->rospub.publish(this->handleMsg(rMsg));
    }
    return true;
}

MoosRosMsg MOOS2RVIZ::handleMsg(CMOOSMsg &Msg) {
    // Create ros msg to send
    fhwa2_MOOS_to_ROS::MOOSrosmsg rosmsg;
    rosmsg.header.stamp = ros::Time::now();
    rosmsg.header.frame_id = std::string("change_me");

    // Populate the rosmsg with data from the MOOS msg
    rosmsg.MOOStime = Msg.GetTime();
    rosmsg.MOOSname = Msg.GetName();
    rosmsg.MOOSsource = Msg.GetSource();
    if (Msg.IsDouble()) {
        rosmsg.MOOStype = std::string("Double");
        rosmsg.MOOSdouble = Msg.GetDouble();
    } else { //Make an assumption
        rosmsg.MOOStype = std::string("String");
        rosmsg.MOOSstring = Msg.GetString();
    }

    return rosmsg;
}
