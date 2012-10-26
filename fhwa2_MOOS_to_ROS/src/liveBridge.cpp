/*
    Define the functions to populate ros msgs with moos info for sending to the 
    python script
*/
#include <fhwa2_MOOS_to_ROS/liveBridge.h>

using namespace liveBridge;

void MOOS2RVIZ::onInit() {
    // m_Comms.setOnConnectCallback(MOOS2RVIZ::onConnectToServer);
    // m_Comms.setOnMailCallback(MOOS2RVIZ::onNewMail);
    return;
}

bool MOOS2RVIZ::onConnectToServer() {
    bool registered = true;
    // populate desired variables
    // register desired variables
    std::cout << "In MOOS2RVIZ::onConnectToServer\n";
    return registered;
}

bool MOOS2RVIZ::onNewMail(MOOSMSG_LIST &NewMail) {
    MOOSMSG_LIST::iterator p;
    for (p=NewMail.begin(); p!=NewMail.end(); p++) {
        CMOOSMsg &rMsg = *p;
        // send no matter what
        this->rospub.publish(this->handleMsg(rMsg));
    }
    std::cout << "In MOOS2RVIZ::onNewMail\n";
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
