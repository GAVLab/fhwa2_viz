/*
    asdf

    Author: Robert Cofield
*/
#ifndef MOOS_ROS_BRIDGE_H
#define MOOS_ROS_BRIDGE_H

#include <iostream>

// MOOS Headers
#include <MOOSLIB/MOOSCommClient.h>
#include <MOOSLIB/MOOSMsg.h>
#include <MOOSLIB/MOOSApp.h>

// ROS Headers
#include <ros/ros.h>
#include <fhwa2_MOOS_to_ROS/MOOSrosmsg.h>

namespace liveBridge{

typedef fhwa2_MOOS_to_ROS::MOOSrosmsg MoosRosMsg;

class MOOS2RVIZ: public CMOOSApp { // change to MOOSApp
public:
    MOOS2RVIZ(){};
    ~MOOS2RVIZ(){};
    std::vector<std::string> desired_variables;
    ros::Publisher rospub;
    void onInit();
    bool onConnectToServer();
    bool onNewMail(MOOSMSG_LIST &NewMail);

private:
    // Puts new mail info into a ros version of the MOOS msg and publishes
    MoosRosMsg handleMsg(CMOOSMsg &Msg);

}; // end class

}; // end namespace

#endif