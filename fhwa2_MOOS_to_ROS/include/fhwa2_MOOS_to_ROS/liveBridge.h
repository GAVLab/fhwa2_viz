/*
    In contrast to William's broad approach, we only want to bridge a 
    pre-determined set of MOOS variables, then broadcast them as ROS messages
    in a very generic format.

    The idea is to obtain data from MOOS and do as much of the handling as 
    possible within the ROS ecosystem, that is, do not specify what type of ROS
    message will correspond until actually received by ROS.

    This setup implies a pure ROS node will receive these messages and direct
    them from there.

    Initialization: William's app handled startup as if moos2ros was core 
    component. Treat both worlds as if they are preexisting.

    Author: Robert Cofield
*/
#ifndef MOOS_ROS_BRIDGE_H
#define MOOS_ROS_BRIDGE_H

#include <iostream>

// MOOS Headers
#include <MOOSLIB/MOOSCommClient.h>
#include <MOOSLIB/MOOSMsg.h>
#include <MOOSLIB/MOOSApp.h>
#include <MOOSGenLib/MOOSGenLibGlobalHelper.h>
#include <MOOSGenLib/MOOSAssert.h>


// ROS Headers
#include <ros/ros.h>
#include <fhwa2_MOOS_to_ROS/MOOSrosmsg.h>

// namespace liveBridge {

// typedef fhwa2_MOOS_to_ROS::MOOSrosmsg MoosRosMsg;

class MOOS2ROS : public CMOOSApp {
public:
    MOOS2ROS();
    virtual ~MOOS2ROS();

    bool OnConnectToServer();
    void GetDesiredVaribles();
    bool OnDisconnectFromServer();
    bool Iterate();
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    fhwa2_MOOS_to_ROS::MOOSrosmsg handleMsg(CMOOSMsg &Msg);

    std::vector<std::string> desired_variables;
    float min_upd;
    ros::Publisher rospub;
    ros::NodeHandle node_handle;
    // virtual bool onConnectToServer(void * pParam);

}; // end class

// }; // end namespace

#endif