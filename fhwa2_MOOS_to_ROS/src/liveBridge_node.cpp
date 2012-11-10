/*
    Runs Moos as a ros node
*/
#include <fhwa2_MOOS_to_ROS/liveBridge.h>
#include <vector>
#include <string>
#include <sstream>

std::string namename;

MOOS2ROS::MOOS2ROS() {
    this->node_handle = ros::NodeHandle("MOOS");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // this->SetQuitOnFailedIterate(true);

    // MOOSTrace("MOOS Name before being set: %s", m_sMOOSName);

    m_sAppName = namename;
    m_sMOOSName = namename; // May cause problems
    m_MissionReader.SetAppName(namename);

    MOOSTrace("\nMOOS2ROS class initialized with App Name: %s \n\n", GetAppName().c_str());
}

MOOS2ROS::~MOOS2ROS() {
    ros::shutdown();
    this->RequestQuit();
}

bool MOOS2ROS::OnConnectToServer() {
    GetDesiredVaribles();

    // Subscriptions 
    std::vector<std::string>::iterator p;
    for (p=this->desired_variables.begin(); 
         p!=this->desired_variables.end(); p++) {
        m_Comms.Register(*p, this->min_upd);
    }
    return true;
}

void MOOS2ROS::GetDesiredVaribles() {
    // Get pre-defined variables (MOOS params)
    bool got_ = m_MissionReader.GetConfigurationParam("min_upd", this->min_upd);

    // Subscriptions
    this->desired_variables.push_back("zX");
    this->desired_variables.push_back("zY");
    this->desired_variables.push_back("zZ");
    this->desired_variables.push_back("zXStdDev");
    this->desired_variables.push_back("zYStdDev");
    this->desired_variables.push_back("zZStdDev");
    this->desired_variables.push_back("zpsrX");
    this->desired_variables.push_back("zpsrY");
    this->desired_variables.push_back("zpsrZ");
    this->desired_variables.push_back("zpsrXStdDev");
    this->desired_variables.push_back("zpsrYStdDev");
    this->desired_variables.push_back("zpsrZStdDev");
    this->desired_variables.push_back("zCourse");
    this->desired_variables.push_back("zpsrNumObs");

    return;
}

bool MOOS2ROS::OnDisconnectFromServer() {
    this->RequestQuit();
    return true;
}

bool MOOS2ROS::Iterate() {
    if (!ros::ok())
        return false;
    return true;
}

// This is the function that should implicitly work
bool MOOS2ROS::OnNewMail(MOOSMSG_LIST &NewMail) {
    MOOSMSG_LIST::iterator p;
    for (p=NewMail.begin(); p!=NewMail.end(); p++) {
        CMOOSMsg &rMsg = *p;
        this->rospub.publish(handleMsg(rMsg));
    }
    return true;
}

fhwa2_MOOS_to_ROS::MOOSrosmsg MOOS2ROS::handleMsg(CMOOSMsg &Msg) {
    // Create ros msg to send
    fhwa2_MOOS_to_ROS::MOOSrosmsg rosmsg;
    rosmsg.header.stamp = ros::Time::now(); // William put the stamp as MOOStime
    rosmsg.header.frame_id = GetAppName();

    // Populate the rosmsg with data from the MOOS msg
    rosmsg.MOOStime = Msg.GetTime();
    rosmsg.MOOSname = Msg.GetName();
    rosmsg.MOOSsource = Msg.GetSource();
    if (Msg.IsDouble()) {
        rosmsg.MOOStype = std::string("Double");
        rosmsg.MOOSdouble = Msg.GetDouble();
    } else if (Msg.IsString()) {
        rosmsg.MOOStype = std::string("String");
        rosmsg.MOOSstring = Msg.GetString();
    } else {
        ROS_INFO_STREAM("Skipped MOOS message of unknown type");
    }

    return rosmsg;
}

int main(int argc, char * argv[]) {   
    ros::init(argc, argv, "moos2ros"); // Let ROS shutdown stuff itself, rename node in mission file
    ROS_INFO_STREAM("moos2ros node initialized");


    const char * mission_file = "Mission.moos";
    const char * app_name = "MOOS2ROS";
    switch(argc) {
    case 3:
        app_name = argv[2];
    case 2:
        mission_file = argv[1];
    }
    
    namename = app_name;

    MOOS2ROS app;
    // ROS_INFO_STREAM("namename: " << namename);
    // ROS_INFO_STREAM("app_name: " << app_name);
    // ROS_INFO_STREAM("mission_file: " << mission_file);

    std::stringstream ss;
    ss << "/moos/" << namename;
    std::string topic_name = ss.str();
    app.rospub = app.node_handle.advertise<fhwa2_MOOS_to_ROS::MOOSrosmsg>(topic_name, 100);

    app.Run(app_name, mission_file); // else call in thread (boost)

    return 0;
}