/*
    Runs Moos as a ros node
*/
#include <fhwa2_MOOS_to_ROS/liveBridge.h>
#include <vector>
#include <string>
#include <sstream>

MOOS2ROS::MOOS2ROS() {
    this->node_handle = ros::NodeHandle("MOOS");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    this->SetQuitOnFailedIterate(true);

    // config_variables[0] = "X_var";
    // config_variables[1] = "Y_var";
    // config_variables[2] = "Z_var";
    // config_variables[3] = "XStdDev_var";
    // config_variables[4] = "YStdDev_var";
    // config_variables[5] = "ZStdDev_var";
    // config_variables[6] = "Course_var";

    // MOOSTrace("MOOS Name before being set: %s", m_sMOOSName.c_str());
    m_sAppName = "gMOOS2ROS";
    m_sMOOSName = "gMOOS2ROS"; // May cause problems
    m_MissionReader.SetAppName("gMOOS2ROS");
    MOOSTrace("\nMOOS2ROS class initialized with App Name: %s \n\n", GetAppName().c_str());
}

MOOS2ROS::~MOOS2ROS() {
    ros::shutdown();
    this->RequestQuit();
}

bool MOOS2ROS::OnConnectToServer() {
    GetDesiredVaribles();

    std::vector<std::string>::iterator p;
    for (p=this->desired_variables.begin(); 
         p!=this->desired_variables.end(); p++) {
        m_Comms.Register(*p, 0);
    }
    return true;
}

void MOOS2ROS::GetDesiredVaribles() {
    //TODO use m_MissionReader.GetConfiguration to return a list of available params
    std::string X_var;
    std::string Y_var;
    std::string Z_var;
    std::string XStdDev_var;
    std::string YStdDev_var;
    std::string ZStdDev_var;
    std::string Course_var;

    // if (m_MissionReader.GetConfigurationParam("DesiredVariables", s_Vars)) {
    if (m_MissionReader.GetConfigurationParam("X_var", X_var))
        desired_variables.push_back(X_var);
    if (m_MissionReader.GetConfigurationParam("Y_var", Y_var))
        desired_variables.push_back(Y_var);
    if (m_MissionReader.GetConfigurationParam("Z_var", Z_var))
        desired_variables.push_back(Z_var);
    if (m_MissionReader.GetConfigurationParam("XStdDev_var", XStdDev_var))
        desired_variables.push_back(XStdDev_var);
    if (m_MissionReader.GetConfigurationParam("YStdDev_var", YStdDev_var))
        desired_variables.push_back(YStdDev_var);
    if (m_MissionReader.GetConfigurationParam("ZStdDev_var", ZStdDev_var))
        desired_variables.push_back(ZStdDev_var);
    if (m_MissionReader.GetConfigurationParam("Course_var", Course_var))
        desired_variables.push_back(Course_var);
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
        // send no matter what
        this->rospub.publish(this->handleMsg(rMsg));
    }
    return true;
}

fhwa2_MOOS_to_ROS::MOOSrosmsg MOOS2ROS::handleMsg(CMOOSMsg &Msg) {
    // Create ros msg to send
    fhwa2_MOOS_to_ROS::MOOSrosmsg rosmsg;
    rosmsg.stamp = ros::Time::now(); // William put the stamp as MOOStime
    rosmsg.frame_id = std::string("MOOS");

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
    ros::init(argc, argv, "moos2ros"); // Let ROS shutdown stuff itself
    ROS_INFO_STREAM("moos2ros node initialized");
    
    const char * mission_file = "Mission.moos";
    const char * app_name = "MOOS2ROS";
    switch(argc) {
    case 3:
        app_name = argv[2];
    case 2:
        mission_file = argv[1];
    }

    MOOS2ROS app;

    app.rospub = app.node_handle.advertise<fhwa2_MOOS_to_ROS::MOOSrosmsg>("/moos/incoming", 1);

    app.Run(app_name, mission_file); // else call in thread (boost)

    return 0;
}