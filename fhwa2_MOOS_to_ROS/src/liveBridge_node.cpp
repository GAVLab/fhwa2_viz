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
    // GetDesiredVaribles();
    // MOOSTrace("Got desired variables");
    // std::vector<std::string>::iterator p;
    // for (p=this->desired_variables.begin(); 
    //      p!=this->desired_variables.end(); p++) {
    //     m_Comms.Register(*p, this->min_upd);
    // }
    m_Comms.Register("zX", 0);
    m_Comms.Register("zY", 0);
    m_Comms.Register("zZ", 0);
    m_Comms.Register("zXStdDev", 0);
    m_Comms.Register("zYStdDev", 0);
    m_Comms.Register("zZStdDev", 0);
    m_Comms.Register("zCourse", 0);
    m_Comms.Register("zpsrX", 0);
    m_Comms.Register("zpsrY", 0);
    m_Comms.Register("zpsrZ", 0);
    m_Comms.Register("zpsrXStdDev", 0);
    m_Comms.Register("zpsrYStdDev", 0);
    m_Comms.Register("zpsrZStdDev", 0);

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

    if (m_MissionReader.GetConfigurationParam("X_var", X_var)) {
        desired_variables.push_back(X_var);
        MOOSTrace("Subscribed to X_var:\n\t");
        std::cout << X_var << std::endl;
    }
    if (m_MissionReader.GetConfigurationParam("Y_var", Y_var)) {
        desired_variables.push_back(Y_var);
        MOOSTrace("Subscribed to Y_var\n\t");
        std::cout << Y_var <<std::endl;
    }
    if (m_MissionReader.GetConfigurationParam("Z_var", Z_var)) {
        desired_variables.push_back(Z_var);
        MOOSTrace("Subscribed to Z_var\n\t");
        std::cout << Z_var <<std::endl;
    }
    if (m_MissionReader.GetConfigurationParam("XStdDev_var", XStdDev_var)) {
        desired_variables.push_back(XStdDev_var);
        MOOSTrace("Subscribed to XStdDev_var\n\t");
        std::cout << XStdDev_var <<std::endl;
    }
    if (m_MissionReader.GetConfigurationParam("YStdDev_var", YStdDev_var)) {
        desired_variables.push_back(YStdDev_var);
        MOOSTrace("Subscribed to YStdDev_var\n\t");
        std::cout << YStdDev_var <<std::endl;
    }
    if (m_MissionReader.GetConfigurationParam("ZStdDev_var", ZStdDev_var)) {
        desired_variables.push_back(ZStdDev_var);
        MOOSTrace("Subscribed to ZStdDev_var\n\t");
        std::cout << ZStdDev_var <<std::endl;
    }
    if (m_MissionReader.GetConfigurationParam("Course_var", Course_var)) {
        desired_variables.push_back(Course_var);
        MOOSTrace("Subscribed to Course_var\n\t");
        std::cout << Course_var <<std::endl;
    }

    if (m_MissionReader.GetConfigurationParam("min_upd", this->min_upd))
        return;
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
        this->rospub.publish(this->handleMsg(rMsg));
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