#include "ros/ros.h"
#include <sstream>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdlib.h>
#include <sensor_msgs/Imu.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Sound.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <time.h>
#include <string.h>
#include <fstream>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>


#include <std_msgs/Int32MultiArray.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include "turtlemotionclass.hpp"
#include "mocapmessenger.hpp"
#include "trajectoryClass.hpp"
#include "Robulab10Class.hpp"
#include "Motion_API.hpp"
#include "kalman/ekfilter.hpp"

void chatterCallbackTrigger(const std_msgs::Float32::ConstPtr &msg)
{
      std::cout << "received data " << msg->data << std::endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_control");
    ros::NodeHandle chatmatlabnode;
    ros::Subscriber dataMatlabCppInitRobotPose;

    MoCapMessenger mocapTBot;
    MoCapMessenger mocapActor;
    Robulab10 Robot;

    /// Initial definiton of the parameters
    int check_event;
    double timenow;
    double velocity_input;
    double Ox, Oy, radius;
    double starting_point_x, end_point_x, starting_point_y, end_point_y;
    std::vector<double> Robot_config;

    // Init Mocap
///    dataMatlabCppInitRobotPose = chatmatlabnode.subscribe("InitRobPos", 2000, getDataMatlab);
///    mocapTBot.sub = mocapTBot.n.subscribe("/evart/RobulabC/PO", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);
///    mocapActor.sub = mocapActor.n.subscribe("/evart/RobulabC/PO", 2000, &MoCapMessenger::callbackFunction, &mocapActor);
    mocapTBot.sub = mocapTBot.n.subscribe("/vicon/robot2/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);
    mocapActor.sub = mocapActor.n.subscribe("/vicon/robot2/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapActor);

    // Init Robot
    //Robot.establish_connection();

    std::cout << "Initialization Robot." << std::flush;
    //Robot.check_battery();

    timenow = time(NULL)+0.1;

    std::cout << "Initialization MoCap." << std::flush;
    while(time(NULL)<timenow){
        ros::spinOnce();
        Robot_config = mocapTBot.item_XY_YAW_configuration();
        std::cout << "." << std::flush;
        ros::Duration(0.01).sleep();
        std::cout << " // DONE // Robot Status [x: " << Robot_config[0] << " ; y: " << Robot_config[1] << " ; th: " << Robot_config[2] << "] //" << std::endl;

    }
    if(fabs(Robot_config[1])<1e-200){ std::cerr << "// ERROR: No Mocap data //" <<std::endl;
        std::cout << " // DONE // Robot Status [x: " << Robot_config[0] << " ; y: " << Robot_config[1] << " ; th: " << Robot_config[2] << "] //" << std::endl;
        return -1;}
    else
      std::cout << " // DONE // Robot Status [x: " << Robot_config[0] << " ; y: " << Robot_config[1] << " ; th: " << Robot_config[2] << "] //" << std::endl;

    /// Line: 1

    velocity_input = 0.6;
    starting_point_x = Robot_config[0];
    end_point_x = -5.62;
    starting_point_y = Robot_config[1];
    end_point_y = Robot_config[1];
    check_event = 1;
    Line_Following_Control(velocity_input, starting_point_x, end_point_x, starting_point_y, end_point_y, check_event);


/*
    std::ifstream is("file.csv");
    std::cout << "letto" << std::endl;
    std::cout << is << std::endl;
    std::string line;
    while (std::getline(is, line))
    {
        const char *begin = line.c_str();

        // strip beyond first comma
        if (const char *end = strchr(begin, ','))
        {
            std::string column1(begin, end - begin);
            std::cout << column1 << std::endl;
        }
    }
*/
    return 0;
}
