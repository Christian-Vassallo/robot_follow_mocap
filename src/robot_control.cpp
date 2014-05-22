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


#include "turtlemotionclass.hpp"
#include "mocapmessenger.hpp"
#include "trajectoryClass.hpp"
#include "Robulab10Class.hpp"
#include "Motion_API.hpp"


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

    dataMatlabCppInitRobotPose = chatmatlabnode.subscribe("InitRobPos", 2000, getDataMatlab);
    mocapTBot.sub = mocapTBot.n.subscribe("/evart/Robulab/PO", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);
    mocapActor.sub = mocapActor.n.subscribe("/evart/Robulab/PO", 2000, &MoCapMessenger::callbackFunction, &mocapActor);
    //mocapTBot.sub = mocapTBot.n.subscribe("/vicon/Robot1/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);
    //mocapActor.sub = mocapActor.n.subscribe("/vicon/helmetLarge/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapActor);

    timenow = time(NULL)+0.5;
    while(time(NULL)<timenow){
        ros::spinOnce();
        Robot_config = mocapTBot.item_XY_YAW_configuration();
    }

    timenow = time(NULL)+0.5;
    while(time(NULL)<timenow){
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        //std::cout << "TBot starts config X Y: " << RobotInitPosXY[0] << " " << RobotInitPosXY[1] << std::endl;
    }
    std::cout << "Matlab data RECEIVED..." << std::endl;


    /// Circle: 1
    velocity_input = 0.55;
    Ox = 0.45;
    Oy = -6;
    radius = 1.0;
    //Circle_Following_Control(&tbm, velocity_input, Ox, Oy, radius, check_event);


    /// Line: 1
    velocity_input = 0.5;
    starting_point_x = Robot_config[0];
    end_point_x = Robot_config[0]-2;
    starting_point_y = Robot_config[1];
    end_point_y = Robot_config[1];

    check_event = 0;
    Line_Following_Control(velocity_input, starting_point_x, end_point_x, starting_point_y, end_point_y, check_event);



    return 0;
}
