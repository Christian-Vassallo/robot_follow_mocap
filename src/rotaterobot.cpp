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
#include <std_msgs/Int32MultiArray.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/Float32.h>

#include "turtlemotionclass.hpp"
#include "mocapmessenger.hpp"
#include "trajectoryClass.hpp"
#include "API_Turtlebot.cpp"



using namespace Eigen;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_control");
    //ros::start();


    /// Initial definiton of the parameters
    TurtlebotMotion tbm;
    MoCapMessenger mocapTBot;

    /// Check Robot init Pos
    std::vector<double> TTBot_configuration;
    mocapTBot.sub = mocapTBot.n.subscribe("/vicon/Robot1/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);

    int timenow = time(NULL)+2;
    while(time(NULL)<timenow)
        ros::spinOnce();
    TTBot_configuration = mocapTBot.item_XY_YAW_configuration();

    bool exitflag = true;

    if(TTBot_configuration[2]>3 || TTBot_configuration[2]<-3){
        while(exitflag){
            std::cout << " tb " << TTBot_configuration[2] << std::endl;
            ros::spinOnce();
            TTBot_configuration = mocapTBot.item_XY_YAW_configuration();
            RotateRobot(&tbm);
            if(TTBot_configuration[2]>-0.05 && TTBot_configuration[2]<0.05)
                exitflag=false;
        }
    }
    else{
        while(exitflag){
            std::cout << " tb " << TTBot_configuration[2] << std::endl;
            ros::spinOnce();
            TTBot_configuration = mocapTBot.item_XY_YAW_configuration();
            RotateRobot(&tbm);
            if(TTBot_configuration[2]>3.05 || TTBot_configuration[2]<-3.05)
                exitflag=false;
        }
    }



    return 0;
}
