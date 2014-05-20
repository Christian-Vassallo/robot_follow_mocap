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

    ros::init(argc, argv, "randommotion1");
    //ros::start();


    /// Initial definiton of the parameters
    TurtlebotMotion tbm;

    Move_Along_a_Line (&tbm, 0.7, 0, 4);
    ros::Duration(1).sleep();
    Move_Along_a_Line (&tbm, -0.7, 0, 4);



    return 0;
}
