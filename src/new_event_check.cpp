/// Tbot1

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

#include <sstream>

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
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <std_msgs/Int32MultiArray.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/Float32.h>

#include "turtlemotionclass.hpp"
#include "mocapmessenger.hpp"
#include "Robulab10Class.hpp"
#include "trajectoryClass.hpp"
#include "Motion_API.hpp"

#include <fstream>
#include <algorithm>
#include <iterator>

using namespace Eigen;

void chatterCallbackTrigger(const std_msgs::Float32::ConstPtr &msg)
{
      std::cout << "received data " << msg->data << std::endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "event_check");

    ros::NodeHandle chatnode;
    ros::Publisher RobotMoCapData = chatnode.advertise<std_msgs::Float64MultiArray>("ObjData", 1000);
    ros::Rate loop_rate(1000);

    MoCapMessenger mocapRobot;
    mocapRobot.sub = mocapRobot.n.subscribe("/evart/RobulabC/PO", 2000, &MoCapMessenger::callbackFunction, &mocapRobot);

    Robulab10 Robot;
    Robot.establish_connection();

    std::vector<double> RobotData_t, TBot1Data_t, TBot2Data_t;
    std::vector<std::vector<double> > RobotData, TBot1Data, TBot2Data;

    std::vector<double> actorX;
    std::vector<double> velY;
    std::vector<double> posX;
    std::vector<double> posY;
    std::vector<double> velMOD;

    bool stop_flag = false;
    int counter = 0;
    int counter2 = 0;
    double init_interval_time;

    std::cout << "Bringing up.." << std::flush;

    double timenow = time(NULL)+3;

    while(time(NULL)<timenow){
        ros::spinOnce();
        std::cout << ".3" << std::flush;
        ros::Duration(1).sleep();
        std::cout << ".2" << std::flush;
        ros::Duration(1).sleep();
        std::cout << ".1" << std::flush;
        ros::Duration(1).sleep();
        std::cout << ".." << std::flush;
    }

    std::cout << "\nStart" << std::endl;


    while(ros::ok() && !stop_flag){
        ros::spinOnce();

        /// Take data from MoCap, tracking the Actor
        RobotData_t.clear();
        RobotData_t = mocapRobot.item_XY_YAW_configuration();
        RobotData.push_back(RobotData_t);

        Robot.move_robot(0.3,0);

        std::cout << "publishing actor" << RobotData_t[0] << std::endl;


        counter++;
        std::cout << counter << std::endl;
         /// Check Event - Set Condition
       if(RobotData_t[0]<-1.5){
            std::cout << "EVENT" << std::endl;
            Robot.move_robot(0,0);
            stop_flag = true;
        }

        ros::Duration(0.005).sleep();

    }
    Robot.move_robot(0,0);

    std::cout << "Send Data" << std::endl;
    Eigen::MatrixXd RobotDataMatrix = Eigen::MatrixXd::Zero(RobotData.size(),3);

    for(int c=0; c<RobotData.size(); ++c){
        RobotDataMatrix(c,0) = RobotData[c][0]; RobotDataMatrix(c,1) = RobotData[c][1]; RobotDataMatrix(c,2) = RobotData[c][2];
    }

    std_msgs::Float64MultiArray RobotDataMatrix_msg;

    tf::matrixEigenToMsg(RobotDataMatrix,RobotDataMatrix_msg);

    //while(1){

    std::cout << "Send data to Matlab " << std::endl;
    RobotMoCapData.publish(RobotDataMatrix_msg);
        //ros::Duration(2).sleep();
    //}


  return 0;
}
