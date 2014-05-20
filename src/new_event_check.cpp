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
#include "trajectoryClass.hpp"
#include "API_Turtlebot.cpp"

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

    ros::Publisher chatter_pub = chatnode.advertise<std_msgs::Int32>("chatter", 1000);

    ros::Publisher ActorMoCapData = chatnode.advertise<std_msgs::Float64MultiArray>("ActorMoCapData2", 1000);
    ros::Publisher TBot1MoCapData = chatnode.advertise<std_msgs::Float64MultiArray>("TBot1MoCapData", 1000);
    ros::Publisher TBot2MoCapData = chatnode.advertise<std_msgs::Float64MultiArray>("TBot2MoCapData", 1000);

    ros::Rate loop_rate(1000);

    MoCapMessenger mocapActor;
    MoCapMessenger mocapTbot1;
    MoCapMessenger mocapTbot2;

    mocapActor.sub = mocapActor.n.subscribe("/vicon/helmetLarge/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapActor);
    mocapTbot1.sub = mocapTbot1.n.subscribe("/vicon/Robot1/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTbot1);
    mocapTbot2.sub = mocapTbot2.n.subscribe("/vicon/Robot2/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTbot2);

    std::vector<double> ActorData_t, TBot1Data_t, TBot2Data_t;
    std::vector<std::vector<double> > ActorData, TBot1Data, TBot2Data;

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
        ActorData_t.clear();
        ActorData_t = mocapActor.item_XY_YAW_configuration();

        /// Take data from MoCap, tracking the TBot1
        TBot1Data_t.clear();
        TBot1Data_t = mocapTbot1.item_XY_YAW_configuration();

        /// Take data from MoCap, tracking the TBot1
        TBot2Data_t.clear();
        TBot2Data_t = mocapTbot2.item_XY_YAW_configuration();

        ActorData.push_back(ActorData_t);
        TBot1Data.push_back(TBot1Data_t);
        TBot2Data.push_back(TBot2Data_t);

        std_msgs::Int32 msg;

        std::cout << "publishing actor" << ActorData_t[1] << std::endl;
        std::cout << "publishing tbot1" << TBot1Data_t[1] << std::endl;
        std::cout << "publishing tbot2" << TBot2Data_t[1] << std::endl;

        chatter_pub.publish(msg);

        counter++;
        std::cout << counter << std::endl;
         /// Check Event - Set Condition
        if(ActorData_t[1]>-4.19){
        //if(counter<100){
            std::cout << "EVENT" << std::endl;
            stop_flag = true;
        }

        ros::Duration(0.006666667).sleep();

    }

    std::cout << "Send Data" << std::endl;

    Eigen::MatrixXd ActorDataMatrix = Eigen::MatrixXd::Zero(ActorData.size(),3);
    Eigen::MatrixXd TBot1DataMatrix = Eigen::MatrixXd::Zero(TBot1Data.size(),3);
    Eigen::MatrixXd TBot2DataMatrix = Eigen::MatrixXd::Zero(TBot2Data.size(),3);

    for(int c=0; c<ActorData.size(); ++c){
        ActorDataMatrix(c,0) = ActorData[c][0]; ActorDataMatrix(c,1) = ActorData[c][1]; ActorDataMatrix(c,2) = ActorData[c][2];
        TBot1DataMatrix(c,0) = TBot1Data[c][0]; TBot1DataMatrix(c,1) = TBot1Data[c][1]; TBot1DataMatrix(c,2) = TBot1Data[c][2];
        TBot2DataMatrix(c,0) = TBot2Data[c][0]; TBot2DataMatrix(c,1) = TBot1Data[c][1]; TBot2DataMatrix(c,2) = TBot2Data[c][2];
    }

    std_msgs::Float64MultiArray ActorDataMatrix_msg;
    std_msgs::Float64MultiArray TBot1DataMatrix_msg;
    std_msgs::Float64MultiArray TBot2DataMatrix_msg;

    tf::matrixEigenToMsg(ActorDataMatrix,ActorDataMatrix_msg);
    tf::matrixEigenToMsg(TBot1DataMatrix,TBot1DataMatrix_msg);
    tf::matrixEigenToMsg(TBot2DataMatrix,TBot2DataMatrix_msg);

    //while(1){

    std::cout << "Send data to Matlab " << std::endl;
    ActorMoCapData.publish(ActorDataMatrix_msg);
    TBot1MoCapData.publish(TBot1DataMatrix_msg);
    TBot2MoCapData.publish(TBot2DataMatrix_msg);
        //ros::Duration(2).sleep();
    //}


  return 0;
}
