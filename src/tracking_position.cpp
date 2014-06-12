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

    ros::init(argc, argv, "tracking_position");
    ros::NodeHandle chatmatlabnode;
    ros::Subscriber dataMatlabCppInitRobotPose;
    ros::Publisher pubObjPos;

    MoCapMessenger mocapRobot;
    MoCapMessenger mocapActor;
    Robulab10 Robot;

    /// Initial definiton of the parameters
    double timenow;
    Eigen::Vector3f Obj_data_t,Obj_data_tbefore;
    std_msgs::Float64MultiArray ObjDataMatrix_msg;
    bool stop_flag = false;

    // Init Mocap
    dataMatlabCppInitRobotPose = chatmatlabnode.subscribe("InitRobPos", 2000, getDataMatlab);
    mocapRobot.sub = mocapRobot.n.subscribe("/evart/RobulabC/PO", 2000, &MoCapMessenger::callbackFunction, &mocapRobot);
    mocapActor.sub = mocapActor.n.subscribe("/evart/RobulabC/PO", 2000, &MoCapMessenger::callbackFunction, &mocapActor);
    //mocapTBot.sub = mocapTBot.n.subscribe("/vicon/Robot1/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);
    //mocapActor.sub = mocapActor.n.subscribe("/vicon/helmetLarge/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapActor);

    pubObjPos = chatmatlabnode.advertise<std_msgs::Float64MultiArray>("ObjData", 1);

    timenow = time(NULL)+1;
    std::cout << "Initialization MoCap." << std::flush;
    while(time(NULL)<timenow){
        ros::spinOnce();
        Obj_data_t = mocapRobot.item_XY_YAW_configuration_Eigen();
        std::cout << "." << std::flush;
        ros::Duration(0.05).sleep();
        std::cout << " // DONE // Robot Status [x: " << Obj_data_t[0] << " ; y: " << Obj_data_t[1] << " ; th: " << Obj_data_t[2] << "] //" << std::endl;

    }
    /*
    if(fabs(Obj_data_t[1])<1e-200){ std::cerr << "// ERROR: No Mocap data //" <<std::endl;
        std::cout << " // DONE // Robot Status [x: " << Obj_data_t[0] << " ; y: " << Obj_data_t[1] << " ; th: " << Obj_data_t[2] << "] //" << std::endl;
        return -1;}
    else
      std::cout << " // DONE // Robot Status [x: " << Obj_data_t[0] << " ; y: " << Obj_data_t[1] << " ; th: " << Obj_data_t[2] << "] //" << std::endl;
*/

    // Collect data
    unsigned int i=0;
    Obj_data_tbefore.setZero();
    while(ros::ok() && !stop_flag){
        ros::spinOnce();

        Obj_data_t.setZero();
        Obj_data_t = mocapRobot.item_XY_YAW_configuration_Eigen();
        if((fabs(Obj_data_t[1])>1e-200) && (Obj_data_t[1]-Obj_data_tbefore[1]!=0 && Obj_data_t[2]-Obj_data_tbefore[2]!=0)){
          tf::matrixEigenToMsg(Obj_data_t,ObjDataMatrix_msg);
          pubObjPos.publish(ObjDataMatrix_msg);

          std::cout << "Robot Pos - X: " << Obj_data_t[0] << " Y: " << Obj_data_t[1] << std::endl;
          ros::Duration(0.1).sleep();
          i++;

          // Exit condition
          if(Obj_data_t[0]<-1.5){
               std::cout << "GOAL" << std::endl;
               stop_flag = true;
           }

          }
        Obj_data_tbefore = Obj_data_t;
        ros::Duration(0.005).sleep();

      }




    return 0;
}
