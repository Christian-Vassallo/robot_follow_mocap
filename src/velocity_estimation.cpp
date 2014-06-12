/// Velocity estimation

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

    ros::NodeHandle _item_node;
    ros::Publisher _item_pub = _item_node.advertise<std_msgs::Float64MultiArray>("ObjData", 2000);

    MoCapMessenger _item_mocap;
    _item_mocap.sub = _item_mocap.n.subscribe("/evart/HGoal/PO", 2000, &MoCapMessenger::callbackFunction, &_item_mocap);

    Robulab10 Robot;
    Robot.establish_connection();

    std::vector<double> item_data_t;
    std::vector<double> item_data_tbef(3);
    std::vector<std::vector<double> > item_data;

    bool stop_flag = false;
    int counter = 0;


    // Initialization MoCap
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

    // Starting to record motion if the object is visible
    std::cout << "\nStart" << std::endl;
    while(ros::ok() && !stop_flag){
        ros::spinOnce();

        /// Take data from MoCap, tracking the Actor
        item_data_t.clear();
        item_data_t = _item_mocap.item_XY_YAW_configuration();

        //Robot.move_robot(0.8,0);

        std::cout << "publishing actor" << item_data_t[0] << std::endl;
        if((fabs(item_data_t[0])>1e-200)){

          item_data.push_back(item_data_t);
          counter++;
          std::cout << counter << std::endl;
           /// Check Event - Set Condition
         if(item_data_t[0]<-1){
              std::cout << "EVENT" << std::endl;
              //Robot.move_robot(0,0);
              stop_flag = true;
          }
        }
        item_data_tbef = item_data_t;

        ros::Duration(0.005).sleep();

    }
    Robot.move_robot(0,0);

    std::cout << "Send Data" << std::endl;
    Eigen::MatrixXd item_data_matrix = Eigen::MatrixXd::Zero(item_data.size(),3);
    std_msgs::Float64MultiArray item_data_msg;

    for(int c=0; c<item_data.size(); ++c){
        item_data_matrix(c,0) = item_data[c][0]; item_data_matrix(c,1) = item_data[c][1]; item_data_matrix(c,2) = item_data[c][2];
    }

    // Define the message to send
    tf::matrixEigenToMsg(item_data_matrix,item_data_msg);

    std::cout << "Send data to Matlab " << std::endl;
    _item_pub.publish(item_data_msg);


  return 0;
}

