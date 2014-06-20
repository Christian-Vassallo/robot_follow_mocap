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
#include <stdlib.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>

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
#include "tracking_API.hpp"

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

    ros::init(argc, argv, "velocity_estimation");

    ros::NodeHandle _item_node;
    ros::Publisher _item_pub = _item_node.advertise<std_msgs::Float64MultiArray>("ObjDataMocap", 2000);
    ros::Subscriber _item_sub = _item_node.subscribe("ObjVel", 2000, &getDataMatlab);

    MoCapMessenger _item_mocap;
    /// LAAS /// _item_mocap.sub = _item_mocap.n.subscribe("/evart/HGoal/PO", 2000, &MoCapMessenger::callbackFunction, &_item_mocap);
    _item_mocap.sub = _item_mocap.n.subscribe("/vicon/head/endBone", 2000, &MoCapMessenger::callbackFunction, &_item_mocap);

    Robulab10 Robot;
    Robot.establish_connection();

    item_goal item1;

    std::vector<double> item_data_t;
    std::vector<double> item_data_tbef(3);
    std::vector<std::vector<double> > item_data;

    bool stop_flag = false;
    int counter = 0;
    double ObjVelocity;
    double _MPD;
    unsigned int iter_abs = 0;  // absolute iterator of the tries

    std::ifstream  data("ordrepassage.csv");
    std::vector<bool> robotmove;
    std::vector<bool> robotLeftRight;
    std::vector<double> MPDvector;

        std::string line;
        while(std::getline(data,line))
        {
            std::stringstream  lineStream(line);
            std::string        cell;
            std::string        subcell;
            while(std::getline(lineStream,cell,','))
            {
              // You have a cell!!!!
              robotmove.push_back(cell[0]=='Y');
              robotLeftRight.push_back(cell[2]=='L');
              subcell = cell.substr(4,8);
              MPDvector.push_back(atof(subcell.c_str()));
            }
        }


    // Initialization MoCap
    std::cout << "Bringing up.." << std::flush;
    double timenow = time(NULL)+0.1;
    while(time(NULL)<timenow){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }


    // Starting to record motion if the object is visible
    std::cout << "\nStart" << std::endl;
    while(ros::ok() && !stop_flag){
        ros::spinOnce();

        /// Take data from MoCap, tracking the Actor
        item_data_t.clear();
        item_data_t = _item_mocap.item_XY_YAW_configuration();

         /// item_data_t.push_back(0+counter);         /// CONDIZIONE OFFLINE
         /// item_data_t.push_back(0+2*counter);         /// CONDIZIONE OFFLINE
         /// item_data_t.push_back(1);         /// CONDIZIONE OFFLINE

        std::cout << "publishing actor" << item_data_t[1] << std::endl;

        //Robot.move_robot(0.8,0);

        // Init Condition

        if(item_data_t[1]>-10.70 && item_data_t[0]>-4){
        /// if(true){   /// CONDITION OFFLINE

          item_data.push_back(item_data_t);
          counter++;
          std::cout << counter << std::endl;

          // End condition
          if(item_data_t[1]>-7.83){
          /// if(counter>1){ /// condizione offline
              //-7.83 line blue
              std::cout << "EVENT" << std::endl;
              //Robot.move_robot(0,0);
              stop_flag = true;
          }
        }
        item_data_tbef = item_data_t;

        ros::Duration(0.01).sleep();

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



    // Receive Message from Matlab - Message structure:
    // 0 - Pos X
    // 1 - Pos Y
    // 2 - Vel X
    // 3 - Vel Y
    std::cout << "Waiting Actor Velocity from Matlab" << std::endl;
    do{
        ros::spinOnce();
        ObjVelocity = MatlabData[3];
        ros::Duration(0.01).sleep();
      }while(ObjVelocity == 0);
    std::cout << "Received: " << ObjVelocity << std::endl;

    // Set the virtual robot to the correct position

    // Extract the time needed by actor to arrive to the line
    double _THETA = -M_PI;
    double dt = 0.009;

    // Actor Path
    double init_actor_line_x =  0;
    double end_actor_line_x  =  MatlabData[0];
    double init_actor_line_y = -6.5;
    double end_actor_line_y  = -1.723;      /// TO CHECK

    // Inserting MPD
    //_MPD = MPDvector[iter_abs];
    _MPD = 1.2;

    // Compute the crossing point according to the velocity actor
    double s = fabs(end_actor_line_y - init_actor_line_y);
    double t = (s + _MPD) / ObjVelocity;

    // Robot Path
    double init_robot_line_x; // Value to compute
    double end_robot_line_x = end_actor_line_x;
    double init_robot_line_y = end_actor_line_y;
    double end_robot_line_y = end_actor_line_y;
    double RobotVelocity = 1.4; // m/s
    double offsetspace = 3.5;
    double offsettime;

    // Some offset to move the robot a little bit more
    offsettime = offsetspace/RobotVelocity;

    /// --- INSERIRE IF ELSE ACCORDING TO LEFT OR RIGHT ---- ////
    init_robot_line_x = end_robot_line_x+RobotVelocity*t;



    std::cout << "percorso " << end_robot_line_x << " = " <<  RobotVelocity<< " * " << t << std::endl;
    std::cout << " l'attore si muove sulla linea x " << end_actor_line_x << std::endl;
    std::cout << " il robot dovrebbe partire da " << init_robot_line_x << " per arrivare a " << end_robot_line_x << std::endl;
    std::cout << "senza mpd - " << " il robot dovrebbe partire da " << end_actor_line_x+RobotVelocity*s/ObjVelocity << " per arrivare a " << end_actor_line_x << std::endl;
    std::cout << " mpd " << _MPD << std::endl;

    std::vector<double> virtualrobot_config_t;
    std::vector<std::vector<double> > virtualrobot_config2;

    MatrixXd virtualrobot_config(int(t/dt)+1, 3);
    virtualrobot_config.row(0) = Vector3d(init_robot_line_x, init_robot_line_y, _THETA);

    virtualrobot_config_t.clear();
    virtualrobot_config_t.push_back(init_robot_line_x);
    virtualrobot_config_t.push_back(init_robot_line_y);
    virtualrobot_config_t.push_back(_THETA);
    virtualrobot_config2.push_back(virtualrobot_config_t);

    std::cout << " prova " << virtualrobot_config2[0][0] <<  " " << virtualrobot_config2[0][1] << " " << virtualrobot_config2[0][2] << std::endl;

    // Initial Configuration

    for(unsigned int kk=0; kk<=(int(t/dt)+int(offsettime/dt)); ++kk){
      //std::vector<int>::iterator it = kk;
     // virtualrobot_config.row(kk+1) = Vector3d(virtualrobot_config(kk,0)+ObjVelocity*dt, virtualrobot_config(kk,1), _THETA);

      virtualrobot_config_t.clear();
      virtualrobot_config_t.push_back(virtualrobot_config2[virtualrobot_config2.size()-1][0] - RobotVelocity*dt);
      virtualrobot_config_t.push_back(virtualrobot_config2[virtualrobot_config2.size()-1][1]);
      virtualrobot_config_t.push_back(_THETA);
      virtualrobot_config2.push_back(virtualrobot_config_t);

      //virtualrobot_config_t.push_back(virtualrobot_config2[kk,0]+ObjVelocity*dt);
    /*virtualrobot_config_t.push_back(virtualrobot_config2[kk,1]);
      virtualrobot_config_t.push_back(_THETA);
      virtualrobot_config2.push_back(virtualrobot_config_t);*/
    }

    std::cout << "il robot arriva fino a " << virtualrobot_config2[virtualrobot_config2.size()-1][0] << std::endl;

   // std::cout << virtualrobot_config << std::endl;
    std::cout << "Vel " << ObjVelocity << std::endl;

   for(unsigned int kk=0; kk<=int(t/dt); ++kk)
      std::cout << dt*kk << " - " << virtualrobot_config2[kk][0] << " " << virtualrobot_config2[kk][1] << " " << virtualrobot_config2[kk][2] << std::endl;

    std::cout << " tempo " << t << std::endl;

    std::cout << robotLeftRight[iter_abs] << std::endl;
    // Track the virtual robot
    item1.virtual_tracking_control_MPD(virtualrobot_config2, robotLeftRight[iter_abs]);

    return 0;
}

