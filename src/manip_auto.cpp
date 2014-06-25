/// manip_auto.cpp

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

void PressEnterToContinue()
  {
  int c;
  fflush( stdout );
  do c = getchar(); while ((c != '\n') && (c != EOF));
  }

int main(int argc, char **argv)
{

    ros::init(argc, argv, "manip_auto");

    ros::NodeHandle _item_node;
    ros::Publisher  _item_pub = _item_node.advertise<std_msgs::Float64MultiArray>("ObjDataMocap", 2000);
    ros::Subscriber _item_sub = _item_node.subscribe("ObjVel", 2000, &getDataMatlab);

    MoCapMessenger _item_mocap;
    MoCapMessenger _robot_mocap;
    _item_mocap.sub = _item_mocap.n.subscribe("/vicon/head/endBone", 2000, &MoCapMessenger::callbackFunction, &_item_mocap);
    _robot_mocap.sub = _robot_mocap.n.subscribe("/vicon/robot2/endBone", 2000, &MoCapMessenger::callbackFunction, &_robot_mocap);

    Robulab10 Robot;
    Robot.establish_connection();


    item_goal item1;

    std::ifstream  data("ordrepassage.csv");
    std::vector<bool> robotmove;
    std::vector<bool> robotLeftRight;
    std::vector<double> MPDvector;
    std::string line;
    int counter;                // General counter
    bool stop_flag;             // Flag to stop the tracking control (set by condition of exit)

    counter = 0;
    stop_flag = false;

    /// READ CSV FILE
    while(std::getline(data,line))
    {
        std::stringstream  lineStream(line);
        std::string        cell;
        std::string        subcell;
        while(std::getline(lineStream,cell,','))
        {
          // You have a cell!!!!
          robotmove.push_back(cell[0]=='Y');
          robotLeftRight.push_back((cell[2]=='R'));
          subcell = cell.substr(4,8);
          MPDvector.push_back(atof(subcell.c_str()));
        }
    }

    unsigned int iter_abs_init;

    // Check if it is the first try
    if(argc>1){
      iter_abs_init = atoi(argv[1]);
      std::cout << "The system will restart again from try number: " << iter_abs_init << std::endl;
      }

    unsigned int iter_abs;      // absolute iterator of the tries

    // Initialization MoCap
    std::cout << "Bringing up everything.." << std::flush;

    for(iter_abs=iter_abs_init; iter_abs<MPDvector.size(); ++iter_abs){

      bool _RBTMV;                // Robot Move flag: 0 no motion, 1 robot actived
      bool _LRPOS;                // Left or Right position flag of robot, 0: Left
      double ObjVelocity;         // Velocity of the actor tracked
      double _MPD;                // MPD value
      double velocity_input;      // velocity of the robot using path following control
      double starting_point_x, end_point_x, starting_point_y, end_point_y;  // Init / End condition path following control
      double timenow;             // Variable used to check the time 'clock'
      double _THETA;              // Theta angle of the robot wrt zero frame
      double dt;                  // interval of time to define the position of the virtual robot t by t
      double init_actor_line_x;   // useless
      double end_actor_line_x;    // x position of the actor
      double init_actor_line_y;   // green line - actor starts to see the robot
      double end_actor_line_y;    // blue line - interaction collision line (where is the robot)
      double s;                   // distance between the moment that the actor sees the robot and the collision line
      double t;                   // time between robot seen and interaction with the robot
      double init_robot_line_x;   // where the robot should start
      double end_robot_line_x;    // where the robot should stop to collide
      double init_robot_line_y;   // interaction line
      double end_robot_line_y;    // interaction line
      double RobotVelocity;       // Robot velocity (1.4 m/s )
      double offsetspace;         // some offset otherwise the robot stops in the collision point
      double offsettime;          // time to add offset space




      std::vector<double> item_data_t;
      std::vector<double> robot_data_t;
      std::vector<double> virtualrobot_config_t;
      std::vector<std::vector<double> > virtualrobot_config;
      std::vector<std::vector<double> > item_data;

      std::cout << " TRY NUMBER " << iter_abs << std::endl;
      std::cout << " PARAM - RBTMV " <<  robotmove[iter_abs] << " LRPOS: " << robotLeftRight[iter_abs] << " MPD " << MPDvector[iter_abs] << std::endl;
      std::cout << "Initizialitation..." << std::endl;

      // Preparing MOCAP
      timenow = time(NULL)+1;
      while(time(NULL)<timenow){
          ros::spinOnce();
          ros::Duration(0.01).sleep();
          robot_data_t = _robot_mocap.item_XY_YAW_configuration_OFFSET(-1.745+M_PI);
      }

      std::cout << " Presse ENTER for the next try (number " << iter_abs << " ) " << std::endl;

      // Reading MPD from CSV
      _MPD = MPDvector[iter_abs];
      _RBTMV = robotmove[iter_abs];
      _LRPOS = robotLeftRight[iter_abs];


      std::cout << " PARK THE ROBOT IN THE " << std::endl;
      if(_LRPOS==0)
        std::cout << " --- LEFT --- SIDE OF THE ACTOR" << std::endl;
      else
        std::cout << " --- RIGHT --- SIDE OF THE ACTOR" << std::endl;

      if(_RBTMV==0)
        std::cout << "ATTENTION: No Robot Motion in this try..." << std::endl;

      PressEnterToContinue();



      if(_RBTMV==0)
        std::cout << "ATTENTION: Do nothing..." << std::endl;
      else{
          // Put in the correct position the robot

          velocity_input = 0.6;
          starting_point_x = robot_data_t[0];
          starting_point_y = robot_data_t[1];
          end_point_y = robot_data_t[1];

          if(_LRPOS==0)
            end_point_x = -5.62;  // LEFT
          else
            end_point_x = 5.62;   // RIGHT

          std::cout << "Creating a line path from ( " << starting_point_x << " ; " << starting_point_y << " ) to (" << end_point_x << " ; " << end_point_y << " ) " << std::endl;

          /// --- PARK THE ROBOT IN THE INIT POSITION --- ///
          //Line_Following_Control_RBTPOS(velocity_input, starting_point_x, end_point_x, starting_point_y, end_point_y, _LRPOS);


          // Starting to record motion if the object is visible
          std::cout << "Actor Analysis..." << std::endl;
          item_data.clear();
          stop_flag = false;
          while(ros::ok() && !stop_flag){
              ros::spinOnce();

              /// Take data from MoCap, tracking the Actor
              item_data_t.clear();
              item_data_t = _item_mocap.item_XY_YAW_configuration();

              //// CONDITION OFFLINE
              //item_data_t.push_back(1);
             // item_data_t.push_back(1);
              //item_data_t.push_back(1);
              //item_data.push_back(item_data_t);

              std::cout << "Publishing actor" << item_data_t[1] << std::endl;
              if(fabs(item_data_t[0])<0.001){
                  std::cout << "Actor non seen" << std::endl;
                  //stop_flag = true;

                }
              else{
                // Init Condition
                  if(item_data_t[1]>-10.70 && item_data_t[0]>-4){

                    item_data.push_back(item_data_t);
                    counter++;
                    std::cout << counter << std::endl;

                    // End condition
                    if(item_data_t[1]>-7.83){
                        std::cout << "end cond " << item_data_t[1] << std::endl;
                        //-7.83 line blue
                        std::cout << "Actor passed blue line" << std::endl;
                        stop_flag = true;
                    }
                  }
                }
              ros::Duration(0.01).sleep();
          }


          std::cout << "Sending Data from C++ to Matlab" << std::endl;
          Eigen::MatrixXd item_data_matrix = Eigen::MatrixXd::Zero(item_data.size(),3);
          std_msgs::Float64MultiArray item_data_msg;
          for(int c=0; c<item_data.size(); ++c){
              item_data_matrix(c,0) = item_data[c][0];
              item_data_matrix(c,1) = item_data[c][1];
              item_data_matrix(c,2) = item_data[c][2];
          }



          // Define the message to send
          tf::matrixEigenToMsg(item_data_matrix,item_data_msg);

          _item_pub.publish(item_data_msg);
          std::cout << "Data sent...Waiting the Actor Velocity... " ;


          // Receive Message from Matlab - Message structure:
          // 0 - Pos X
          // 1 - Pos Y
          // 2 - Vel X
          // 3 - Vel Y
          /// --- READ DATA FROM MATLAB --- ///
          /*
          MatlabData[0]=0;
          MatlabData[1]=0;
          MatlabData[2]=0;
          MatlabData[3]=0;
*/
          std::cout << "mat " << MatlabData[3] << std::endl;
          ObjVelocity = 0;
          do{
              ros::spinOnce();
              //std::cout << ObjVelocity << std::endl;
              ObjVelocity = MatlabData[3];
              //ObjVelocity = 1;
              ros::Duration(0.01).sleep();
            }while(ObjVelocity == 0);
          std::cout << " Received - Actor Velocity: " << ObjVelocity << std::endl;

          /// Set the virtual robot to the correct position

          // Extract the time needed by actor to arrive to the line
          _THETA = -M_PI;
          dt = 0.01;

          // Actor Path
          init_actor_line_x =  0;                  // useless
          end_actor_line_x  =  MatlabData[0];      // x position of the actor
          init_actor_line_y = -6.5;                // green line
          end_actor_line_y  = -1.723;              // interaction line (where is the robot)


          // Compute the crossing point according to the velocity actor
          s = fabs(end_actor_line_y - init_actor_line_y);
          t = (s + _MPD) / ObjVelocity;

          // Robot Path
          init_robot_line_x; // Value to compute
          end_robot_line_x = end_actor_line_x;
          init_robot_line_y = end_actor_line_y;
          end_robot_line_y = end_actor_line_y;          RobotVelocity = 1.4; // m/s
          offsetspace = 3.5;
          offsettime;

          // Some offset to move the robot a little bit more
          offsettime = offsetspace/RobotVelocity;

          /// --- Left or Right position of the robot ---- ////
          if(_LRPOS==1)
            init_robot_line_x = end_robot_line_x+RobotVelocity*t;
          else
            init_robot_line_x = end_robot_line_x-RobotVelocity*t;


          std::cout << "percorso " << end_robot_line_x << " = " <<  RobotVelocity<< " * " << t << std::endl;
          std::cout << " l'attore si muove sulla linea x " << end_actor_line_x << std::endl;
          std::cout << " il robot dovrebbe partire da " << init_robot_line_x << " per arrivare a " << end_robot_line_x << std::endl;

          if(_LRPOS==1)
          std::cout << "senza mpd - " << " il robot dovrebbe partire da " << end_actor_line_x+RobotVelocity*s/ObjVelocity << " per arrivare a " << end_actor_line_x << std::endl;
          else
          std::cout << "senza mpd - " << " il robot dovrebbe partire da " << end_actor_line_x-RobotVelocity*s/ObjVelocity << " per arrivare a " << end_actor_line_x << std::endl;

          std::cout << " mpd " << _MPD << std::endl;

          /// Virtual Robot definition t=0
          virtualrobot_config_t.clear();
          virtualrobot_config.clear();
          virtualrobot_config_t.push_back(init_robot_line_x);
          virtualrobot_config_t.push_back(init_robot_line_y);
          virtualrobot_config_t.push_back(_THETA);
          virtualrobot_config.push_back(virtualrobot_config_t);

          std::cout << _LRPOS << std::endl;
          // Initial Configuration
          if(_LRPOS==1){
              for(unsigned int kk=0; kk<=(int(t/dt)+int(offsettime/dt)); ++kk){
                virtualrobot_config_t.clear();
                // Px(t) = Px(tbefore) + v * (t-tbefore)
                virtualrobot_config_t.push_back(virtualrobot_config[virtualrobot_config.size()-1][0] - RobotVelocity*dt);
                virtualrobot_config_t.push_back(virtualrobot_config[virtualrobot_config.size()-1][1]);
                virtualrobot_config_t.push_back(_THETA);
                virtualrobot_config.push_back(virtualrobot_config_t);
              }
            }
          else{
              for(unsigned int kk=0; kk<=(int(t/dt)+int(offsettime/dt)); ++kk){
                virtualrobot_config_t.clear();
                // Px(t) = Px(tbefore) + v * (t-tbefore)
                virtualrobot_config_t.push_back(virtualrobot_config[virtualrobot_config.size()-1][0] + RobotVelocity*dt);
                virtualrobot_config_t.push_back(virtualrobot_config[virtualrobot_config.size()-1][1]);
                virtualrobot_config_t.push_back(0);
                virtualrobot_config.push_back(virtualrobot_config_t);
              }
            }


          std::cout << "il robot arriva fino a " << virtualrobot_config[virtualrobot_config.size()-1][0] << std::endl;
          std::cout << "orientaz " << virtualrobot_config[virtualrobot_config.size()-1][2] << std::endl;
          std::cout << "L: 0 " << _LRPOS << std::endl;



          /// --- Track the virtual robot --- INTERACTION --- ///
          std::cout << " Tracking control" << robotLeftRight[iter_abs] << std::endl;
          item1.virtual_tracking_control_MPD(virtualrobot_config, robotLeftRight[iter_abs]);

          std::cout << "TEST ENDED: NUMBER " << iter_abs << std::endl;
          PressEnterToContinue();

      }
    }
    return 0;
}

