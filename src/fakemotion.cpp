/// Tbot1

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
#include "API_Turtlebot.cpp"



using namespace Eigen;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "fakemotion");
    //ros::start();
    MoCapMessenger mocapTBot;
    MoCapMessenger mocapActor;
    ros::Rate(2000);

    /// Check Robot init Pos
    std::vector<double> Actor_configuration;
    std::vector<double> TTBot_configuration;
    int check_event;
    mocapActor.sub = mocapActor.n.subscribe("/vicon/helmetLarge/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapActor);
    mocapTBot.sub = mocapTBot.n.subscribe("/vicon/Robot1/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);

    ros::NodeHandle chatmatlabnode;
    ros::Subscriber dataMatlabCppInitRobotPose;

    ros::Rate(2000);

    dataMatlabCppInitRobotPose = chatmatlabnode.subscribe("InitRobPos", 2000, getDataMatlab);

    /// Initial definiton of the parameters
    TurtlebotMotion tbm;
    double velocity_input;
    double starting_point_x, end_point_x, starting_point_y, end_point_y;
    int chatflag=1;
    double linecrossingYpos = -4.19;
    bool exitflag;


    double timenow = time(NULL)+0.5;
    while(time(NULL)<timenow){
        ros::spinOnce();
        std::cout << "TBot starts config X Y: " << RobotInitPosXY[0] << " " << RobotInitPosXY[1] << std::endl;
        TTBot_configuration = mocapTBot.item_XY_YAW_configuration();
        ros::Duration(0.1).sleep();
    }

    std::cout << "configuration saved.." << std::endl;

    //std::cout << "TB " << TTBot_configuration[0] << " " << TTBot_configuration[2] << std::endl;

    //std::cout << "d " << fabs(TTBot_configuration[0]-RobotInitPosXY[0]) << std::endl;


        check_event = 1;
    if(fabs(TTBot_configuration[0]-RobotInitPosXY[0])>0.15){
        check_event = 1;
        if(TTBot_configuration[0]>RobotInitPosXY[0]){

            /// The robot is more far than its init pos.
            /// Then it must go along -x direction and the orientation is along -x (-pi)

            if(TTBot_configuration[2]>3 || TTBot_configuration[2]<-3){

                /// If the robot is oriented in the correct direction (around -pi), just go
                std::cout << "case 1 " << std::endl;

                velocity_input = 0.55;
                starting_point_x = TTBot_configuration[0];
                end_point_x = RobotInitPosXY[0];
                starting_point_y = RobotInitPosXY[1];
                end_point_y = RobotInitPosXY[1];
                Line_Following_Control(&tbm, velocity_input, starting_point_x, end_point_x, starting_point_y, end_point_y, check_event);
            }
            if(TTBot_configuration[2]>-0.3 && TTBot_configuration[2]<0.3){

                /// If the robot is oriented in the other way, then rotate it and then move
                std::cout << "case 2 " << std::endl;

                /// Rotate it
                exitflag = true;
                while(exitflag){
                    ros::spinOnce();
                    TTBot_configuration = mocapTBot.item_XY_YAW_configuration();
                    RotateRobot(&tbm);
                    if(TTBot_configuration[2]>3.12 || TTBot_configuration[2]<-3.12)
                        exitflag=false;
                }

                /// Move it
                velocity_input = 0.55;
                starting_point_x = TTBot_configuration[0];
                end_point_x = RobotInitPosXY[0];
                starting_point_y = RobotInitPosXY[1];
                end_point_y = RobotInitPosXY[1];
                Line_Following_Control(&tbm, velocity_input, starting_point_x, end_point_x, starting_point_y, end_point_y, check_event);
            }
        }
        else{
            if(TTBot_configuration[0]<RobotInitPosXY[0]){

                /// The robot position is lesser than goal
                /// In this case, since we are in robot1 and the robot goes alwyas along -x, we need to rotate it correctly
                if(TTBot_configuration[2]>-0.3 && TTBot_configuration[2]<0.3){

                    /// if the robot is already oriented in direction of the goal
                    /// then move it and orient it along -x
                    std::cout << "case 3 " << std::endl;

                    /// Move it
                    velocity_input = 0.55;
                    starting_point_x = TTBot_configuration[0];
                    end_point_x = RobotInitPosXY[0];
                    starting_point_y = RobotInitPosXY[1];
                    end_point_y = RobotInitPosXY[1];
                    Line_Following_Control(&tbm, velocity_input, starting_point_x, end_point_x, starting_point_y, end_point_y, check_event);

                    /// and Rotate it
                    exitflag = true;
                    while(exitflag){
                    ros::spinOnce();
                    TTBot_configuration = mocapTBot.item_XY_YAW_configuration();
                    RotateRobot(&tbm);
                    if(TTBot_configuration[2]>3.12 || TTBot_configuration[2]<-3.12)
                        exitflag=false;
                    }

                }
                if(TTBot_configuration[2]>3 || TTBot_configuration[2]<-3){

                    /// In this case the robot is oriented in the other side.
                    /// Then rotate the robot, move it and rotate it again because the robot1 move along -x
                    std::cout << "case 4 " << std::endl;

                    exitflag = true;
                    while(exitflag){
                    ros::spinOnce();
                    TTBot_configuration = mocapTBot.item_XY_YAW_configuration();
                    RotateRobot(&tbm);
                    if(TTBot_configuration[2]>-0.03 && TTBot_configuration[2]<0.03)
                        exitflag=false;
                    }

                    velocity_input = 0.55;
                    starting_point_x = TTBot_configuration[0];
                    end_point_x = RobotInitPosXY[0];
                    starting_point_y = RobotInitPosXY[1];
                    end_point_y = RobotInitPosXY[1];
                    Line_Following_Control(&tbm, velocity_input, starting_point_x, end_point_x, starting_point_y, end_point_y, check_event);

                    exitflag=true;
                    while(exitflag){
                        ros::spinOnce();
                        TTBot_configuration = mocapTBot.item_XY_YAW_configuration();
                        RotateRobot(&tbm);
                        if(TTBot_configuration[2]>3.12 || TTBot_configuration[2]<-3.12)
                            exitflag=false;
                    }

                }
            }
        }
    }



    std::cout << "fake motion.." << std::endl;

    while(chatflag==1){
        ros::spinOnce();
        Actor_configuration = mocapActor.item_XY_YAW_configuration();

        //std::cout << Actor_configuration[1] << " " << linecrossingYpos << std::endl;

        if(Actor_configuration[1]>linecrossingYpos){
            chatflag=0;
        }

    }

    std::cout << "move.." << std::endl;

    /// Line: 1
    velocity_input = -0.75;
    Move_Along_a_Line (&tbm, velocity_input, 0, 4);


    return 0;
}
