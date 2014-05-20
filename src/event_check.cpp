
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

#include "turtlemotionclass.hpp"
#include "mocapmessenger.hpp"
#include "trajectoryClass.hpp"
#include "API_Turtlebot.cpp"

#include <fstream>
#include <algorithm>
#include <iterator>

using namespace Eigen;

void chatterCallbackTrigger(const std_msgs::Int32::ConstPtr &msg)
{
      std::cout << "received data " << msg->data << std::endl;
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "event_check");
    ros::NodeHandle chatnode;

    ros::Publisher chatter_pub = chatnode.advertise<std_msgs::Int32>("chatter", 1000);
    ros::Publisher Tbot_XYYaw = chatnode.advertise<std_msgs::Float64MultiArray>("Tbot_XYYaw", 1000);

    //ros::Subscriber chatter_sub = chatnode.subscribe("TOPIC", 1000, chatterCallbackTrigger);

    ros::Rate loop_rate(1000);

    //ros::NodeHandle pubnode_matlab;
    //ros::Publisher chatter_matpub = pubnode_matlab.advertise<std_msgs::Int32>("chatter", 1000);


    MoCapMessenger mocapItem;
    mocapItem.sub = mocapItem.n.subscribe("/evart/TBot1/PO", 2000, &MoCapMessenger::callbackFunction, &mocapItem);

    std::vector<double> posXY;
    Eigen::MatrixXd Tt;;
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(30,2);
    Eigen::VectorXd X = Eigen::VectorXd::Zero(30);
    Eigen::VectorXd Y = Eigen::VectorXd::Zero(30);
    Eigen::VectorXd solX, solY;


    MatrixXd map = MatrixXd::Zero(2,3);
    std_msgs::Float64MultiArray map_info;


    std::vector<double> velX;// = Eigen::VectorXd::Zero(500); ;
    std::vector<double> velY;// = Eigen::VectorXd::Zero(500);
    std::vector<double> posX;// = Eigen::VectorXd::Zero(500);
    std::vector<double> posY;// = Eigen::VectorXd::Zero(500);
    std::vector<double> velMOD;// = Eigen::VectorXd::Zero(500);


    int flag = 0;
    int counter;
    int counter2 = 0;
    double init_interval_time;

/*
    int count = 0;
    while (ros::ok())
    {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    chatter_pub.publish(msg);


    loop_rate.sleep();
    ++count;
    }
*/

    counter = 0;
    T(0,1) = 1; T(1,1) = 1; T(2,1) = 1; T(3,1) = 1; T(4,1) = 1;
    T(5,1) = 1; T(6,1) = 1; T(7,1) = 1; T(8,1) = 1; T(9,1) = 1;

    T(10,1) = 1; T(11,1) = 1; T(12,1) = 1; T(13,1) = 1; T(14,1) = 1;
    T(15,1) = 1; T(16,1) = 1; T(17,1) = 1; T(18,1) = 1; T(19,1) = 1;

    T(20,1) = 1; T(21,1) = 1; T(22,1) = 1; T(23,1) = 1; T(24,1) = 1;
    T(25,1) = 1; T(26,1) = 1; T(27,1) = 1; T(28,1) = 1; T(29,1) = 1;

    std::cout << "attesa" << std::endl;

    double timenow = time(NULL)+3;
    while(time(NULL)<timenow)
        ros::spinOnce();

    std::cout << "attesa2" <<std::endl;


    while(ros::ok()){
        ros::spinOnce();


        posXY.clear();
        posXY = mocapItem.item_XY_YAW_configuration();

        map(0,0) = 0; map(0,1) = 1; map(0,2) = 2;
        map(1,0) = 3; map(1,1) = 4; map(1,2) = 5;

        tf::matrixEigenToMsg(map,map_info );
        std::cout << "Sending: X: " << posXY[0] << " " << "Y: " << posXY[1] << " " << "Th: " << posXY[2] << std::endl;
        Tbot_XYYaw.publish(map_info);

        if(counter==0){
           init_interval_time = ros::Time::now().toSec();
        }

        X(29) = X(28);
        X(28) = X(27);
        X(27) = X(26);
        X(26) = X(25);
        X(25) = X(24);
        X(24) = X(23);
        X(23) = X(22);
        X(22) = X(21);
        X(21) = X(20);
        X(20) = X(19);

        X(19) = X(18);
        X(18) = X(17);
        X(17) = X(16);
        X(16) = X(15);
        X(15) = X(14);
        X(14) = X(13);
        X(13) = X(12);
        X(12) = X(11);
        X(11) = X(10);
        X(10) = X(9);
        X(9) = X(8);
        X(8) = X(7);
        X(7) = X(6);
        X(6) = X(5);
        X(5) = X(4);
        X(4) = X(3);
        X(3) = X(2);
        X(2) = X(1);
        X(1) = X(0);
        X(0) = posXY[0];


        Y(29) = Y(28);
        Y(28) = Y(27);
        Y(27) = Y(26);
        Y(26) = Y(25);
        Y(25) = Y(24);
        Y(24) = Y(23);
        Y(23) = Y(22);
        Y(22) = Y(21);
        Y(21) = Y(20);
        Y(20) = Y(19);

        Y(19) = Y(18);
        Y(18) = Y(17);
        Y(17) = Y(16);
        Y(16) = Y(15);
        Y(15) = Y(14);
        Y(14) = Y(13);
        Y(13) = Y(12);
        Y(12) = Y(11);
        Y(11) = Y(10);
        Y(10) = Y(9);
        Y(9) = Y(8);
        Y(8) = Y(7);
        Y(7) = Y(6);
        Y(6) = Y(5);
        Y(5) = Y(4);
        Y(4) = Y(3);
        Y(3) = Y(2);
        Y(2) = Y(1);
        Y(1) = Y(0);
        Y(0) = posXY[1];


        T(29,0) = T(28,0);
        T(28,0) = T(27,0);
        T(27,0) = T(26,0);
        T(26,0) = T(25,0);
        T(25,0) = T(24,0);
        T(24,0) = T(23,0);
        T(23,0) = T(22,0);
        T(22,0) = T(21,0);
        T(21,0) = T(20,0);
        T(20,0) = T(19,0);

        T(19,0) = T(18,0);
        T(18,0) = T(17,0);
        T(17,0) = T(16,0);
        T(16,0) = T(15,0);
        T(15,0) = T(14,0);
        T(14,0) = T(13,0);
        T(13,0) = T(12,0);
        T(12,0) = T(11,0);
        T(11,0) = T(10,0);
        T(10,0) = T(9,0);
        T(9,0) = T(8,0);
        T(8,0) = T(7,0);
        T(7,0) = T(6,0);
        T(6,0) = T(5,0);
        T(5,0) = T(4,0);
        T(4,0) = T(3,0);
        T(3,0) = T(2,0);
        T(2,0) = T(1,0);
        T(1,0) = T(0,0);
        T(0,0) = ros::Time::now().toSec()-init_interval_time;


        ros::Duration(0.033).sleep();
        std_msgs::Int32 msg;


        if(counter>30){
            solX = T.jacobiSvd( ComputeThinU | ComputeThinV ).solve(X);
            solY = T.jacobiSvd( ComputeThinU | ComputeThinV ).solve(Y);


            posX.push_back(posXY[0]);
            posY.push_back(posXY[1]);

            velX.push_back(solX(0));
            velY.push_back(solY(0));

            velMOD.push_back(sqrt(pow(solX(0),2)+pow(solY(0),2)));

            counter2++;

            std::cout << "X: " << posXY[0] << " " << "Y: " << posXY[1] << " - vel X: " << solX(0) << " - vel Y: " << solY(0) << std::endl;

            //std::cout << "velx: " << solX(0) << " - vely: " << solY(0) << std::endl;

        }

        if(posXY[0]<0 && posXY[1]<0){
            msg.data = 1;
            std::cout << "EVENT" << std::endl;
        }
        else
            msg.data = 7;

        std::cout << "publishing " << msg.data << std::endl;

        //ROS_INFO("%s", flag);

        //std::cout << "publishing: " << msg.data << " " << counter << std::endl;

        chatter_pub.publish(msg);

        counter++;

    }


    std::cout << "Scrivo" << std::endl;
/*
    std::ofstream myfile;
    myfile.open ("example.txt");
    myfile << "x = [" << "\n" << posXE << "]; \n\n\n" << "y = [" << "\n" << posYE << "]; \n\n\n" << "vx = [" << "\n" << velXE << "];\n\n\n" << "vy=[" << "\n" << velYE << "];\n\n\n" << "vel=[" << velMODE << "];" << "hold on; plot(vx,'r'); plot(vy,'y'); plot(vel); " << std::endl;
*/
    std::ofstream myfile;
    myfile.open ("example.txt");
    //myfile << "GLfloat vector[]={";
    //std::copy(velX.begin(), velX.end(), std::ostream_iterator<double>(myfile , ", "));

    myfile << "posx = [" << "\n";  std::copy(posX.begin(), posX.end(), std::ostream_iterator<double>(myfile , ", ")); myfile << "]; \n\n\n";
    myfile << "posy = [" << "\n";  std::copy(posY.begin(), posY.end(), std::ostream_iterator<double>(myfile , ", ")); myfile << "]; \n\n\n";
    myfile << "velx = [" << "\n";  std::copy(velX.begin(), velX.end(), std::ostream_iterator<double>(myfile , ", ")); myfile << "]; \n\n\n";
    myfile << "vely = [" << "\n";  std::copy(velY.begin(), velY.end(), std::ostream_iterator<double>(myfile , ", ")); myfile << "]; \n\n\n";
    myfile << "velMOD = [" << "\n";  std::copy(velMOD.begin(), velMOD.end(), std::ostream_iterator<double>(myfile , ", ")); myfile << "]; \n\n\n";

    myfile << "figure (1); hold on; plot(posx,'r'); plot(posy,'g'); hold off;";
    myfile << "figure (2); hold on; plot(velx,'r'); plot(vely,'g'); plot(velMOD,'k'); hold off;";

    //<< "y = [" << "\n" << posYE << "]; \n\n\n" << "vx = [" << "\n" << velXE << "];\n\n\n" << "vy=[" << "\n" << velYE << "];\n\n\n" << "vel=[" << velMODE << "];" << "hold on; plot(vx,'r'); plot(vy,'y'); plot(vel); " << std::endl;


    myfile.close();

  return 0;
}
