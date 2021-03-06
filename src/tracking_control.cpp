
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

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

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
#include "tracking_API.hpp"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracking_control");
    ros::NodeHandle _item_node;
    ros::Publisher _item_pub = _item_node.advertise<std_msgs::Float64MultiArray>("ObjData", 2000);



    item_goal item1;

    std::vector<std::vector<double> > matrix;
    std::vector<double> vec;
    vec.push_back(0);
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);

    for(unsigned int i=0; i<8; i++)
      matrix.push_back(vec);

    item1.set_name("head");
    item1.virtual_tracking_control(matrix);

}
