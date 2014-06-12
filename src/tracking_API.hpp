
int Tracking_Control(std::string _item_name){

    MoCapMessenger mocapTBot;
    PathTrajectory pathobject;
    Robulab10 Robot;

    mocapTBot.sub = mocapTBot.n.subscribe("/evart/"+ _item_name.c_str() + "/PO", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);
    std::vector<double> Robot_configuration;

    timenowdouble = time(NULL)+0.5;
    while(time(NULL)<timenowdouble)
        ros::spinOnce();

    while(1){
        ros::spinOnce();
        Robot_configuration = mocapTBot.item_XY_YAW_configuration_OFFSET(-3.0551);
        ros::Duration(0.005).sleep();
      }




/*
    /// Open the connection with the Robot
    Robot.establish_connection();

    // Active signal (CTRL + C interrupt)
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);


    std::vector<double> Robot_configuration;
    std::vector<double> X_DOT, YAW_DOT;
    std::vector<double> Actor_configuration;

    //mocapTBot.sub = mocapTBot.n.subscribe("/vicon/Robot1/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);
    //mocapActor.sub = mocapActor.n.subscribe("/vicon/helmetLarge/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapActor);
    mocapTBot.sub = mocapTBot.n.subscribe("/evart/RobulabC/PO", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);
    mocapActor.sub = mocapActor.n.subscribe("/evart/RobulabC/PO", 2000, &MoCapMessenger::callbackFunction, &mocapActor);

    //ros::Rate loop_rate(1000);
    ros::NodeHandle node_message;
    //ros::Subscriber sub = node_message.subscribe("chatter", 1000, chatterCallback);
    ros::Publisher robochat = node_message.advertise<std_msgs::Float64MultiArray>("robotState", 1);
    ros::Publisher goalchat = node_message.advertise<std_msgs::Float64MultiArray>("goalState", 1);

    /// Path Initialization
    std::vector<std::vector<double> > path;
    std::cout << "\n\n-----------------------------------------" << std::endl;
    std::cout << " PATH FOLLOWING CONTROL: LINE PATH" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;

    std::cout << "Creating a line path from ( " << starting_point_x << " ; " << starting_point_y << " ) to (" << end_point_x << " ; " << end_point_y << " ) " << std::endl;
    path = pathobject.get_line_path(starting_point_x, starting_point_y, end_point_x, end_point_y, 1000);
    std::cout << "Line Path Created." << std::endl;

    /// Path Following Control - LINE

    unsigned int config_t = 0;
    bool goal_reached = false;
    double distance_point_to_point;
    double c, l, omega;
    double theta_desired, theta_tilda, u, v;
    double TTBot_yaw;
    int stop_flag = 0;
    double k2, k3;
    double linecrossingYpos = -4.19;
    double timenowdouble;
    double angle_robot_nearestpointpath;
    double _THREESHOLD = 0.05;

    u = 0;
    v = 0.8;

    /// Definition of the parameters (c and gains)
    /// k2: influences position precision
    /// k3: influences orientation precision

    //changing the gain according to linear velocity
    if(v<0.35){
      k2=8;
      k3 =4*sqrt(2);
      }
    else{
        if(v<0.75){
            k2=6;
            k3 =2.2*sqrt(2);
          }
        else if(v<0.85){
               k2=6;
               k3 =1.75*sqrt(2);
          }
        else if(v<1){
               k2=2.85;
               k3 =1.75*sqrt(2);
          }
    }
    c = 0;

    /// Delay before to get information from MoCap (to avoid null data)
    std::cout << "... Initilization ..." << std::endl;
    std::cout << "... Gain - k2: " << k2 << " k3: " << k3 << std::endl;
    timenowdouble = time(NULL)+0.5;
    while(time(NULL)<timenowdouble)
        ros::spinOnce();

    std_msgs::Float64MultiArray robotState;
    std_msgs::Float64MultiArray goalState;

    // The message is structured such that
    // 0: x position
    // 1: y position
    // 2: rot_z orientation
    // 3: velocity control
    // 4: omega control
    robotState.data.resize(5);

    // The message is structured such that
    // 0: x position INIT
    // 1: y position INIT
    // 2: x position GOAL
    // 3: y position GOAL
    // 4: threeshold area
    goalState.data.resize(5);

    goalState.data[0] = starting_point_x;
    goalState.data[1] = starting_point_y;
    goalState.data[2] = end_point_x;
    goalState.data[3] = end_point_y;
    goalState.data[4] = _THREESHOLD;

    /// PATH FOLLOWING CONTROL
    while(ros::ok() && goal_reached == false && stop_interrupt==false){

        l = 9999;

        ros::spinOnce();

        /// Read Turtlebot position XY and orientation theta (yaw)
       // Robot_configuration = mocapTBot.item_XY_YAW_configuration();
        //Robot_configuration = mocapTBot.item_XY_YAW_configuration_OFFSET(0.0388);
        Robot_configuration = mocapTBot.item_XY_YAW_configuration_OFFSET(-3.0551);

        /// Read Actor pos XY and yaw angle
        Actor_configuration = mocapActor.item_XY_YAW_configuration();

        /// Fix the orientation setting the offset (if there is)
        TTBot_yaw = Robot_configuration[2];

        /// Set the offset if it exists
        std::cout << TTBot_yaw << std::endl;

        /// Look for the nearest point
        for(unsigned int it=0; it<path.size(); ++it)
        {
        distance_point_to_point = sqrt(pow((Robot_configuration[0] - path[it][0]),2)+pow((Robot_configuration[1] - path[it][1]),2));
        if(distance_point_to_point < l){
            l = distance_point_to_point;
            config_t = it;
            }
        }

        /// Set L and end-condition for a line

        /// Fix the problem of the angles: decide if L or -L
        // (very complicate to explain, I just dreamed it and done)

        angle_robot_nearestpointpath = std::atan2((Robot_configuration[1]-path[config_t][1]),(Robot_configuration[0]-path[config_t][0]));

        if (path[config_t][2]==M_PI){
            if((angle_robot_nearestpointpath <0 && angle_robot_nearestpointpath<= -M_PI) || (angle_robot_nearestpointpath >0 && angle_robot_nearestpointpath<= M_PI))
                l = -1 * l;
        }
        else{
             if(angle_robot_nearestpointpath<-M_PI/2-0.01 && angle_robot_nearestpointpath>=-M_PI){       // if angle negative -> increase it
                angle_robot_nearestpointpath = angle_robot_nearestpointpath + 2*M_PI;
                if(path[config_t][2]<0 && path[config_t][2]>-M_PI){
                    if(angle_robot_nearestpointpath < path[config_t][2] + 2*M_PI){
                        l = -1 * l;
                    }
                }
                else{
                    if(path[config_t][2] > angle_robot_nearestpointpath){
                        l = -1 * l;
                    }
                }
            }
            else{
                if(angle_robot_nearestpointpath>M_PI/2+0.01 && angle_robot_nearestpointpath<=M_PI){      // if angle positive ->
                        if(path[config_t][2]<0 && path[config_t][2]>-M_PI){
                            if(angle_robot_nearestpointpath < path[config_t][2] + 2*M_PI){
                                l = -1 * l;
                            }
                        }
                        else{
                            if(path[config_t][2] > angle_robot_nearestpointpath){
                                l = -1 * l;
                            }
                        }
                    }
                    else{
                            if(path[config_t][2] > angle_robot_nearestpointpath){
                                l = -1 * l;
                            }
                    }
            }

        }

        /// Check if the GOAL is reached
        if ((Robot_configuration[0] > end_point_x-_THREESHOLD) && (Robot_configuration[0] < end_point_x+_THREESHOLD) && (Robot_configuration[1] > end_point_y-_THREESHOLD && Robot_configuration[1] < end_point_y+_THREESHOLD)){
            std::cout << "-- GOAL REACHED -- " << std::endl;
            Robot.move_robot(0,0); // Stop the Robot (if Robulab)
            goal_reached = true;
        }


        /// Check the closest configurations of the path to the Robot
        if(config_t >= path.size()-3)
            config_t = 0;
        else
            config_t = config_t+3;

        theta_desired = path[config_t][2];
        theta_tilda = TTBot_yaw - theta_desired;   /// error

        if(fabs(theta_tilda)>3.14){
            if(theta_tilda < 0)
                theta_tilda = 2*M_PI+theta_tilda;
            else
                theta_tilda = theta_tilda - 2*M_PI;
        }

        /// Non Linear: u = -k2vl(sinthetatilda/thetatilda) - k3|v|thetatilda
        u = -k2 * v * l * (sin(theta_tilda)/theta_tilda) - k3*v*theta_tilda;

        /// Linear Control: u = -k2vl - k3|v|thetatilda
        //u = -k2 * v * l - k3 * v * theta_tilda;

        omega = u + (v*cos(theta_tilda)*c/(1-c*l));

        // Some print out
        /// Print only when the robot moves
        if (check_event==1 || (check_event==0 && Actor_configuration[1]>linecrossingYpos)){
            std::cout << " ----------------------------------------- " << std::endl;
            std::cout << "robot config x: " << Robot_configuration[0] << " y : " << Robot_configuration[1] << " th: " << TTBot_yaw << std::endl;
            std::cout << "goal config x: " << path[config_t][0] << " y: " << path[config_t][1] << " th " << path[config_t][2] << std::endl;
            std::cout << "GOAL x: " << end_point_x << " y: " << end_point_y << std::endl;
            std::cout << "Robot controls - v: " << v << " omega: " << omega << std::endl;
            std::cout << " ----------------------------------------- " << std::endl;
        }

        /// Security Check
        if(omega>1000*100){
            std::cout << "---- SECURITY STOP: omega too high, probably diverging---- " << std::endl;
            return -1;
        }
        else{ /// MOVE!
           Robot.move_robot(v, omega);
        }

        ros::Duration(0.03).sleep();


        // Publish Robot and goal Configuration Data
        robotState.data[0] = Robot_configuration[0];
        robotState.data[1] = Robot_configuration[1];
        robotState.data[2] = Robot_configuration[2];
        robotState.data[3] = v;
        robotState.data[4] = omega;

        //
        robochat.publish(robotState);
        goalchat.publish(goalState);

     }
    std::cout << "out" << std::endl;

    Robot.move_robot(0, 0);

    robotState.data[0] = 999;
    std::cout << "Send END COMMAND " << std::endl;
    for (unsigned int j=0; j<10; ++j)
      robochat.publish(robotState);

*/

    return 0;
}