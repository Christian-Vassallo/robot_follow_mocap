

void Move_Along_a_Line (TurtlebotMotion *, double linear_velocity, double yaw_velocity, int delay);
int Circle_Following_Control(double velocity_input, double Ox, double Oy, double radius, int check_event);
int Line_Following_Control(double velocity_input, double starting_point_x, double end_point_x, double starting_point_y, double end_point_y);
void chatterCallback(const std_msgs::Int32::ConstPtr &msg);
void getDataMatlab(std_msgs::Float32MultiArray::ConstPtr& array);
void RotateRobot (TurtlebotMotion *turtlebot);

int fleggs = 0;
double RobotInitPosXY[2];
double ActorPosStart[3];
double MatlabData[4];

static bool stop_interrupt = false;

void my_handler(int s){
           Robulab10 Robot;
           printf("Caught signal %d\n",s);
           stop_interrupt = true;

           Robot.establish_connection();
           Robot.move_robot(0,0);
}

void chatterCallback(const std_msgs::Int32::ConstPtr &msg)
{
      fleggs = msg->data;
}


void getDataMatlab(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int i = 0;
    // print all the remaining numbers
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        MatlabData[i] = *it;
        i++;
    }

return;
}

void RotateRobot (TurtlebotMotion *turtlebot){
    double V_LIN[3] = {0,0,0};
    double V_ANG[3] = {0,0,M_PI/3*2};

    turtlebot->set_lin_velocity(&V_LIN[0]);
    turtlebot->set_ang_velocity(&V_ANG[0]);
    turtlebot->move_easy_turtlebot();
}

void Move_Along_a_Line (TurtlebotMotion *turtlebot, double linear_velocity, double yaw_velocity, int delay){
    double V_LIN[3] = {linear_velocity,0,0};
    double V_ANG[3] = {0,0,yaw_velocity};

    std::cout << "Move forward for " << delay << " seconds..." << std::endl;
    turtlebot->set_lin_velocity(&V_LIN[0]);
    turtlebot->set_ang_velocity(&V_ANG[0]);
    turtlebot->move_turtlebot(delay);
}

int Circle_Following_Control(double velocity_input, double Ox, double Oy, double radius, bool check_event){

    MoCapMessenger mocapTBot;
    MoCapMessenger mocapActor;
    PathTrajectory pathobject;

    std::vector<double> TTBot_configuration;
    std::vector<double> Actor_configuration;
    std::vector<double> X_DOT, YAW_DOT;


    //ros::Rate loop_rate(50);

//    mocapTBot.sub = mocapTBot.n.subscribe("/evart/TBot2/PO", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);

    mocapTBot.sub = mocapTBot.n.subscribe("/vicon/Robot1/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapTBot);


        /// Path Initialization
    std::vector<std::vector<double> > path;
/*
    /// Circle definition
    std::cout << "Radius value [m] (Default value: 1.0): ";
    if (std::cin.peek() == '\n')    // check if next character is newline
        radius = 1.0;                // and assign the default
    else
        std::cin >> radius;
*/

    ///    ------------- Circle Path
    /// get_circle_path returns a vector of vectors with N configurations: [x, y, theta]
    /// First input: radius value (double). Second input: number of samples (integer)
    std::cout << "Creating a circle path of radius " << radius << std::endl;
    path = pathobject.get_circle_path(radius, Ox, Oy, 100);
    std::cout << "Circle Path Created." << std::endl;

    /// Path Following Control
    int timenow;
    unsigned int config_t = 0;
    double distance_point_to_point;
    double c, l, omega;
    double theta_desired, theta_tilda, u, v;
    double TTBot_yaw;
    double k2, k3;

    int stop_flag = 0;

    double startP, endP;

    c = radius;
    u = 0;
    v = velocity_input;

    /// Delay before to get information from MoCap (to avoid null data)
    std::cout << "... Initilization ..." << std::endl;

    timenow = time(NULL)+2;
    while(time(NULL)<timenow)
        ros::spinOnce();

        endP = ros::Time::now().toSec();

    /// PATH FOLLOWING CONTROL
    while(ros::ok() && !stop_flag){

        std_msgs::Int32 msg;
        l = 9999;

        ros::spinOnce();

        startP = endP;
        endP = ros::Time::now().toSec();
        /// Read Turtlebot position XY and orientation theta (yaw)
        TTBot_configuration = mocapTBot.item_XY_YAW_configuration();

        std::cout << "TEMPO : " << endP-startP << std::endl;

        /// Read Actor pos XY and yaw angle
        if(check_event)
            Actor_configuration = mocapActor.item_XY_YAW_configuration();

        /// Fix the orientation setting the offset

        TTBot_yaw = TTBot_configuration[2];

        TTBot_yaw = TTBot_yaw + M_PI/2;

        if(TTBot_yaw>M_PI)
            TTBot_yaw = TTBot_yaw - 2*M_PI;
/*
        if (TTBot_yaw <0)
            TTBot_yaw = TTBot_yaw + M_PI/2;
        else
            TTBot_yaw = TTBot_yaw - M_PI/2;
*/
        //std::cout << "Data : " << TTBot_configuration[0] << " - " << TTBot_configuration[1] << " a: " << TTBot_yaw << std::endl;

        /// Look for the nearest point
        for(unsigned int it=0; it<path.size(); ++it)
        {
        distance_point_to_point = sqrt(pow((TTBot_configuration[0] - path[it][0]),2)+pow((TTBot_configuration[1] - path[it][1]),2));
        if(distance_point_to_point < l){
            l = distance_point_to_point;
            config_t = it;
            }
        }


        /// Set L and end-condition for a circle
        if (sqrt(pow(TTBot_configuration[0] - Ox,2)+pow(TTBot_configuration[1] - Oy,2)) > radius)
            l = -1 * l;

        std::cout << "distance " << sqrt(pow(TTBot_configuration[0] - Ox,2)+pow(TTBot_configuration[1] - Oy,2)) << std::endl;

        /// Gains for a circle
        if (l>0.3){
            k2=20;
            k3 =sqrt(2);
            std::cout << "GAIN INCREASED TO GO CLOSE THE GOAL" << std::endl;
        }
        else{
            k2=8;
            k3 = 2*sqrt(2);
        }


        /// Check the closest configurations of the path to the Robot
        if(config_t >= path.size()-5)
            config_t = 0;
        else
            config_t = config_t+5;

        theta_desired = path[config_t][2];
        theta_tilda = TTBot_yaw - theta_desired;   /// error

        if(fabs(theta_tilda)>3.14){
            if(theta_tilda < 0)
                theta_tilda = 2*M_PI+theta_tilda;
            else
                theta_tilda = theta_tilda - 2*M_PI;
        }

        // Some print out

/*
        std::cout << "theta tilda: " << theta_tilda << " TB " << TTBot_yaw << " Td " << theta_desired << std::endl;
        std::cout << "robot config x: " << TTBot_configuration[0] << " y : " << TTBot_configuration[1] << " th: " << TTBot_yaw << std::endl;
        std::cout << "goal config x: " << path[config_t][0] << " y: " << path[config_t][1] << " th " << path[config_t][2] << std::endl;
        std::cout << "distance " << l << std::endl; // check the sign of L
        std::cout << " x: " << path[config_t][0] << " y : " << path[config_t][1] << "th: " << path[config_t][2] << " config: " << config_t << " dist: " << l << std::endl;
*/
        /* // To analise the differences between the two controls
        std::cout << "Linear: " << -k2 * v * l - k3 * v * theta_tilda << std::endl;
        std::cout << "NL :" << -k2 * v * l * (sin(theta_tilda)/theta_tilda) - k3*v*theta_tilda << std::endl;
        */

        /// Non Linear: u = -k2vl(sinthetatilda/thetatilda) - k3|v|thetatilda
        u = -k2 * v * l * sin(theta_tilda)/theta_tilda - k3*v*theta_tilda;

        /// Linear Control: u = -k2vl - k3|v|thetatilda
        // u = -k2 * v * l - k3 * v * theta_tilda;

        omega = u + (v*cos(theta_tilda)*c/(1-c*l));


        /// Security Check
        if(omega>100){
            std::cout << "---- SECURITY STOP ---- " << std::endl;
            return -1;
        }
        else/// MOVE!
            //turtlebot->pub.publish(turtlebot->move_cmd);
            std::cout << "move" << std::endl;
        //loop_rate.sleep();
        //ros::Duration(0.0025).sleep();

        if(check_event){
            /*
            distance_TBot_Actor = sqrt(pow((TTBot_configuration[0]-Actor_configuration[0]),2)+pow((TTBot_configuration[1]-Actor_configuration[1]),2));
            std::cout << "Actor to Robot: " << distance_TBot_Actor << std::endl;

            if(distance_TBot_Actor < _THREESHOLD){
                std::cout << "-- INTERRUPT RECEIVED ---" << std::endl;
                stop_flag = true;
            }
            */
            stop_flag = msg.data;
        }
    }
    std::cout << "exit" << std::endl;
    return 0;
}

int Line_Following_Control(double velocity_input, double starting_point_x, double end_point_x, double starting_point_y, double end_point_y, int check_event){

    MoCapMessenger mocapTBot;
    MoCapMessenger mocapActor;
    PathTrajectory pathobject;
    Robulab10 Robot;

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

/*
        if(check_event){
            distance_TBot_Actor = sqrt(pow((TTBot_configuration[0]-Actor_configuration[0]),2)+pow((TTBot_configuration[1]-Actor_configuration[1]),2));
            std::cout << "Actor to Robot: " << distance_TBot_Actor << std::endl;

            if(distance_TBot_Actor < _THREESHOLD){
                std::cout << "-- INTERRUPT RECEIVED ---" << std::endl;
                stop_flag = true;
            }
        }
        */

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



    return 0;
}




