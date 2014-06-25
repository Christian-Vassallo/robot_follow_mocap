class item_goal
{
private:
  std::string _item_name;
  double Px, Py, aY;
public:
  item_goal(){};
  ~item_goal(){};
  void set_name(std::string _item_setname){_item_name=_item_setname;};
  std::string get_name(){return _item_name;};
  int mocap_tracking_control();
  int virtual_tracking_control(std::vector<std::vector<double> > &virtual_robot_config);
  int virtual_tracking_control_MPD(std::vector<std::vector<double> > &virtual_robot_config, bool RBT_POS);
};


int item_goal::virtual_tracking_control_MPD(std::vector<std::vector<double> > &virtual_robot_config, bool RBT_POS)
{
    // Init ROS
    ros::NodeHandle node_message;

    // Init Robulab and Mocap class
    Robulab10 Robot;
    MoCapMessenger mocapRobot;

    // Active Mocap to track the robot
    mocapRobot.sub = mocapRobot.n.subscribe("/vicon/robot2/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapRobot);
    std::vector<double> Robot_configuration, Goal_Configuration;
    // Open the connection with the Robot Robulab
    Robot.establish_connection();

    std::vector<double> vitesserobot;

    // Active signal (CTRL + C interrupt)
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    /// Tracking Control - Virtual object

    double vr, wr;
    double e1, e2, e3, theta_tilda;
    double x, y ,a, xr, yr, ar;
    double epsilon, b;
    double k1, k2, k3, k4;
    double _THREESHOLD = 0.15;
    double u1, u2, v, w;
    bool goal_reached = false;
    bool end_vector = false;
    unsigned int iter=0;
    unsigned int end_iter_vector;

    u1 = 0;   u2 = 0;
    v = 0;    w = 0;

    /// Definition of the virtual robot parameter
    vr = 1.4;   // vr will be computed
    wr = 0;

    /// Definition of the parameters
    epsilon = 0.5;   // defined positive and constant
    b = 0.5;        // defined positive

    /// Delay before to get information from MoCap (to avoid null data)
    std::cout << "... Initilization Virtual Tracking..." << std::endl;
/*
    double timenowdouble = time(NULL)+0.15;
    while(time(NULL)<timenowdouble){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
      }
*/
    /// TRACK CONTROL
    while(ros::ok() && goal_reached == false && stop_interrupt==false ){

        ros::spinOnce();

        /// Read Turtlebot position XY and orientation theta (yaw)
        Robot_configuration = mocapRobot.item_XY_YAW_configuration_OFFSET(-1.745+M_PI);

        x = Robot_configuration[0];   // X position
        y = Robot_configuration[1];   // Y position
        a = Robot_configuration[2];   // YAW orientation =

        /// Look where is the robot
        //Goal_Configuration = mocapGoal.item_XY_YAW_configuration();

        if(iter<virtual_robot_config.size()){
            xr = virtual_robot_config[iter][0];//Goal_Configuration[0];   // X position
            yr = virtual_robot_config[iter][1];//Goal_Configuration[1];   // Y position
            ar = virtual_robot_config[iter][2];//Goal_Configuration[2];   // YAW orientation
        }
        else{
            std::cout << "fine del vettore" << std::endl;
            if(end_vector==false){
                end_vector = true;
                end_iter_vector = iter-1;
              }
            xr = virtual_robot_config[end_iter_vector][0];//Goal_Configuration[0];   // X position
            yr = virtual_robot_config[end_iter_vector][1];//Goal_Configuration[1];   // Y position
            ar = virtual_robot_config[end_iter_vector][2];//Goal_Configuration[2];   // YAW orientation
          }

        /// k1 and k2 definition
        k1 = 2 * epsilon * sqrt(wr*wr + b * vr * vr);
        k2 = b * vr;
        k3 = k1;
        k4 = b;

        /// Check if the GOAL is reached
        if ((x - xr < _THREESHOLD) && end_vector==true){
            std::cout << "-- GOAL REACHED AND ROBOT STOPPED -- " << std::endl;
            Robot.move_robot(0,0); // Stop the Robot (if Robulab)
            goal_reached = true;
        }

        /// Compute the error
        e1 =  cos(a) * (xr-x) + sin(a) * (yr-y);
        e2 = -sin(a) * (xr-x) + cos(a) * (yr-y);
        e3 =  ar-a;
        theta_tilda = e3;

        if(fabs(e3)>3.14){
            if(e3 < 0)
                e3 = 2*M_PI+e3;
            else
                e3 = e3 - 2*M_PI;
        }

        /// Non Linear:
        /// u1 = -k1(vr,wr)*e1
        /// u2 = -k4*vr*sin(e3)/e3*e2-k3vr,wr)*e3
        u1 = - k1 * e1;
        u2 = - k4 * vr * (sin(e3) / e3) * e2 - k3 * e3;

        /// Linear Control: u = -k2vl - k3|v|thetatilda
        //u = -k2 * v * l - k3 * v * theta_tilda;

        /// Compute v
        /// u1 = -v * vr *cose3
        /// u2 = wr - w
        v = - u1 + vr * cos(e3);
        w = - u2 + wr;
        if (v>2.5)
            v = 1.4;

        //v = vr * cos(e3) - e1;
        // Some print out
        std::cout << " ----------------------------------------- " << std::endl;
        std::cout << "robot config x: " << x << " y : " << y << " th: " << a << std::endl;
        std::cout << "goal config x: " << xr << " y: " << yr << " th " << ar << std::endl;
        std::cout << "Robot controls - v: " << v << " omega: " << w << std::endl;
        //std::cout << "errors: " << e1 << " " << e2 << " " << e3 << std::endl;
        //std::cout << "u1 " << u1 << " u2 " << u2 << std::endl;
        //std::cout << "ar - a " << ar << " - " << a << " " << e3 << std::endl;
        //std::cout << "v = vr * cos(e3) - k * e1 " << v << " " << vr << " " << cos(e3) << " " << e1 << std::endl;
        //std::cout << " ----------------------------------------- " << std::endl;

        /// Security Check

            // Remember: ROB_POS indicates if the robot is in the left side (1) or right side (0) of the actor
            // Considering that the actor moves along Y line, left is negative and right is positive
            if(RBT_POS==1){
                if(xr<=x){
                    std::cout << "position v r" <<  xr << " "<< x << std::endl;
                    std::cout << "FOLLLLLLLLLLLOWWW RIGHT" << std::endl;
                    Robot.move_robot(v, w);
                  vitesserobot.push_back(v);
                  }
                else{
                    std::cout << "position v r" <<  xr << " "<< x << std::endl;
                  std::cout << "virtual robot behind" << std::endl;
                  }
              }
            else{ // if the robot start from left side, it is in the negative quadrant: farest means more negative
                if(xr>=x){
                    std::cout << "position v r" <<  xr << " "<< x << std::endl;
                    std::cout << "FOLLLLLLLLLLLOWWW LEFT" << std::endl;
                  Robot.move_robot(v, w);
                  vitesserobot.push_back(v);
                  }
                else{
                    std::cout << "position v r" <<  xr << " "<< x << std::endl;

                  std::cout << "virtual robot behind" << std::endl;
                  }
              }


        ros::Duration(0.01).sleep();

        ++iter;
    }
    std::cout << "out" << std::endl;

    Robot.move_robot(0, 0);

   // for(unsigned int pp=0; pp<vitesserobot.size();pp++)
    //  std::cout << vitesserobot[pp] << std::endl;

    return 0;

}

int item_goal::virtual_tracking_control(std::vector<std::vector<double> > &virtual_robot_config)
{
    // Init ROS
    ros::NodeHandle node_message;

    // Init Robulab and Mocap class
    Robulab10 Robot;
    MoCapMessenger mocapRobot;

    // Active Mocap to track the robot
    mocapRobot.sub = mocapRobot.n.subscribe("/vicon/robot2/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapRobot);
    std::vector<double> Robot_configuration, Goal_Configuration;
    // Open the connection with the Robot Robulab
    Robot.establish_connection();

    // Active signal (CTRL + C interrupt)
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    /// Tracking Control - Virtual object

    double vr, wr;
    double e1, e2, e3, theta_tilda;
    double x, y ,a, xr, yr, ar;
    double epsilon, b;
    double k1, k2, k3, k4;
    double _THREESHOLD = 0.15;
    double u1, u2, v, w;
    bool goal_reached = false;
    bool end_vector = false;
    unsigned int iter=0;
    unsigned int end_iter_vector;

    u1 = 0;   u2 = 0;
    v = 0;    w = 0;

    /// Definition of the virtual robot parameter
    vr = 1;   // vr will be computed
    wr = 0;   // equal to zero : moving along a line

    /// Definition of the parameters
    epsilon = 0.75;   // defined positive and constant
    b = 0.75;        // defined positive

    /// Delay before to get information from MoCap (to avoid null data)
    std::cout << "... Initilization Virtual Tracking..." << std::endl;

    double timenowdouble = time(NULL)+0.15;
    while(time(NULL)<timenowdouble){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
      }

    /// TRACK CONTROL
    while(ros::ok() && goal_reached == false && stop_interrupt==false ){

        ros::spinOnce();

        /// Read Turtlebot position XY and orientation theta (yaw)
        Robot_configuration = mocapRobot.item_XY_YAW_configuration_OFFSET(-1.745+M_PI);

        x = Robot_configuration[0];   // X position
        y = Robot_configuration[1];   // Y position
        a = Robot_configuration[2];   // YAW orientation =

        /// Look where is the robot
        //Goal_Configuration = mocapGoal.item_XY_YAW_configuration();

        if(iter<virtual_robot_config.size()){
            xr = virtual_robot_config[iter][0];//Goal_Configuration[0];   // X position
            yr = virtual_robot_config[iter][1];//Goal_Configuration[1];   // Y position
            ar = virtual_robot_config[iter][2];//Goal_Configuration[2];   // YAW orientation
        }
        else{
            std::cout << "fine del vettore" << std::endl;
            if(end_vector==false){
                end_vector = true;
                end_iter_vector = iter-1;
              }
            xr = virtual_robot_config[end_iter_vector][0];//Goal_Configuration[0];   // X position
            yr = virtual_robot_config[end_iter_vector][1];//Goal_Configuration[1];   // Y position
            ar = virtual_robot_config[end_iter_vector][2];//Goal_Configuration[2];   // YAW orientation
          }

        /// k1 and k2 definition
        k1 = 2 * epsilon * sqrt(wr*wr + b * vr * vr);
        k2 = b * vr;
        k3 = k1;
        k4 = b;

        /// Check if the GOAL is reached
        if ((x - xr < _THREESHOLD) && end_vector==true){
            std::cout << "-- GOAL REACHED AND ROBOT STOPPED -- " << std::endl;
            Robot.move_robot(0,0); // Stop the Robot (if Robulab)
            goal_reached = true;
        }

        /// Compute the error
        e1 =  cos(a) * (xr-x) + sin(a) * (yr-y);
        e2 = -sin(a) * (xr-x) + cos(a) * (yr-y);
        e3 =  ar-a;
        theta_tilda = e3;

        if(fabs(e3)>3.14){
            if(e3 < 0)
                e3 = 2*M_PI+e3;
            else
                e3 = e3 - 2*M_PI;
        }

        /// Non Linear:
        /// u1 = -k1(vr,wr)*e1
        /// u2 = -k4*vr*sin(e3)/e3*e2-k3vr,wr)*e3
        u1 = - k1 * e1;
        u2 = - k4 * vr * (sin(e3) / e3) * e2 - k3 * e3;

        /// Linear Control: u = -k2vl - k3|v|thetatilda
        //u = -k2 * v * l - k3 * v * theta_tilda;

        /// Compute v
        /// u1 = -v * vr *cose3
        /// u2 = wr - w
        v = - u1 + vr * cos(e3);
        w = - u2 + wr;

        //v = vr * cos(e3) - e1;
        // Some print out
        std::cout << " ----------------------------------------- " << std::endl;
        std::cout << "robot config x: " << x << " y : " << y << " th: " << a << std::endl;
        std::cout << "goal config x: " << xr << " y: " << yr << " th " << ar << std::endl;
        std::cout << "Robot controls - v: " << v << " omega: " << w << std::endl;
        //std::cout << "errors: " << e1 << " " << e2 << " " << e3 << std::endl;
        //std::cout << "u1 " << u1 << " u2 " << u2 << std::endl;
        //std::cout << "ar - a " << ar << " - " << a << " " << e3 << std::endl;
        //std::cout << "v = vr * cos(e3) - k * e1 " << v << " " << vr << " " << cos(e3) << " " << e1 << std::endl;
        //std::cout << " ----------------------------------------- " << std::endl;

        /// Security Check
        if(w>100 || v>10){
            std::cout << "---- SECURITY STOP: omega too high, probably diverging---- " << std::endl;
            return -1;
        }
        else{ /// MOVE!
          Robot.move_robot(v, w);
        }

        ros::Duration(0.01).sleep();

        ++iter;
    }
    std::cout << "out" << std::endl;

    Robot.move_robot(0, 0);


    return 0;

}


int item_goal::mocap_tracking_control(){

    ros::NodeHandle node_message;
    //ros::Subscriber sub = node_message.subscribe("chatter", 1000, chatterCallback);
    ros::Publisher robochat = node_message.advertise<std_msgs::Float64MultiArray>("robotState", 1);
    ros::Publisher goalchat = node_message.advertise<std_msgs::Float64MultiArray>("goalState", 1);

    MoCapMessenger mocapRobot;
    MoCapMessenger mocapGoal;
    PathTrajectory pathobject;
    Robulab10 Robot;

    mocapRobot.sub = mocapRobot.n.subscribe("/vicon/robot2/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapRobot);
    /// LAAS /// mocapGoal.sub = mocapGoal.n.subscribe("/evart/"+ _item_name + "/PO", 2000, &MoCapMessenger::callbackFunction, &mocapGoal);
    mocapGoal.sub = mocapGoal.n.subscribe("/vicon/"+ _item_name + "/endBone", 2000, &MoCapMessenger::callbackFunction, &mocapGoal);

    std::vector<double> Robot_configuration, Goal_Configuration;

    double timenowdouble = time(NULL)+0.5;
    while(time(NULL)<timenowdouble)
        ros::spinOnce();

    /// Open the connection with the Robot
    Robot.establish_connection();

    // Active signal (CTRL + C interrupt)
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);





    /// Tracking Control - Real object

    double vr, wr;
    double e1, e2, e3, theta_tilda;
    double x, y ,a, xr, yr, ar;
    double epsilon, b;
    double k1, k2, k3, k4;
    double _THREESHOLD = 0.05;
    double u1, u2, v, w;
    bool goal_reached = false;

    u1 = 0;
    u2 = 0;
    v = 0;
    w = 0;

    /// Definition of the virtual robot parameter
    vr = 1; // vr will be computed
    wr = 0;   // equal to zero : moving along a line

    /// Definition of the parameters
    epsilon = 1;   // defined positive and constant
    b = 1;        // defined positive


    /// Delay before to get information from MoCap (to avoid null data)
    std::cout << "... Initilization ..." << std::endl;
    std::cout << "... Gain - k2: " << k2 << " k3: " << k3 << std::endl;
    timenowdouble = time(NULL)+0.5;
    while(time(NULL)<timenowdouble)
        ros::spinOnce();

    std_msgs::Float64MultiArray robotState;
    std_msgs::Float64MultiArray goalState;

    /// TRACK CONTROL
    while(ros::ok() && goal_reached == false && stop_interrupt==false ){

        ros::spinOnce();

        /// Read Turtlebot position XY and orientation theta (yaw)
        Robot_configuration = mocapRobot.item_XY_YAW_configuration_OFFSET(-1.71+M_PI);

        x = Robot_configuration[0];   // X position
        y = Robot_configuration[1];   // Y position
        a = Robot_configuration[2];   // YAW orientation =

        /// Look where is the robot
        Goal_Configuration = mocapGoal.item_XY_YAW_configuration();

        xr = Goal_Configuration[0];//Goal_Configuration[0];   // X position
        yr = Goal_Configuration[1]-2;//Goal_Configuration[1];   // Y position
        ar = -M_PI;//Goal_Configuration[2];   // YAW orientation

        /// k1 and k2 definition
        k1 = 2 * epsilon * sqrt(wr*wr + b * vr * vr);
        k2 = b * vr;
        k3 = k1;
        k4 = b;

        /// Check if the GOAL is reached
        if (((x - xr < _THREESHOLD) && (x - xr < _THREESHOLD) && (a - ar < _THREESHOLD)) && vr==0 ){
            std::cout << "-- GOAL REACHED AND ROBOT STOPPED -- " << std::endl;
            Robot.move_robot(0,0); // Stop the Robot (if Robulab)
            goal_reached = true;
        }

        /// Compute the error
        e1 =  cos(a) * (xr-x) + sin(a) * (yr-y);
        e2 = -sin(a) * (xr-x) + cos(a) * (yr-y);
        e3 =  ar-a;
        theta_tilda = e3;

        if(fabs(e3)>3.14){
            if(e3 < 0)
                e3 = 2*M_PI+e3;
            else
                e3 = e3 - 2*M_PI;
        }

        /// Non Linear:
        /// u1 = -k1(vr,wr)*e1
        /// u2 = -k4*vr*sin(e3)/e3*e2-k3vr,wr)*e3
        u1 = - k1 * e1;
        u2 = - k4 * vr * (sin(e3) / e3) * e2 - k3 * e3;

        /// Linear Control: u = -k2vl - k3|v|thetatilda
        //u = -k2 * v * l - k3 * v * theta_tilda;

        /// Compute v
        /// u1 = -v * vr *cose3
        /// u2 = wr - w
        v = - u1 + vr * cos(e3);
        w = - u2 + wr;

        //v = vr * cos(e3) - e1;
        // Some print out
        /*std::cout << " ----------------------------------------- " << std::endl;
        std::cout << "robot config x: " << x << " y : " << y << " th: " << a << std::endl;
        std::cout << "goal config x: " << xr << " y: " << yr << " th " << ar << std::endl;
        std::cout << "Robot controls - v: " << v << " omega: " << w << std::endl;
        std::cout << "errors: " << e1 << " " << e2 << " " << e3 << std::endl;
        std::cout << "u1 " << u1 << " u2 " << u2 << std::endl;
        std::cout << "ar - a " << ar << " - " << a << " " << e3 << std::endl;
        std::cout << "v = vr * cos(e3) - k * e1 " << v << " " << vr << " " << cos(e3) << " " << e1 << std::endl;
        std::cout << " ----------------------------------------- " << std::endl;
*/
        /// Security Check
        if(w>100 || v>10){
            std::cout << "---- SECURITY STOP: omega too high, probably diverging---- " << std::endl;
            return -1;
        }
        else{ /// MOVE!
         // Robot.move_robot(v, w);
        }

        ros::Duration(0.01).sleep();


    }
    std::cout << "out" << std::endl;

    Robot.move_robot(0, 0);

    robotState.data[0] = 999;
    std::cout << "Send END COMMAND " << std::endl;
    for (unsigned int j=0; j<10; ++j)
      robochat.publish(robotState);

    return 0;
}
