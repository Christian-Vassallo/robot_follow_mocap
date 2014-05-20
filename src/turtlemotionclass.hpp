
// Class TurtlebotMotion to control the robot motion

class TurtlebotMotion
{
    private:
        double *lin_vel;
        double *ang_vel;
        unsigned int now, end;
        int _state, _bumper;
        geometry_msgs::Twist _nomotion;

    public:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Subscriber bumper_event;
        geometry_msgs::Twist move_cmd;

        TurtlebotMotion();
        ~TurtlebotMotion();
        void move_turtlebot(unsigned int);
        void check_bumber();
        void set_lin_velocity(double * );
        void set_ang_velocity(double * );
        void bumperCallback(const kobuki_msgs::BumperEvent &bumperMessage);
        void give_trajectory(std::vector<double>, std::vector<double>);
        void move_easy_turtlebot();
        std::vector<double> trapezoidal_velocity(int, int);
        std::vector<double> constant_velocity(int, double);
        std::vector<double> sin_angular_velocity(int);

        void stamp();

};
TurtlebotMotion::TurtlebotMotion(){
        now = 0;
        end = 0;
        _state = 0;
        _bumper = 0;
}

TurtlebotMotion::~TurtlebotMotion(){}

void TurtlebotMotion::stamp(){std::cout<< "OK andata" << std::endl;}

void TurtlebotMotion::bumperCallback(const kobuki_msgs::BumperEvent &bumperMessage)
{
        std::cout << "--- Collision detected!!! ---" << std::endl;
        _bumper = 1;
}


void TurtlebotMotion::set_lin_velocity(double * lin_vel_input){
    lin_vel = lin_vel_input;
}

void TurtlebotMotion::set_ang_velocity(double * ang_vel_input){
    ang_vel = ang_vel_input;
}

void TurtlebotMotion::move_turtlebot(unsigned int delay){

    pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    double velx = lin_vel[0];
    move_cmd.linear.x  = lin_vel[0];
    move_cmd.linear.y  = lin_vel[1];
    move_cmd.linear.z  = lin_vel[2];
    move_cmd.angular.x = ang_vel[0];
    move_cmd.angular.y = ang_vel[1];
    move_cmd.angular.z = ang_vel[2];

    now  = time(NULL);
    end  = now + delay;

    std::cout << now << std::endl;
    while (time(NULL) < end){
            move_cmd.linear.x  = velx;
            pub.publish(move_cmd);
            ros::spinOnce();
            if (_bumper == 1)
               while (_bumper == 1){
                    pub.publish(_nomotion);
                    return;
               }
            else{
                pub.publish(move_cmd);
                //std::cout <<  "Linear vel.x : " << move_cmd.linear.x << " | Angular vel.z : " << move_cmd.angular.z << std::endl;
           }
    }
    std::cout << time(NULL) << std::endl;
}

void TurtlebotMotion::move_easy_turtlebot(){

    pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    double velx = lin_vel[0];
    move_cmd.linear.x  = lin_vel[0];
    move_cmd.linear.y  = lin_vel[1];
    move_cmd.linear.z  = lin_vel[2];
    move_cmd.angular.x = ang_vel[0];
    move_cmd.angular.y = ang_vel[1];
    move_cmd.angular.z = ang_vel[2];

    pub.publish(move_cmd);

}

void TurtlebotMotion::give_trajectory(std::vector<double> _V_LIN_X, std::vector<double> _V_ANG_Z){

    ros::Rate loop_rate(1000);
    unsigned ii = 0;

    pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);


    while (ii<_V_LIN_X.size() && ros::ok()){
            ros::spinOnce();
            if (_bumper == 1)
               while (_bumper == 1){
                    pub.publish(_nomotion);
                    return;
               }
            else{
                move_cmd.linear.x = _V_LIN_X[ii];
                move_cmd.angular.z = _V_ANG_Z[ii];
                pub.publish(move_cmd);
                //std::cout << "[ " << ii << " ] - " <<  "Linear vel.x : " << _V_LIN_X[ii] << " | Angular vel.z : " << _V_ANG_Z[ii] << std::endl;
            }
        ++ii;
        loop_rate.sleep();
    }
}

std::vector<double> TurtlebotMotion::trapezoidal_velocity(int n_samples, int max_vel){

    std::vector<double> vector_of_double;

    for (int j=n_samples-1 ; j>=0; j--){
        if(j<=n_samples/4){
            vector_of_double.push_back(max_vel/(0.25*n_samples) * j);
        }
        if(j>n_samples/4 && j<3*n_samples/4)
            vector_of_double.push_back(max_vel);
        if(j>=3*n_samples/4){
            vector_of_double.push_back(-j*(max_vel/(0.25*n_samples)) + 4*max_vel);
        }
    }

    return vector_of_double;
}

std::vector<double> TurtlebotMotion::constant_velocity(int n_samples, double value){

    std::vector<double> vector_of_double;

    for (int i=0; i<n_samples; i++)
        vector_of_double.push_back(value);

    return vector_of_double;
}

std::vector<double> TurtlebotMotion::sin_angular_velocity(int n_samples){

    std::vector<double> vector_of_double;

    for (int j=0 ; j<n_samples; j++){
    //    vector_of_double.push_back( std::sin( 2*M_PI / n_samples*j ) );
    //    std::cout << vector_of_double[j] << std::endl;
	vector_of_double.push_back(M_PI/2);
    }

    return vector_of_double;
}
