#define EPS 0.001

class PathTrajectory
{
    private:
        std::vector<double> path_pointN_data_x;
        std::vector<std::vector<double> > path_data;

    public:
        PathTrajectory();
        ~PathTrajectory();
        void set_item_x(double);
        void set_item_y(double);
        void set_item_theta(double);
        std::vector<std::vector<double> > get_circle_path(double, double, double, int);
        std::vector<std::vector<double> > get_line_path(double, double, double, double, int);

};

PathTrajectory::~PathTrajectory(){}

PathTrajectory::PathTrajectory(){
    path_pointN_data_x.clear();
    path_data.clear();}

std::vector<std::vector<double> > PathTrajectory::get_circle_path(double radius, double offset_x, double offset_y, int samples){

    path_pointN_data_x.clear();
    path_data.clear();
    double angle;

    for(double theta=0; theta<=2*M_PI; theta=theta+2*M_PI/samples){
        path_pointN_data_x.push_back(offset_x + radius*cos(theta));
        path_pointN_data_x.push_back(offset_y + radius*sin(theta));
        angle = atan2(sin(theta),cos(theta));
        angle = angle + M_PI*0.5;

        if (angle>M_PI+2*M_PI/samples)
            angle = angle - 2*M_PI;

        path_pointN_data_x.push_back(angle);
        path_data.push_back(path_pointN_data_x);
        path_pointN_data_x.clear();
    }

    return path_data;
}

std::vector<std::vector<double> > PathTrajectory::get_line_path(double init_pos_x, double init_pos_y, double end_pos_x, double end_pos_y, int samples){
    double point_x = init_pos_x;
    double point_y = init_pos_y;
    std::cout << "atam2 " << end_pos_y - init_pos_y << " " << end_pos_x - init_pos_x << " " << std::atan2(end_pos_y - init_pos_y, end_pos_x - init_pos_x) << std::endl;
    double delta_x = (end_pos_x-init_pos_x)/samples;
    double delta_y = (end_pos_y-init_pos_y)/samples;
    double point_theta = std::atan2(end_pos_y - init_pos_y, end_pos_x - init_pos_x);
    path_pointN_data_x.clear();
    path_data.clear();

    if (init_pos_x<end_pos_x)
        if(init_pos_y<end_pos_y)
            while(point_x<=end_pos_x && point_y<=end_pos_y){
                //std::cout << "dist x> y> : " << point_x << point_y << std::endl;
                path_pointN_data_x.push_back(point_x);
                path_pointN_data_x.push_back(point_y);
                path_pointN_data_x.push_back(point_theta);

                path_data.push_back(path_pointN_data_x);
                path_pointN_data_x.clear();

                point_x = point_x + delta_x;
                point_y = point_y + delta_y;
            }
        else
            while(point_x<=end_pos_x && point_y>=end_pos_y){
                //std::cout << "dist x> y< : " << point_x << point_y << std::endl;
                path_pointN_data_x.push_back(point_x);
                path_pointN_data_x.push_back(point_y);
                path_pointN_data_x.push_back(point_theta);

                path_data.push_back(path_pointN_data_x);
                path_pointN_data_x.clear();

                point_x = point_x + delta_x;
                point_y = point_y + delta_y;
            }
    else
        if(init_pos_y<end_pos_y)
             while(point_x>=end_pos_x && point_y<=end_pos_y){
                //std::cout << "dist x< y> : " << point_x << point_y << std::endl;
                path_pointN_data_x.push_back(point_x);
                path_pointN_data_x.push_back(point_y);
                path_pointN_data_x.push_back(point_theta);

                path_data.push_back(path_pointN_data_x);
                path_pointN_data_x.clear();

                point_x = point_x + delta_x;
                point_y = point_y + delta_y;
            }
        else
            while(point_x>=end_pos_x && point_y>=end_pos_y){
                //std::cout << "dist x< y<: " << point_x << point_y << std::endl;
                path_pointN_data_x.push_back(point_x);
                path_pointN_data_x.push_back(point_y);
                path_pointN_data_x.push_back(point_theta);

                path_data.push_back(path_pointN_data_x);
                path_pointN_data_x.clear();

                point_x = point_x + delta_x;
                point_y = point_y + delta_y;
            }

    return path_data;
}
