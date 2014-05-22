#include <Eigen/Dense>
using namespace Eigen;

class MoCapMessenger
{
    private:
        double _item_position[3];
        double _item_orientation[4];
        double _item_yaw_angle;
        double tangent;
        double R11, R12, R13, R21, R22, R23, R31, R32, R33;
        std::vector<std::vector <double> > TMatrix;
        std::vector<double> Trow1, Trow2, Trow3;
        std::vector<double> item_XY_YAW_data;
        std::vector<double> item_XY_velocity_data;
        Matrix3f worldRrobot;
        Matrix3f robotRoffset;
        Matrix3f worldRoffset;
        Vector3f angles;


    public:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
        MoCapMessenger();
        ~MoCapMessenger();
        void callbackFunction( const geometry_msgs::TransformStamped& tf);
        double * get_item_position();
        double * get_item_orientation();
        std::vector<double> item_XY_YAW_configuration();
        std::vector<double> item_XY_velocity(double, double, int);
        std::vector<double> item_XY_YAW_configuration_OFFSET(double offset);

};

MoCapMessenger::MoCapMessenger(){
    _item_yaw_angle = 0;
    worldRoffset = Matrix3f::Zero();
    worldRrobot = Matrix3f::Zero();
    robotRoffset = Matrix3f::Zero();
    angles = Vector3f::Zero();
    }

MoCapMessenger::~MoCapMessenger(){}

void MoCapMessenger::callbackFunction( const geometry_msgs::TransformStamped& tf){
                geometry_msgs::Transform t = tf.transform;
                std::string name_id = tf.child_frame_id;
                _item_position[0] = t.translation.x;
                _item_position[1] = t.translation.y;
                _item_position[2] = t.translation.z;
                _item_orientation[0] = t.rotation.x;
                _item_orientation[1] = t.rotation.y;
                _item_orientation[2] = t.rotation.z;
                _item_orientation[3] = t.rotation.w;
}

double * MoCapMessenger::get_item_position(){return _item_position;}

double * MoCapMessenger::get_item_orientation(){return _item_orientation;}

std::vector<double> MoCapMessenger::item_XY_YAW_configuration(){
    item_XY_YAW_data.clear();

    /// Compute the rotation matrix from quaternion
    /*
    Rrow1.clear();  Rrow2.clear();  Rrow3.clear();  RMatrix.clear();
    R11 = 2 * (_item_orientation[3]*_item_orientation[3] + _item_orientation[0]*_item_orientation[0]) - 1;
    R12 = 2 * (_item_orientation[0]*_item_orientation[1] - _item_orientation[3]*_item_orientation[2]);
    R13 = 2 * (_item_orientation[0]*_item_orientation[2] + _item_orientation[3]*_item_orientation[1]);
    R21 = 2 * (_item_orientation[0]*_item_orientation[1] + _item_orientation[3]*_item_orientation[2]);
    R22 = 2 * (_item_orientation[3]*_item_orientation[3] + _item_orientation[1]*_item_orientation[1]) - 1;
    R23 = 2 * (_item_orientation[1]*_item_orientation[2] - _item_orientation[3]*_item_orientation[0]);
    R31 = 2 * (_item_orientation[0]*_item_orientation[2] - _item_orientation[3]*_item_orientation[1]);
    R32 = 2 * (_item_orientation[1]*_item_orientation[2] + _item_orientation[3]*_item_orientation[0]);
    R33 = 2 * (_item_orientation[3]*_item_orientation[3] + _item_orientation[2]*_item_orientation[2]) - 1;

    Rrow1.push_back(R11); Rrow1.push_back(R12); Rrow1.push_back(R13);
    Rrow2.push_back(R21); Rrow2.push_back(R22); Rrow2.push_back(R23);
    Rrow3.push_back(R31); Rrow3.push_back(R32); Rrow3.push_back(R33);

    RMatrix.push_back(Rrow1); RMatrix.push_back(Rrow2); RMatrix.push_back(Rrow3);
*/

    /// Check by output (decomment here)
    /*
    std::cout<< RMatrix[0][0] << " " << RMatrix[0][1] << " " << RMatrix[0][2] << std::endl;
    std::cout<< RMatrix[1][0] << " " << RMatrix[1][1] << " " << RMatrix[1][2] << std::endl;
    std::cout<< RMatrix[2][0] << " " << RMatrix[2][1] << " " << RMatrix[2][2] << std::endl;
    std::cout<< "-------------------------" << std::endl;
*/

    /// Set the position XY
    item_XY_YAW_data.push_back(_item_position[0]);
    item_XY_YAW_data.push_back(_item_position[1]);

    /// Compute yaw angle from quaternion
    /// atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3));
    item_XY_YAW_data.push_back(std::atan2(2 * (_item_orientation[3]*_item_orientation[2] + _item_orientation[0]*_item_orientation[1]), 1 - 2 * (_item_orientation[1]*_item_orientation[1] + _item_orientation[2]*_item_orientation[2])));

    return item_XY_YAW_data;
}

std::vector<double> MoCapMessenger::item_XY_YAW_configuration_OFFSET(double offset){
    item_XY_YAW_data.clear();

    /// Compute the rotation matrix from quaternion

    // Rotation matrix robot from Mocap
    worldRrobot(0,0) = 2 * (_item_orientation[3]*_item_orientation[3] + _item_orientation[0]*_item_orientation[0]) - 1;
    worldRrobot(0,1) = 2 * (_item_orientation[0]*_item_orientation[1] - _item_orientation[3]*_item_orientation[2]);
    worldRrobot(0,2) = 2 * (_item_orientation[0]*_item_orientation[2] + _item_orientation[3]*_item_orientation[1]);
    worldRrobot(1,0) = 2 * (_item_orientation[0]*_item_orientation[1] + _item_orientation[3]*_item_orientation[2]);
    worldRrobot(1,1) = 2 * (_item_orientation[3]*_item_orientation[3] + _item_orientation[1]*_item_orientation[1]) - 1;
    worldRrobot(1,2) = 2 * (_item_orientation[1]*_item_orientation[2] - _item_orientation[3]*_item_orientation[0]);
    worldRrobot(2,0) = 2 * (_item_orientation[0]*_item_orientation[2] - _item_orientation[3]*_item_orientation[1]);
    worldRrobot(2,1) = 2 * (_item_orientation[1]*_item_orientation[2] + _item_orientation[3]*_item_orientation[0]);
    worldRrobot(2,2) = 2 * (_item_orientation[3]*_item_orientation[3] + _item_orientation[2]*_item_orientation[2]) - 1;

    // Rotation Matrix offset
    robotRoffset = AngleAxisf(0, Vector3f::UnitX())
        * AngleAxisf(0, Vector3f::UnitY())
        * AngleAxisf(offset, Vector3f::UnitZ());

    // Rotation matrix final
    worldRoffset = worldRrobot * robotRoffset;

    angles = worldRoffset.eulerAngles(0, 1, 2);

    /// Check by output (decomment here)
    /*
    std::cout<< RMatrix[0][0] << " " << RMatrix[0][1] << " " << RMatrix[0][2] << std::endl;
    std::cout<< RMatrix[1][0] << " " << RMatrix[1][1] << " " << RMatrix[1][2] << std::endl;
    std::cout<< RMatrix[2][0] << " " << RMatrix[2][1] << " " << RMatrix[2][2] << std::endl;
    std::cout<< "-------------------------" << std::endl;
*/

    /// Set the position XY
    item_XY_YAW_data.push_back(_item_position[0]);
    item_XY_YAW_data.push_back(_item_position[1]);

    /// Compute yaw angle from quaternion
    /// atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3));

    item_XY_YAW_data.push_back(angles[2]);

    return item_XY_YAW_data;
}

std::vector<double> MoCapMessenger::item_XY_velocity(double x_before, double y_before, int time_before){
    item_XY_velocity_data.clear();

    /// Set the position XY
    /// Instantenous velocity x
    item_XY_velocity_data.push_back((_item_position[0] - x_before)/1);

    /// Compute yaw angle from quaternion
    /// atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3));
    item_XY_YAW_data.push_back(std::atan2(2 * (_item_orientation[3]*_item_orientation[2] + _item_orientation[0]*_item_orientation[1]), 1 - 2 * (_item_orientation[1]*_item_orientation[1] + _item_orientation[2]*_item_orientation[2])));

    return item_XY_YAW_data;
}
