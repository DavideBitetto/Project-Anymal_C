#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <urdf/model.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h> 
 
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/Constraints.h>

#include <sensor_msgs/JointState.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

namespace my_controller_ns
{

class floating_base_controller: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

    public:
    Model* model; 

    Eigen::MatrixXd M_; 
    Eigen::VectorXd c_; 
    Eigen::VectorXd cor_; 
    Eigen::VectorXd g_; 
    Eigen::MatrixXd S_; 
    Eigen::MatrixXd Nc_; 
    Eigen::MatrixXd Jc_; 
    
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
    
    void starting(const ros::Time&);
    
    void stopping(const ros::Time&);
    
    void update(const ros::Time&, const ros::Duration& period);
    
    private:
    /* Definig the timing */
    
    double kp, kv;
    /* Gain Matrices */
    /* Defining q_current, dot_q_current, and tau_cmd */
    
    Eigen::Matrix<double, 18, 1> q_curr;
    Eigen::Matrix<double, 18, 1> dot_q_curr;
    Eigen::Matrix<double, 18, 1> dot_dot_q_curr;
    Eigen::Matrix<double, 18, 1> tau_cmd;
    
    /* Error and dot error feedback */
    Eigen::Matrix<double, 18, 1> err;
    Eigen::Matrix<double, 18, 1> dot_err;
    
    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    Eigen::Matrix<double, 18, 1> command_q_d;         // desired command position 
    Eigen::Matrix<double, 18, 1> command_dot_q_d;       // desired command velocity
    Eigen::Matrix<double, 18, 1> command_dot_dot_q_d;   // estimated desired acceleration command 
    
    
    /* ROS variables */
    ros::NodeHandle n;
    ros::Subscriber sub_command_;
    ros::Publisher pub_err_;
    
    /* Setting Command Callback*/
    
    void setCommandCB (const sensor_msgs::JointStateConstPtr& msg);
    
    std::vector<hardware_interface::JointHandle> joint_handle;
};
}