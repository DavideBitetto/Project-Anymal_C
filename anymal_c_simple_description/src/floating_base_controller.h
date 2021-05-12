#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

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
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/LinkState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include <sensor_msgs/JointState.h>
#include "tf/transform_datatypes.h"

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
    Model* model2;

    
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
    
    void starting(const ros::Time&);
    
    void stopping(const ros::Time&);
    
    void update(const ros::Time&, const ros::Duration& period);

    void state_estimator(const gazebo_msgs::LinkStatesConstPtr& msg);

    void setCommandCB(const sensor_msgs::JointStateConstPtr& msg);
    
    private:
    /* Definig the timing */

    Eigen::MatrixXd M_; 
    Eigen::VectorXd c_; 
    Eigen::VectorXd cor_; 
    Eigen::VectorXd g_; 
    Eigen::MatrixXd S_; 
    Eigen::MatrixXd Nc_; 
    Eigen::MatrixXd Jc_; 
    double kp1, kp2, kp3, kp4, kv1;
    double roll, pitch, yaw;
    /* Gain Matrices */
    /* Defining q_current, dot_q_current, and tau_cmd */
    
    Eigen::Matrix<double, 18, 1> q_curr;
    Eigen::Matrix<double, 18, 1> dot_q_curr;
    Eigen::Matrix<double, 18, 1> dot_dot_q_curr;
    Eigen::Matrix<double, 18, 1> tau_cmd;
   

    Eigen::Matrix<double, 7, 1> q_temp;
    Eigen::Matrix<double, 3, 1> rpy_temp;
    Eigen::Matrix<double, 6, 1> dot_q_temp;
    
    /* Error and dot error feedback */
    Eigen::Matrix<double, 18, 1> err;
    Eigen::Matrix<double, 18, 1> dot_err;
    
    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    Eigen::Matrix<double, 18, 1> command_q_d;         // desired command position 
    Eigen::Matrix<double, 18, 1> command_dot_q_d;       // desired command velocity
    Eigen::Matrix<double, 18, 1> command_dot_dot_q_d;   // estimated desired acceleration command

    Eigen::Matrix<double, 18, 18> Kp;    
    Eigen::Matrix<double, 18, 18> Kv;    
    
    
    
    /* ROS variables */
    ros::NodeHandle n;
    ros::Subscriber sub_command_;
    ros::Publisher pub_err_;
    ros::Subscriber sub_gaze;

    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    /* Setting Command Callback*/
    
    
    std::vector<hardware_interface::JointHandle> joint_handle;
};
}