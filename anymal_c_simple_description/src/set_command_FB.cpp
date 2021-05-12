#include<iostream>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>

using namespace std;

int main(int argc, char** argv)
 {
    ros::init(argc, argv, "set_command"); //The name of the node
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/anymal/floating_base_controller/command", 1);    
    ros::Rate loop_rate(10);
    
    // message declarations
    sensor_msgs::JointState joint_state;
    
    while (ros::ok()) {
        joint_state.name.resize(18);
        joint_state.position.resize(18);
        joint_state.velocity.resize(18);
        joint_state.effort.resize(18);
        
        joint_state.name[0]="world_to_trans_x";
        joint_state.position[0] = -3.93005e-05;
        joint_state.velocity[0] = 0.0;
        joint_state.effort[0] = 0.0;

        joint_state.name[1]="trans_x_to_trans_y";
        joint_state.position[1] = 1.72816e-05;
        joint_state.velocity[1] = 0.0;
        joint_state.effort[1] = 0.0;
        
        joint_state.name[2]="trans_y_to_trans_z";
        joint_state.position[2] =   0.556676;
        joint_state.velocity[2] = 0.0;
        joint_state.effort[2] = 0.0;
        
        joint_state.name[3]="trans_z_to_rot_z";
        joint_state.position[3] = 0.000563285;
        joint_state.velocity[3] = 0.0;
        joint_state.effort[3] = 0.0;
        
        joint_state.name[4]="rot_z_to_rot_y";
        joint_state.position[4] =  -0.00446539;
        joint_state.velocity[4] = 0.0;
        joint_state.effort[4] = 0.0;
        
        joint_state.name[5]="rot_y_to_rot_torso";
        joint_state.position[5] = -1.37294e-05;
        joint_state.velocity[5] = 0.0;
        joint_state.effort[5] = 0.0;
        
        joint_state.name[6]="LF_HAA";
        joint_state.position[6] =  -0.00224275;
        joint_state.velocity[6] = 0.0;
        joint_state.effort[6] = 0.0;
        
        joint_state.name[7]="LF_HFE";
        joint_state.position[7] = 0.4165;
        joint_state.velocity[7] = 0.0;
        joint_state.effort[7] = 0.0;

        joint_state.name[8]="LF_KFE";
        joint_state.position[8] =   -0.764323;
        joint_state.velocity[8] = 0.0;
        joint_state.effort[8] = 0.0;
        
        joint_state.name[9]="LH_HAA";
        joint_state.position[9] =  -0.00259729;
        joint_state.velocity[9] = 0.0;
        joint_state.effort[9] = 0.0;
        
        joint_state.name[10]="LH_HFE";
        joint_state.position[10] = -0.419275;
        joint_state.velocity[10] = 0.0;
        joint_state.effort[10] = 0.0;
        
        joint_state.name[11]="LH_KFE";
        joint_state.position[11] = 0.786255;
        joint_state.velocity[11] = 0.0;
        joint_state.effort[11] = 0.0;
        
        joint_state.name[12]="RF_HAA";
        joint_state.position[12] =  0.00109757;
        joint_state.velocity[12] = 0.0;
        joint_state.effort[12] = 0.0;
        
        joint_state.name[13]="RF_HFE";
        joint_state.position[13] = 0.416916;
        joint_state.velocity[13] = 0.0;
        joint_state.effort[13] = 0.0;
        
        joint_state.name[14]="RF_KFE";
        joint_state.position[14] = -0.765104;
        joint_state.velocity[14] = 0.0;
        joint_state.effort[14] = 0.0;
        
        joint_state.name[15]="RH_HAA";
        joint_state.position[15] =  0.00143594;
        joint_state.velocity[15] = 0.0;
        joint_state.effort[15] = 0.0;
        
        joint_state.name[16]="RH_HFE";
        joint_state.position[16] = -0.419698;
        joint_state.velocity[16] = 0.0;
        joint_state.effort[16] = 0.0;
        
        joint_state.name[17]="RH_KFE";
        joint_state.position[17] =  0.787012;
        joint_state.velocity[17] = 0.0;
        joint_state.effort[17] = 0.0;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}