#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include<iostream>
#include <string>
#include <ros/ros.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include "gazebo_msgs/LinkStates.h"

using namespace std;

geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
string name ="anymal::base";


void callback(const gazebo_msgs::LinkStatesConstPtr& msg)
{
        cout<< msg->name[1]<< endl;
        pose = msg->pose[1];
        twist = msg->twist[1];
        
}
int main(int argc, char** argv)
 {
    ros::init(argc, argv, "state_estimator"); //The name of the node
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, callback);
    ros::Rate loop_rate(10);
    ros::Publisher pub_pose = n.advertise<geometry_msgs::Pose>("q_lin",1);
    ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("q_ang",1);

    // message declarations
    
    while (ros::ok()) {
        pub_pose.publish(pose);

        pub_twist.publish(twist);

       ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}