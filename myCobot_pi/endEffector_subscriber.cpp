#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

#include<sstream>  
#include <vector>
#include <array>
#include <string>

//TODO: change std_msgs
void endEffectorCallback(const sensor_msgs::JointState msg){
    //float <vector> pos = msg->position;

    //std::vector<float> position_arr = msg.position;
    ROS_INFO("End effector: [%f]", msg.position[5]);
}

int main(int argc, char **argv){

    std::stringstream ss;

    ros::init(argc, argv, "joint_states_subscriber");
    ros::NodeHandle n;

    //ros::Subscriber joint_states = n.subscribe<sensor_msgs::JointState>("/joint_states", 1000, printEndEffector);
    
    ros::Subscriber joint_states = n.subscribe("/joint_states", 1000, endEffectorCallback);


    sensor_msgs::JointState jointState; //create an object of type "jointState"
    ros::spin();

    return 0;

}//end of main