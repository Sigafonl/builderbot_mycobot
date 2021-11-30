#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <queue> 

using namespace std;

//Variables
double distanceToGoal;
float jointStates[6];

geometry_msgs::Point endEff_coord;
geometry_msgs::Point goal_coord;
geometry_msgs::PoseStamped::ConstPtr goal_pose;

priority_queue<double> openList;
queue<double> closedList;


/**
 * @brief This method loads the joint state radiants into the jointStates array
 * 
 * @param msg 
 */
void jointStateCallback(const sensor_msgs::JointState msg){
    
    //Assign each joint state
    for(int i = 0; i < 6; i++){
        jointStates[i] = msg.position[i];
    }
}

/**
 * @brief This method loads the goal pose into the goal_pose and goal_coord variables
 * 
 * @param msg 
 */
void goalCallback(const geometry_msgs::PoseStamped msg){
    goal_coord = msg.pose.position;

}

/**
 * @brief This method calculates the distance from the end effector to the goal position
 * 
 * @return int 
 */
int calcEndEff(){
    return 10;
}

int main(int argc, char **argv){

    //Subscribers
    ros::init(argc, argv, "armState");
    ros::NodeHandle n;

    ros::Subscriber joint_states = n.subscribe("/joint_states", 1000, jointStateCallback);
    ros::Subscriber goal_pose = n.subscribe("/move_base_simple/goal", 1000, goalCallback);
    
    ros::spin();

    return 0;
}
