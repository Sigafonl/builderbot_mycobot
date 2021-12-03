#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>


#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <queue> 

using namespace std;
//using namespace ArmState;

class ArmState{

    public:
        //Joint variables
        double distanceToGoal;
        float jointAngles[6];
        vector<float> jointPoses[6];
        vector<float> goal_coord;

        //A* queues
        priority_queue<double> openList;
        queue<double> closedList;

        //Publisher/Subscriber global variables
        geometry_msgs::Point endEff_coord;
        //geometry_msgs::Point goal_coord;
        geometry_msgs::PoseStamped goal_position;

        //Subscriber variables
        ros::Subscriber joint_angles;
        ros::Subscriber joint_poses;
        ros::Subscriber goal_pose;

        ArmState(ros::NodeHandle *n){
            //Subscribers
            //        number_subscriber = nh->subscribe("/number", 1000, &NumberCounter::callback_number, this);

            joint_angles = n -> subscribe("/joint_states", 1000, &ArmState::jointStateAngleCallback, this);
            //joint_poses = n -> subscribe("/tf", 1000, jointStatePoseCallback);
            goal_pose = n -> subscribe("/move_base_simple/goal", 1000, &ArmState::goalCallback, this);
        }

        /**
         * @brief This method loads the joint state radiants into the jointStates array
         * 
         * @param msg 
         */
        void jointStateAngleCallback(const sensor_msgs::JointState msg){
            
            //Assign each radian angle joint state
            for(int i = 0; i < 6; i++){
                jointAngles[i] = msg.position[i];
            }
        }
        void jointStatePoseCallback(const geometry_msgs::TransformStamped msg){
            vector<float> position;

            //loop through the msg to get the xyz coordinates of each joint 
            for(int i = 0; i < 6; i++){
                ROS_INFO("hello");
            }
            
            //add each vector to the jointPoses array
            //for(int i = 0; i < 6; i++){
                //msg.transforms.transform.translation

                //jointPoses[i].push_back()
                //jointPoses[i] = msg.position[i];
            //}
        }

        /**
         * @brief This method loads the goal pose into the goal_coord vector. Ordered by xyz
         * 
         * @param msg 
         */
        void goalCallback(const geometry_msgs::PoseStamped msg){
            goal_coord.push_back(msg.pose.position.x);
            goal_coord.push_back(msg.pose.position.y);
            goal_coord.push_back(msg.pose.position.z);
            
            //ROS_INFO("GoalCoord: [%f]\n[%f]\n[%f]", goal_coord[0], goal_coord[1], goal_coord[2]);
        }

        /**
         * @brief This method calculates the distance from the end effector to the goal position
         * 
         * @return int 
         */
        int calcEndEff(){
            return 10;
        }

};

int main(int argc, char **argv){

    ros::init(argc, argv, "armState");
    ros::NodeHandle n;

    ArmState armState(&n);
    ros::spin();

    return 0;
}
