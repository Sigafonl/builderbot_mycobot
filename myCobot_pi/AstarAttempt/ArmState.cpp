#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <queue> 

using namespace std;
//using namespace ArmState;

class ArmState{

    public:
        //Joint and goal variables
        double distanceToGoal;
        vector<float> goal_coord; //change to goalPose

        float jointAngles[6];
        vector<string> jointNames = {"base", "link1", "link2", "/link3", "/link4", "/link5", "/link6"};
        vector<float> jointPoses[6];
        

        //A* queues
        priority_queue<float> openList;
        queue<float> closedList;

        //Publisher/Subscriber global variables
        //geometry_msgs::Point endEff_coord;
        //geometry_msgs::Point goal_coord;
        //geometry_msgs::PoseStamped goal_position;

        //Subscriber variables
        ros::Subscriber joint_angles;
        ros::Subscriber joint_poses;
        ros::Subscriber goal_pose;


        ArmState(ros::NodeHandle *n){
            //Subscribers
            //        number_subscriber = nh->subscribe("/number", 1000, &NumberCounter::callback_number, this);
            
            ros::Rate rate(10.0);

            while (n -> ok()){

                try{
                    joint_angles = n -> subscribe("/joint_states", 1000, &ArmState::jointStateAngleCallback, this);
                    goal_pose = n -> subscribe("/move_base_simple/goal", 1000, &ArmState::goalCallback, this);

                    //joint_poses = n -> subscribe("/tf", 1000, &ArmState::jointStatePoseCallback, this);

                    jointStatePoseCallback();
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
            }
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

        /**
         * @brief Loop through the msg to get the xyz coordinates of each joint 
         * 
         * @param msg 
         */
        //void jointStatePoseCallback(const geometry_msgs::TransformStamped msg){
        void jointStatePoseCallback(){
            //Loop through the transforms in order to get each joint translation
            /*
            for(int i = 0; i < 6; i++){
                jointPoses[i].push_back(msg.transform.translation.x);
                jointPoses[i].push_back(msg.transform.translation.y);
                jointPoses[i].push_back(msg.transform.translation.z);

                //ROS_INFO("Joint [%d] : [%f][%f][%f]", i, jointPoses[i][0], jointPoses[i][1], jointPoses[i][2]);
            }*/

            // receiving tf transformations over the wire, and buffers them for up to 10 seconds.
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            
            /*
            for(int i = 0; i < jointNames.size()-1; i++){
                geometry_msgs::TransformStamped currentLink = tfBuffer.lookupTransform(jointNames[i+1], jointNames[i], ros::Time(0));
            
                //add x, y, z of current joint to it's respective vector
                jointPoses[i].push_back(currentLink.transform.translation.x);
                jointPoses[i].push_back(currentLink.transform.translation.y);
                jointPoses[i].push_back(currentLink.transform.translation.z);
            
            }*/

            // Get the transform between two frames
            // First argument is the target frame, the second argument is the source frame
            // We want to transform to the target frame from the source frame.
            // ros::Time(0) gets us the most recent transform.

            //ex. turtle2 is the leader and turtle2 is the follower  
            //transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",ros::Time(0));
            //rosrun tf tf_echo turtle1 turtle2
            //rosrun tf tf_echo link1 base
            geometry_msgs::TransformStamped currentLink = tfBuffer.lookupTransform("base", "link1", ros::Time(0));
            
            ROS_INFO("transform x : [%f]" , currentLink.transform.translation.x);

            //ROS_INFO("transform x : [%f]", jointPoses[0][0]);

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
        double calcDistance(){
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
