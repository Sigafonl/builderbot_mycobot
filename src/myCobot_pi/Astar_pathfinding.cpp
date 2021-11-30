#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

#include<sstream>  
#include <vector>
#include <array>
#include <string>

bool isValidMove(long x, long y, long z){
    return false;
}

bool isDestination(long x, long y, long z){
    return false;
}

double calcHuristic(long x, long y, long z){
    return 0.0;
}

/*
void aStar(){
    if(!isValidMove){

    }

    if(isDestination){

    }
}
*/

int main(int argc, char **argv){
    return 0;
}