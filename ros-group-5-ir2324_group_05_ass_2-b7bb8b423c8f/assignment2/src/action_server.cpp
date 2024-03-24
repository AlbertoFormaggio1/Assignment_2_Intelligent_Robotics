#include <tf2_ros/buffer.h>
#include <assignment2/Robot.h>
#include <vector>
#include <iostream>


int main(int argc, char** argv){
    ros::init(argc, argv, "server");

    ROS_INFO("Before server starts...");
    RobotActionServer robot("move_around");

    ros::spin();
}