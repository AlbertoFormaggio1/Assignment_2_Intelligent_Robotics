#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <control_msgs/PointHeadAction.h>
#include <image_transport/image_transport.h>
#include <assignment2/headMovement.h>
#include <sensor_msgs/CameraInfo.h>

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;
ros::NodeHandle* nh;

void createClient(PointHeadClientPtr& actionClient){
    ROS_INFO("Creating action client to head controller ...");
    actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );
    actionClient->waitForServer(ros::Duration(2.0));
}

bool moveHeadCallback(assignment2::headMovement::Request &req, assignment2::headMovement::Response &res){
    // Adapting the tutorial found at http://docs.ros.org/en/indigo/api/look_to_point/html/look__to__point_8cpp_source.html
    PointHeadClientPtr pointHeadclientPtr;
    pointHeadclientPtr.reset( new PointHeadClient("/head_controller/point_head_action") );
    pointHeadclientPtr->waitForServer();

    geometry_msgs::PointStamped pointStamped;

    pointStamped.header.frame_id = "/xtion_rgb_optical_frame";
    pointStamped.header.stamp    = ros::Time::now();

    // select point where to look with respect to current position
    double x = 0.0;
    double y = 0.0;
    double Z = 1.0;
    // If you need to raise the head, change the coordinates
    if(req.mode_ud == 1)
        y = -0.8; //Head up
    else if(req.mode_ud == 0)
        y = 0.8; //Head down
    else if(req.mode_ud == 2)
        y = 0; //Head still

    // If you need to turn the head, then change the coordinates appropriately
    if(req.mode_lr == 1)
        x = -0.3; //Left
    else if(req.mode_lr == 0)
        x = 0.3; //Right
    else if(req.mode_lr == 2)
        x = 0.0; //Head still

    pointStamped.point.x = x * Z;
    pointStamped.point.y = y * Z;
    pointStamped.point.z = Z;

    // build the action goal
    control_msgs::PointHeadGoal goal;
    goal.pointing_frame = "/xtion_rgb_optical_frame";
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 0.25;
    goal.target = pointStamped;

    ROS_INFO("Sending request to move the head");
    pointHeadclientPtr->sendGoal(goal);
    // Wait for the action to be accomplished
    ros::Duration(2).sleep();

    res.state = true;

    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "move_head_server");
    ros::NodeHandle n;
    nh = &n;
    ros::ServiceServer server = n.advertiseService("move_head", moveHeadCallback);

    createClient(pointHeadClient);

    ros::spin();
    return 0;
}