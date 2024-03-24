//
// Created by sara on 11/12/23.
//

#include "../include/assignment2/Robot.h"
#include <tf/transform_listener.h>


// rostopic pub -1 /move_base_simple/goal geo
// metry_msgs/PoseStamped
// '{header: {stamp: now, frame_id: "map"},
// pose: {position: {x: 10.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
bool RobotActionServer::moveRobot(const assignment2::robotGoalConstPtr &goal) {
    // wait for the server of the "/move_base" topic to start
    ac_.waitForServer();

    // Set the position in the map to achieve as goal
    move_base_msgs::MoveBaseGoal g;

    g.target_pose.header.frame_id = "map";
    g.target_pose.header.stamp = ros::Time::now();
    g.target_pose.pose.position.x = goal->x;
    g.target_pose.pose.position.y = goal->y;

    tf2::Quaternion q;
    q.setRPY(0, 0, goal->th); // roll=0, pitch=0, yaw
    q.normalize();
    g.target_pose.pose.orientation.x = q.getX();
    g.target_pose.pose.orientation.y = q.getY();
    g.target_pose.pose.orientation.z = q.getZ();
    g.target_pose.pose.orientation.w = q.getW();

    ROS_INFO("Sending goal");

    // sending goal to the server
    ac_.sendGoal(g);

    // The robot is moving, send this information to the client
    RobotActionServer::publishRobotPosition();

    // it waits for the results
    ac_.waitForResult(ros::Duration(45.0));

    //gets the result
    actionlib::SimpleClientGoalState result = ac_.getState();

    // Checks if the robot was able to successfully reach the final position or not. Returns the state accordingly.
    if (result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The robot achieved the final position with success");
        publishRobotPosition();
        return true;
    } else {
        ROS_INFO("The robot could not find a way to reach its goal");
        publishRobotPosition();
        return false;
    }
}


void RobotActionServer::moveAndDetect(const assignment2::robotGoalConstPtr &goal) {
    bool succeded = moveRobot(goal);

    // If there was a problem in the navigation of the robot set the action result to ABORTED and return.
    if (!succeded) {
        as_.setAborted(result_);
        return;
    }

    as_.setSucceeded(result_);
}


void RobotActionServer::publishRobotPosition() {
    float x;
    float y;
    RobotActionServer::GetRobotPosition(x, y);
    feedback_.x = x;
    feedback_.y = y;
    as_.publishFeedback(feedback_);
}

void RobotActionServer::GetRobotPosition(float &x, float &y) {
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        // Wait for a transform from base_link (robot reference frame) to odom (global reference frame)
        listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
        // Get the coordinates of thfeedbacke position (0,0) of the robot in the global reference frame
        float x_rob = transform.getOrigin().x();
        float y_rob = transform.getOrigin().y();
        // Get the yaw of the rotation of the transform
        float th = tf::getYaw(transform.getRotation());
        ROS_INFO("%f", th);

        convertReferenceFrame(x_rob, y_rob, th, RobotActionServer::WORLD_X, RobotActionServer::WORLD_Y, x, y);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void RobotActionServer::convertReferenceFrame(float x, float y, float th, float dx, float dy, float &out_x, float &out_y) {

    float trans[4][4] = {{cos(th), -sin(th), 0, dx},
                         {sin(th), cos(th),  0, dy},
                         {0,       0,        1, 0},
                         {0,       0,        0, 1}};

    float coord[4] = {x, y, 0, 1};

    std::vector<float> res(4);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res[i] = res[i]+(trans[i][j] * coord[j]);
        }
    }

    out_x=res[0];
    out_y=res[1];
}
