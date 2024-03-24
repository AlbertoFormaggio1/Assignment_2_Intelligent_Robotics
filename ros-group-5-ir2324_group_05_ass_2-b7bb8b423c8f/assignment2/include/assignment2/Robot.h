#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment2/robotAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>


class RobotActionServer{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2::robotAction> as_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    std::string action_name_;
    assignment2::robotFeedback feedback_;
    assignment2::robotResult result_;

    const float WORLD_X = - 6.579991;
    const float WORLD_Y = 1.369981;

public:

    RobotActionServer(std::string name): as_(nh_, name, boost::bind(&RobotActionServer::moveAndDetect, this, _1), false), action_name_(name), ac_("move_base", true){
        ROS_INFO("Server starting...");
        as_.start();
    }

    /// function that given a goal returns true or false whether the robot reaches it or not
    /// \param goal point to be reached expressed in coordinates (x,y,th)
    /// \return
    bool moveRobot(const assignment2::robotGoalConstPtr &goal);

    void moveAndDetect(const assignment2::robotGoalConstPtr &goal);

    /// Get the robot position in the global reference frame with (x,y) coordinates
    /// \param x output param x
    /// \param y output param y
    void GetRobotPosition (float& x, float& y);

    /// Publish a feedback with the current robot position
    void publishRobotPosition();

    /// Given the x, y, th from the robot's odometry, returns the position of the robot in the global reference frame defined
    /// in Gazebo.
    /// \param x odometry x
    /// \param y odometry y
    /// \param th odometry th
    /// \param out_x x in Gazebo reference frame
    /// \param out_y y in Gazebo reference frame
    void convertReferenceFrame(float x, float y, float th, float dx, float dy, float &out_x, float &out_y);

        ~RobotActionServer(void){};
    };


#endif //SRC_ROBOT_H
