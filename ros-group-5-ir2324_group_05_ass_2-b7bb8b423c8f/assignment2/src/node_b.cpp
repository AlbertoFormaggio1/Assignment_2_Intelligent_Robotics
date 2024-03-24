//
// Created by alberto on 10/01/24.
//
#include "ros/ros.h"
#include "assignment2/detection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <tf/transform_listener.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
// To move the head
#include <assignment2/headMovement.h>


ros::NodeHandle *nh;

/**
* @Brief this function saves an image
* @param iter number of head pose taken by the head
*/
void save_img(int iter) {
    ROS_INFO("Saving image...");
    // Define ROS topic from where TIAGo publishes images
    image_transport::ImageTransport it(*nh);
    // use compressed image transport to use less network bandwidth
    image_transport::TransportHints transportHint("compressed");
    //take the image
    sensor_msgs::ImageConstPtr imgMsg = ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/rgb/image_raw", *nh);
    ROS_INFO("Got image message");
    cv_bridge::CvImagePtr cvImgPtr;
    //conversion into openCv matrix
    cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
    //save the image
    cv::imwrite(
            "/home/local/formalb15485/catkin_ws_ass/src/assignment2/images/robot_camera_" + std::to_string(iter) +
            ".jpg",
            cvImgPtr->image);
    ROS_INFO("Saved image");
}


/**
*@Brief gets the Apriltags seen by Tiago's camera, moves the head in order to detect all the possible Apriltags and then sends back related id and the position of each detected Apriltag.  
@param req 
@param res response sent back to teh user, containing a vector of ids and a vector of pose
*/
bool get_pose(assignment2::detection::Request &req, assignment2::detection::Response &res) {

    ROS_INFO("Node b started");
    //initialize parameter for the detection and vectors to keep apriltag ids and poses
    ros::NodeHandle n;
    boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> poses_ptr;
    apriltag_ros::AprilTagDetectionArray detections;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::vector<int> ids;
    std::vector<geometry_msgs::Pose> camera_poses;
    tf::Pose pose;
    std::vector<geometry_msgs::Pose> obj_poses;
    assignment2::headMovement srv;
    //call the client to ask head motion
    ros::ServiceClient client = nh->serviceClient<assignment2::headMovement>("move_head");

    // Make the robot look towards the table
    srv.request.mode_lr = 2; //Center
    srv.request.mode_ud = 0; //Down
    client.call(srv);

    try {
        for (int iter = 1; iter <= 3; iter++) {

            //change the position of the head, by looking of right and on the left
            if (iter == 2 || iter == 3) {
                //Create the message with the information to send to the server
                srv.request.mode_lr = iter - 2; //Look right at iteration 2 and left at iteration 3
                srv.request.mode_ud = 2; //Keep the head down
                ROS_INFO("Calling head server");
                if (client.call(srv))
                    ROS_INFO("The head server returned %d", srv.response.state);
                else
                    ROS_INFO("The head server was not able to accomplish the task");
                if(iter == 3){
                    srv.request.mode_lr = 1; //Now that you're at the center, go left again
                    srv.request.mode_ud = 2; //Keep the head still by looking down
                    ROS_INFO("Calling head server");
                    if (client.call(srv))
                        ROS_INFO("The head server returned %d", srv.response.state);
                    else
                        ROS_INFO("The head server was not able to accomplish the task");
                }
            }


            listener.waitForTransform("base_footprint", "xtion_rgb_optical_frame", ros::Time(0), ros::Duration(2.0));
            listener.lookupTransform("base_footprint", "xtion_rgb_optical_frame", ros::Time(0), transform);

            poses_ptr = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", n);

            if (poses_ptr != NULL) {
                detections = *poses_ptr;
            }

            //number of detections found
            ROS_INFO("The camera found %d objects at iter %d", detections.detections.size(), iter);
            //this parameter keeps track of the number of detections kept
            int s = 0;
            for(int i = 0; i < detections.detections.size(); i++) {
                bool enter = true;
                // idd is the id of the current object
                int idd = detections.detections[i].id[0];
                //ROS_INFO("id found: %d", idd);
                for (int j = 0; j < ids.size(); j++) {
                    // If the id was already added to ids, then don't do anything
                    if (ids[j] == idd) {
                        enter = false;
                    }
                }
                //if the ids has not been detected yet the store it and its position
                if (enter) {
                    //ROS_INFO("id is ok");

                    ROS_INFO("Pose Robot frame for id [%d]: %f, %f, %f", idd, detections.detections[i].pose.pose.pose.position.x,
                             detections.detections[i].pose.pose.pose.position.y,
                             detections.detections[i].pose.pose.pose.position.z);

                    s++;
                    //take the pose w.r.t the world reference frame
                    geometry_msgs::Pose world_poses;
                    tf::poseMsgToTF(detections.detections[i].pose.pose.pose, pose);
                    //ROS_INFO("ID SAVED %d ", detections.detections[i].id[0]);
                    pose = transform * pose;
                    tf::poseTFToMsg(pose, world_poses);

                    ROS_INFO("Pose Odom frame for id [%d]: %f %f %f", idd, world_poses.position.x, world_poses.position.y,
                             world_poses.position.z);
                    //sending poses
                    obj_poses.push_back(world_poses);
                    //sending ids
                    ids.push_back(idd);
                }
            }
            //print the number of id sent
            //ROS_INFO("ID KEPT %d", s);
            //image acquisition
        }

        // Before going back you need to make the robot head look back to the center
        assignment2::headMovement srv;
        srv.request.mode_lr = 0; //Look back at the center
        srv.request.mode_ud = 2; //Head still

        client.call(srv);

        srv.request.mode_lr = 2; //Look back at the center
        srv.request.mode_ud = 1; //Head still

        client.call(srv);

        res.ids = ids;
        //ROS_INFO("ID sent %d", obj_poses.size());
        res.pose_objects = obj_poses;


        ROS_INFO("Node b done");
        return true;
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "get_pose");
    ros::NodeHandle n;
    nh = &n;
    ros::ServiceServer service = n.advertiseService("get_pose", get_pose);
    ROS_INFO("Ready to get the pose of the objects...");
    ros::spin();
    return 0;
}
