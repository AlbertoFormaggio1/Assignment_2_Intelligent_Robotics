//
// Created by sara on 16/01/24.
//

#include "ros/ros.h"
#include "assignment2/pose_place.h"
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <numeric>
#include <vector>
#include <assignment2/headMovement.h>
#include <tf/transform_listener.h>

ros::NodeHandle *nh;
/**
* @Brief this function gets the image of what the robot is seeing
*/
cv::Mat get_img() {
    ROS_INFO("Saving image...");
    // Define ROS topic from where TIAGo publishes images
    image_transport::ImageTransport it(*nh);
    // use compressed image transport to use less network bandwidth
    image_transport::TransportHints transportHint("compressed");

    sensor_msgs::ImageConstPtr imgMsg = ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/rgb/image_raw", *nh);
    ROS_INFO("Got image message");
    cv_bridge::CvImagePtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
    return cvImgPtr->image;
}

/**
* @Brief this function saves an image locally
*/
void save_img() {
    ROS_INFO("Saving image...");
    cv::imwrite("/home/local/farrsar65258/catkin_ws_ass/src/assignment2/images/robot_camera.jpg", get_img());
    ROS_INFO("Saved image");
}

/**
* @Brief this function gets the correct cylinder position where to perform the place based on the color of the object
*/
bool get_pose_place(assignment2::pose_place::Request &req, assignment2::pose_place::Response &res) {
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<assignment2::headMovement>("move_head");
    assignment2::headMovement srv;
    // Make the robot look towards the table
    srv.request.mode_lr = 2; //Center
    srv.request.mode_ud = 0; //Down
    client.call(srv);

    ROS_INFO("Getting the image to analyze");
    cv::Mat image_robot = get_img();
    ROS_INFO("Image obtained");
    std::vector<std::vector<int>> x_vector;
    std::vector<float> average{0, 0, 0};
    std::vector<float> sum{0, 0, 0};
    float avg_interest;


    std::vector<int> ids{1, 2, 3};
    cv::Mat mat_blue;
    cv::Mat mat_green;
    cv::Mat mat_red;
    cv::Mat imageHSV;

    std::vector<int> xs_blue;
    std::vector<int> xs_green;
    std::vector<int> xs_red;

    // Convert image to HSV
    cv::cvtColor(image_robot, imageHSV, cv::COLOR_BGR2HSV);

    // Compute thresholded images, the output image is a mask with pixel equal to 255 if the value was inside the threshold, 0 otherwise
    cv::inRange(imageHSV, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), mat_blue);
    cv::inRange(imageHSV, cv::Scalar(36, 25, 25), cv::Scalar(70, 255, 255), mat_green);
    cv::inRange(imageHSV, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mat_red);
    ROS_INFO("Images are thresholded");

    std::vector<cv::Mat> matrices{mat_blue, mat_green, mat_red};
    x_vector.push_back(xs_blue);
    x_vector.push_back(xs_green);
    x_vector.push_back(xs_red);

  try {
      //for each image, we take the x of the pixels inside the threshold (which corresponds to the color we want)
      for (int i = 0; i < matrices.size(); i++) {
          for (int j = 0; j < matrices[i].rows; j++) {
              for (int k = 0; k < matrices[i].cols; k++) {
                  if (matrices[i].at<uchar>(j, k) == 255) {
                      x_vector[i].push_back(k);
                  }

              }
          }
      }
      ROS_INFO("All the x have been collected");


      // we compute the sum of x collected for each image
      for (int m = 0; m < x_vector.size(); m++) {
          for (int l = 0; l < x_vector[m].size(); l++) {
              sum[m] = sum[m] + x_vector[m][l];
          }
      }
      ROS_INFO("Sum computed");


      //we compute the average of the x for each image
      ROS_INFO("Computing the average...");
      for (int i = 0; i < sum.size(); i++) {
          average[i] = sum[i] / x_vector[i].size();
          ROS_INFO("The average for image %d is %f", i, average[i]);
      }

      ROS_INFO("The requested id is %d", req.id);
      //for each color I assign the corresponded average found
      if (req.id == 1) {  //blue
          avg_interest = average[0];
      }

      if (req.id == 2) { //green
          avg_interest = average[1];
      }

      if (req.id == 3) { //red
          avg_interest = average[2];
      }

      ROS_INFO("The average of the x of the color I'm interested in is %f", avg_interest);


      //we sort the vector of averages
      if (average[0] > average[1]) {
          std::swap(average[0], average[1]);
      }
      if (average[0] > average[2]) {
          std::swap(average[0], average[2]);
      }
      if (average[1] > average[2]) {
          std::swap(average[1], average[2]);
      }

      // we see in which position the average of interest is in and based on that we assign the corresponded position of the cylinder
      if (avg_interest == average[0]) {
          res.pose_place = req.pillars_poses[0];
      }
      if (avg_interest == average[1]) {
          res.pose_place = req.pillars_poses[1];
      }
      if (avg_interest == average[2]) {
          res.pose_place = req.pillars_poses[2];
      }

      ROS_INFO("The position of the pillar in which the place should be performed is %f %f %f", res.pose_place.x, res.pose_place.y,
               res.pose_place.z);


      srv.request.mode_lr = 2; //Stay in the center
      srv.request.mode_ud = 1; //Look up
      client.call(srv);

      return true;

  }catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      return false;
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_place");
    ros::NodeHandle n;
    nh = &n;
    ros::ServiceServer service = n.advertiseService("pose_place", get_pose_place);
    ROS_INFO("Ready to get the pose of the place point...");
    ros::spin();
    return 0;
}