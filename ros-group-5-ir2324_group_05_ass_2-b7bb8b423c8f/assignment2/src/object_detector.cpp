#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/core.hpp>
#include <assignment2/scan_clusterizer.h>
#include <tf/tf.h>
#include <assignment2/laser_detect.h>
#include <tf/transform_listener.h>

ros::NodeHandle* nh;
float cylinder_pose_z = 0.69;

bool detectObstacles(assignment2::laser_detect::Request &req, assignment2::laser_detect::Response &res) {
    // Vector containing the coordinates of the points read by the LiDAR
    std::vector<cv::Point2f> coordinates_scanner;
    // Wait for the first reading of the laser scanner on the /scan topic
    sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", *nh);
    // Read the coordinates and convert them into cartesian coordinates (the result is in coordinates_scanner)
    ScannerClusterizer::read_scanner(msg->ranges, coordinates_scanner, msg->angle_min, msg->angle_increment);

    std::vector<cv::Point2f> coordinates_objects;
    std::vector<float> radii;
    // We compute the number and the location of the cylindrical objects
    ROS_INFO("Detecting objects...");
    int n_objects = ScannerClusterizer::compute_clusters(coordinates_scanner, coordinates_objects, 0.3,
                                                         radii, req.min_radius, req.max_radius);

    ROS_INFO("Number of clusters is %d", n_objects);

    // We need to convert the coordinates from base_laser_link (the (0,0) position for the lidar) to base_link
    // and then they must be converted from base_link to odom
    tf::TransformListener listener;
    tf::StampedTransform transform_odom;
    tf::StampedTransform transform_laser;
    tf::Point tf_point;
    listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("odom", "base_link", ros::Time(0), transform_odom);
    listener.waitForTransform("base_link", "base_laser_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("base_link", "base_laser_link", ros::Time(0), transform_laser);

    ROS_INFO("Got the transforms");

    std::vector<geometry_msgs::Point> cylinder_poses;
    for(int i = 0; i < coordinates_objects.size(); i++)
    {
        // Do the actual conversion between reference frames
        geometry_msgs::Point point;
        point.x = coordinates_objects[i].x;
        point.y = coordinates_objects[i].y;
        tf::pointMsgToTF(point, tf_point);
        tf_point = transform_odom * (transform_laser * tf_point);
        tf::pointTFToMsg(tf_point, point);
        point.z = cylinder_pose_z;

        cylinder_poses.push_back(point);

        ROS_INFO("Cylinder %d added", i);
    }

    ROS_INFO("Written all the coordinates in the result");

    res.cylinder_poses = cylinder_poses;

    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "detect_objects");
    ros::NodeHandle n;
    nh = &n;
    ros::ServiceServer server = n.advertiseService("detect_objects", detectObstacles);

    ros::spin();

    return 0;
}