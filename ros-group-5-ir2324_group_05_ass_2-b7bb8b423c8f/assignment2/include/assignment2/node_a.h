#ifndef ASSIGNMENT2_NODE_A_H
#define ASSIGNMENT2_NODE_A_H

#include <ros/ros.h>

// For calling the action servers
#include <actionlib/client/simple_action_client.h>

// For getting the ids
#include <tiago_iaslab_simulation/Objs.h>

// For storing the coordinates
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <control_msgs/PointHeadAction.h>
#include <vector>

// For calling all the necessary actions or servers defined
#include "assignment2/detection.h"
#include "assignment2/pose_place.h"
#include "assignment2/pickplaceAction.h"
#include "assignment2/laser_detect.h"
#include <assignment2/robotAction.h>

// For transforming the reference frames
#include <tf/transform_listener.h>
#include <stdlib.h>

// To print the feedback based on the enum defined in node_c.h
#include <assignment2/node_c.h>


class NodeA{
protected:
    ros::NodeHandle n_;

    // the order of the indices is blue, green, red
    const std::vector <std::vector<double>> objects_positions = {{8.30761, -1.9986,  -1.5707963},
                                                                 {7.69915, -4.02856, +1.5707963},
                                                                 {7.44911, -1.9486,  -1.5707963}};
// Min and Max radius for a circle to be detected as cylinder
    const float MIN_RADIUS = 0.0;
    const float MAX_RADIUS = 0.3;

// Dimensions of the place table
    const float CYLINDER_RADIUS = 0.21;
    const float CYLINDER_HEIGHT = 0.69;

    // The position of these waypoints (with the corresponding names) is indicated in the report
    std::vector<double> waypoint_1 = {8.5, 0, -M_PI_2};

    // the "neg" waypoint is simply a waypoint with a different orientation based on where the robot is headed
    std::vector<double> waypoint_2 = {8.521, -1.742, -M_PI_2};
    std::vector<double> neg_waypoint_2 = {8.521, -1.742, M_PI};

    std::vector<double> waypoint_3 = {9, -3.5, -M_PI_2};
    std::vector<double> neg_waypoint_3 = {9, -3.5, 1};

    std::vector<double> waypoint_4 = {6.57991 + 4.437482, - 1.359981 -1.106445, 1.480961};
    std::vector<double> neg_waypoint_4 = {6.57991 + 4.437482, - 1.359981 -1.106445, -1.370961};

    std::vector <geometry_msgs::Point> place_cylinder_poses{3};

public:
    /**
     * Moves the robot to the position provided in input
     * @param point vector containing as coordinates (x, y, th): they are associated with the only 3DOFs that the robot can achieve
     * @return true if the operation succeeded
     */
    bool move_to_point(const std::vector<double>& point);

    /**
     * Performs the detection of the april tags placed on the table by also moving tiago's head to capture all the surroundings.
     * Done by calling node_b
     * @param detection_poses Output parameter: poses detected by Tiago in the base_footprint reference frame
     * @param ids Output parameter: The ids associated to the object detected
     * The output params are done in such a way ids[i] <-> detection_poses[i]
     */
    void detection(std::vector <geometry_msgs::Pose> &detection_poses, std::vector<int> &ids);

    /**
     * Performs the pick action by invoking node_c
     * @param obj_poses The poses of the collision objects detected by NodeA::detection
     * @param ids the ids of the collision objects
     * @param id the id of the object that tiago needs to pick
     */
    void pick_object(std::vector <geometry_msgs::Pose> &obj_poses, std::vector<int> &ids, int id);

    /**
     * Places the object by calling node_c
     * @param id the id of the object that is being placed
     * @param cylinder_radius the radius of the cylinder
     * @param cylinder_pos the center of the cylinder over which you should place the object (needed for the creation of the collision object)
     */
    void place_object(int id, float cylinder_radius, geometry_msgs::Pose& cylinder_pos);

    /**
     * Fills place_cylinder_poses, containing the positions of the cylinders that are detected (or hard-coded, if without bonus points part).
     * These points are NOT the center of the cylinders, but the points of each cylinder that are the closest to the robot.
     * The centers of the cylinders are computed only for the cylinder of interest in NodeA::compute_cluster_center
     */
    void fill_cylinder();

    /**
     * Computes the center of the clusters given the closest point of the cylinder selected
     * @param point_cylinder the point that is the closest to the robot of the cylinder where the robot should go
     * @param center_cylinder the center of the cylinder
     */
    void compute_cluster_center(geometry_msgs::Point point_cylinder, geometry_msgs::Point& center_cylinder);

    /**
     * By colling the find_place_pose server, it computes the position of the right cylinder based on which is the color
     * associated to the object the robot is holding
     * @param id the id of the object
     * @param waypoint_place the waypoint where the robot should go (offset from the center)
     * @param cylinder_radius the radius of the cylinder
     * @param cylinder_pos the center of the cylinder
     */
    void get_place_pose(int id, std::vector<double> &waypoint_place, float& cylinder_radius,
                        geometry_msgs::Pose& cylinder_pos);

    /**
     * Calls all the actions to make the robot move in the environment
     * @param id_sequence the sequence of ids given by the human node
     * @return true if succeeded, false otherwise
     */
    void do_routine(std::vector<int> id_sequence);

    /**
     * Routine to handle the feedback sent by node C
     * @param feedback the node c feedback
     */
    static void feedbackCb_pickplace(const assignment2::pickplaceFeedbackConstPtr & feedback);
};




#endif //ASSIGNMENT2_NODE_A_H
