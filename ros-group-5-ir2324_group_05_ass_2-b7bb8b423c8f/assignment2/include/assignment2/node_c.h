#ifndef ASSIGNMENT2_NODE_C_H
#define ASSIGNMENT2_NODE_C_H
#include <map>
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "assignment2/pickplaceAction.h"

// For End Effector control
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#define _USE_MATH_DEFINES // for C++
#include <cmath>

// For converting reference frame
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> GripperClient;
typedef boost::shared_ptr<GripperClient> GripperClientPtr;

class NodeC{
protected:
    ros::NodeHandle n_;
    actionlib::SimpleActionServer<assignment2::pickplaceAction> as_;

    std::string action_name;
    assignment2::pickplaceFeedback feedback_;
    assignment2::pickplaceResult result_;

    // elevation + 7 links position for safe pose (when the robot has the arm turned at 90 degrees) and for close pose
    // (arm close to the body)
    const std::vector<float> SAFE_POSE = {0.34, 0.07, 0.34, -3.13, 1.31, 1.58, 0.0, 0.0};
    const std::vector<float> CLOSE_POSE = {0.12, 0.2, -1.339, -0.2, 1.937, -1.57, 1.37, 0.0};

    // Size of the cube
    const std::map<std::string, double> red_cube_size{{"side", 0.05}};
    // Height and radius of the hexagon
    const std::map<std::string, double> blue_hexagon_size{{"height", 0.1},{"radius", 0.025}};
    // dimensions of the triangle
    const std::map<std::string, double> green_triangle_size{{"x", 0.05}, {"y", 0.065}, {"height", 0.0325}};
    const std::map<std::string, double> golden_obs_size{{"height", 0.22}, {"radius", 0.07}};
    const std::map<std::string, double> place_cylinder_size{{"height", 0.69}, {"radius", 0.21}};
    // Red Green Blue
    const std::vector<std::vector<double>> place_cylinder_poses {{6.57991 + 4.007396, - 1.369981 + 1.015966},
                                                                 {6.57991 + 5.007404, - 1.369981 + 1.015966},
                                                                 {6.57991 + 6.007146, - 1.369981 + 1.015966}};

    // poses w.r.t. gazebo reference frame
    const std::vector<double> table_pose{6.579991 + 1.245143, - 1.369981 - 1.613171, 0.755, 0, 0, 0};
    const std::vector<double> table_dim{0.913, 0.913, 0.04};
    // object enlargement of the base of the objects
    double object_enlargement_base = 0.04;
    double object_enlargement_base_cylinder = 0.15;
    // object enlargement of the height of the objects
    double object_enlargement_height = 0.04;
    //object enlargement cylinder place
    double place_table_enlargement_height = 0.20;
    // length of the gripper
    double gripper_length = 0.21;

public:
    enum MessageCode{
        ERROR_SAFE_POSE,
        ERROR_PICK,
        ERROR_PLACE,
        ERROR_CLOSING,

        SAFE_POSE_REACHED,
        PICK_COMPLETED,
        PLACE_COMPLETED,
        ROBOT_READY,
    };

    NodeC(std::string name): as_(n_, name, boost::bind(&NodeC::manage_request, this, _1), false), action_name(name) {
        ROS_INFO("Server starting...");
        as_.start();
    }

    /**
     * Manages a request to pick or place an object.
     * It adds the collision objects, reaches the safe pose if needed and performs the pick/place action by sending regularly
     * updates to the action client
     * @param goal the goal containing the info about the action to do
     */
    void manage_request(const assignment2::pickplaceGoalConstPtr &goal);

    /**
     * Does the backward kinematics on the given pose, if possible
     * @param goal_pose the goal pose that the arm needs to reach
     * @param frame the reference frame of the pose
     * @return true if the movement was successful, false if it failed in finding a plan or there was a problem during the motion
     */
    bool planAndExecuteInverseKinematics(geometry_msgs::Pose& goal_pose, std::string frame);

    /**
     * Returns the pose of the object that the robot needs to pick
     * @param ids the set of object ids
     * @param object_poses the set of object poses. They are in 1 to 1 correspondence with the ids
     * @param id the id of the object the robot needs to pick
     * @param pose out param: the pose of the object with id "id"
     */
    void find_goal_object_pose(std::vector<int> ids, std::vector<geometry_msgs::Pose> object_poses, int id,
                               geometry_msgs::Pose& pose);

    /**
     * changes the gripper state by opening/closing it
     * @param close if true the gripper will close, if false it will open
     */
    void changeGripperState(bool close);

    /**
     * Executes the pick routine: it moves in the target position at some distance from the object
     * sets the gripper around the object, closes the gripper. the attach must be done by the client
     * @param goal_pose the pose of the object the robot needs to pick
     * @param id the id of the object: needed for getting the size of the object and the type of pick to use
     * @return true if the object was picked (the gripper is around it)
     */
    bool pickRoutine(geometry_msgs::Pose &goal_pose, int id, std::string obj_id);

    /**
     * Lifts or lowers the torso of the robot
     * @param lift lifts the torso if true, lowers the torso otherwise
     * @return true if the movement was successful
     */
    bool lift_torso(bool lift);

    /**
     * reaches the safe position
     * @param open true if the robot should open and put the arm at 90 degrees. False for putting the arm close to the robot's body
     * @return true if the movement had success
     */
    bool reach_safe_pos(bool open);

    /**
     * Remove the collision objects from the scene
     * @param current_scene scene containing the collision objects
     * @param collision_objects collision objects to remove
     */
    void remove_collision_objects(moveit::planning_interface::PlanningSceneInterface &current_scene,
                                  std::vector<moveit_msgs::CollisionObject>& collision_objects);

    /**
     * It plans and exectues a plan by relying on the forward kinematics: given the position of the links, it
     * will put the robot in such position
     * @param target_position a map containing pairs ("name of the link", position of the link)
     * The name is a string, while the position a floating point number
     * @return true if the action had success
     */
    bool planAndExecuteForwardKinematics(const std::map<std::string, double>& target_position);

    /**
     * Creates the collision objects associated with the poses provided in input
     * @param poses set of poses of the collision objects
     * @param ids the set of ids of the objects
     * @param vec output parameter: it will contain the collision objects in the scene
     * @param current_scene the current scene
     * @param obj_id the string id associated to the object that needs to be picked
     */
    void create_collision_object(std::vector<geometry_msgs::Pose>& poses, std::vector<int>& ids,
                                 std::vector<moveit_msgs::CollisionObject>& vec,
                                 moveit::planning_interface::PlanningSceneInterface& current_scene, std::string& obj_id,
                                 int id);

    /**
     * Creates the collision object associated to the table where the robot needs to place the object
     * @param collision_objects output vector that will contain the collision objects after the creation
     * @param current_scene the current scene
     * @param cylinder_pose the pose of the cylinder that needs to be added to the frame. it is in the odom (or map) reference frame
     */
    void create_place_table_collision_object(std::vector<moveit_msgs::CollisionObject>& collision_objects,
                                             moveit::planning_interface::PlanningSceneInterface& current_scene,
                                             geometry_msgs::Pose cylinder_pose, float radius);

    /**
     *
     * @param collision_objects
     * @param current_scene
     * @param id
     * @param cylinder_pose
     */
    void find_cylinder_place(std::vector<moveit_msgs::CollisionObject>& collision_objects,
                             moveit::planning_interface::PlanningSceneInterface& current_scene,
                             int id, geometry_msgs::Pose& cylinder_pose);

    /**
     * Executes the place routine by moving the arm on the table lifted by some cm from the table, lowers the arm
     * and drops the object by opening the gripper
     * @param goal_pose the pose at the center of the cylinder
     * @param id the id of the object that needs to be left on the table (it tells us if the robot is holding it from
     * the side or from the top)
     * @return true if the place routine had success
     */
    bool place_routine(geometry_msgs::Pose goal_pose, int id);

    /**
     * Attaches the object to the tiago reference frame in gazebo
     * @param obj_id the id of the collision object
     */
    void attach(const int id ,moveit::planning_interface::PlanningSceneInterface &current_scene);

    /**
     * Detaches the object from the tiago reference frame
     * @param obj_id id of the collision object
     */
    void detach(const int obj_id);

};

#endif //ASSIGNMENT2_NODE_C_H