#include <ros/ros.h>
#include <vector>
#include <assignment2/node_c.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// For Movit
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>
#include <map>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <tf/transform_listener.h>


void NodeC::manage_request(const assignment2::pickplaceGoalConstPtr &goal) {
    //Create collision objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit::planning_interface::PlanningSceneInterface current_scene;
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    bool success = true;

    std::string obj_id;
    if (goal->pickplace) {
        // Create the poses of the collision objects if picking
        std::vector<geometry_msgs::Pose> poses = goal->object_poses;
        std::vector<int> ids = goal->ids;
        create_collision_object(poses, ids, collision_objects, current_scene, obj_id, goal->id);
        ROS_INFO("Collision objects created");
    } else {
        // Create the collision objects of the table when placing
        geometry_msgs::Pose cylinder_pose;
        cylinder_pose = goal->object_poses[0];
        //ROS_INFO("BEFORE CREATING COLLISION OBJECT Cylinder position is: (%f, %f, %f) (%f, %f, %f, %f)", cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z,
        //         cylinder_pose.orientation.x,cylinder_pose.orientation.y, cylinder_pose.orientation.z, cylinder_pose.orientation.w);
        create_place_table_collision_object(collision_objects, current_scene, cylinder_pose, goal->cylinder_radius);
    }

    ROS_INFO("Received request for id: %d", goal->id);

    // If the robot needs to go in safe position, lift the torso e move the arm in the safe position

    success = success & NodeC::lift_torso(true);

    success = success & NodeC::reach_safe_pos(true);

    if (!success) {
        feedback_.status = ERROR_SAFE_POSE;
        as_.publishFeedback(feedback_);
        result_.success = false;
        as_.setAborted(result_);
        return;
    }
    feedback_.status = SAFE_POSE_REACHED;
    as_.publishFeedback(feedback_);
    ROS_INFO("The arm is in safe place");

    ////////////////////////////           PICK              ///////////////////////////
    if (goal->pickplace) {
        ROS_INFO("Picking routine started");
        geometry_msgs::Pose goal_pose;
        // Find the pose of the object associated to the object I need to pick
        find_goal_object_pose(goal->ids, goal->object_poses, goal->id, goal_pose);

        //ROS_INFO("CHIAMATA PRIMA DI PICK ROUTINE %s",obj_id.c_str());
        // start the pick routine to move the arm and close the gripper
        success = success & pickRoutine(goal_pose, goal->id, obj_id);
        if (!success) {
            feedback_.status = ERROR_PICK;
            as_.publishFeedback(feedback_);
            result_.success = false;
            as_.setAborted(result_);
            return;
        }
        feedback_.status = PICK_COMPLETED;
        as_.publishFeedback(feedback_);
        // Attach the object to tiago's reference frame
        attach(goal->id, current_scene);

        // Close the arm and the torso so that tiago is in a safe position when moving within the environemnt
        success = reach_safe_pos(true);
        success = success & reach_safe_pos(false);
        success = success & lift_torso(false);
        if (!success) {
            //ROS_INFO("NON SONO RIUSCITO");
            feedback_.status = ERROR_CLOSING;
            as_.publishFeedback(feedback_);
            result_.success = false;
            as_.setAborted(result_);
            return;
        }
        feedback_.status = ROBOT_READY;
        as_.publishFeedback(feedback_);
        ROS_INFO("Picking routine completed");
    }

        //////////////////////       PLACE        ////////////////////////
    else {
        ROS_INFO("Placing object");
        geometry_msgs::Pose cylinder_pose;

        // Get the pose of the goal cylinder: only 1 cylinder must be provided in input
        cylinder_pose = goal->object_poses[0];
        //ROS_INFO("BEFORE PLACE ROUTINE Cylinder position is: (%f, %f, %f) (%f, %f, %f, %f)", cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z,
        //         cylinder_pose.orientation.x,cylinder_pose.orientation.y, cylinder_pose.orientation.z, cylinder_pose.orientation.w);
        ROS_INFO("Placing routine");

        success = success & place_routine(cylinder_pose, goal->id);
        if (!success) {
            ROS_ERROR("Fail place");
            feedback_.status = ERROR_PLACE;
            as_.publishFeedback(feedback_);
            result_.success = false;
            as_.setAborted(result_);
            return;
        }
        ROS_INFO("Place Completed");
        feedback_.status = PLACE_COMPLETED;
        as_.publishFeedback(feedback_);
        detach(goal->id);

        success = reach_safe_pos(true);
        success = success & reach_safe_pos(false);
        success = success & lift_torso(false);

        if (!success) {
            feedback_.status = ERROR_CLOSING;
            as_.publishFeedback(feedback_);
            result_.success = false;
            as_.setAborted(result_);
            return;
        }
        feedback_.status = ROBOT_READY;
        as_.publishFeedback(feedback_);
        ROS_INFO("Placing routine done");
    }

    remove_collision_objects(current_scene, collision_objects);

    result_.success = true;
    as_.setSucceeded(result_);
}

bool NodeC::planAndExecuteForwardKinematics(const std::map<std::string, double> &target_position) {

    std::vector<std::string> torso_arm_joint_names;
    // Get the arm group and the planner
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    group_arm_torso.setPlannerId("SBLkConfigDefault");

    // Getting the names of the joints to print the goal position
    torso_arm_joint_names = group_arm_torso.getJoints();

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);

    for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i) {
        if (target_position.count(torso_arm_joint_names[i]) > 0) {
            ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: "
                                 << target_position.at(torso_arm_joint_names[i]));
            group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position.at(torso_arm_joint_names[i]));
        }
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    group_arm_torso.setPlanningTime(5.0);
    // Get the plan, if possible
    bool success = bool(group_arm_torso.plan(my_plan));

    if (!success) {
        ROS_ERROR("No plan was found");
        return false;
    }

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    ros::Time start = ros::Time::now();

    // Execute the movement
    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
    if (!bool(e)) {
        ROS_ERROR("Error executing the movement");
        return false;
    }

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    return true;
}

void NodeC::changeGripperState(bool close) {
    //Create a torso controller action client to move the TIAGo's gripper
    GripperClientPtr grip_client;
    grip_client.reset(new GripperClient("/parallel_gripper_controller/follow_joint_trajectory"));
    grip_client->waitForServer();

    // Generates the goal for the TIAGo's torso
    control_msgs::FollowJointTrajectoryGoal gripper_goal;
    gripper_goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    gripper_goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    //There is only one point to achieve
    gripper_goal.trajectory.points.resize(1);

    //If the gripper needs to close we make the 2 fingers touch (0.0) otherwise set them apart
    if (close) {
        gripper_goal.trajectory.points[0].positions.resize(2);
        gripper_goal.trajectory.points[0].positions[0] = 0.01;
        gripper_goal.trajectory.points[0].positions[1] = 0.01;
        ROS_INFO("Closing gripper");
    } else {
        gripper_goal.trajectory.points[0].positions.resize(2);
        gripper_goal.trajectory.points[0].positions[0] = 0.9;
        gripper_goal.trajectory.points[0].positions[1] = 0.9;
        ROS_INFO("Opening gripper");
    }

    gripper_goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    // Sends the command to start the given trajectory 1s from now
    gripper_goal.trajectory.header.stamp = ros::Time::now();
    grip_client->sendGoal(gripper_goal);
    ROS_INFO("Sending gripper goal");

    // Wait for four seconds instead of trajectory execution, because here the 0.0 position of grippers will never be reached due to box presence
    ros::Duration(4).sleep();
}

bool NodeC::pickRoutine(geometry_msgs::Pose &goal_pose, int id, std::string obj_id) {
    double dist = 0.1; // Dist is the offset used for the so-called "target" position
    bool success = true;
    tf2::Quaternion transform;
    tf2::Quaternion end_orientation;
    tf2::Quaternion obj_orientation;

    // We only care about the yaw of the object, not pitch nor roll
    tf2::convert(goal_pose.orientation, obj_orientation);
    // Consider only the yaw of the object (we care about its rotation along the z axis)
    double r, p, y;
    tf2::Matrix3x3(obj_orientation).getRPY(r, p, y);
    obj_orientation.setRPY(0, 0, y);

    // In all the cases you must take into account that the position you give to the robotic link is not the one of the
    // end effector (the gripper), you must keep into consideration this difference
    switch (id) {
        case 1:
            // Blue object has lateral pick
            transform.setRPY(M_PI_2, 0, 0);
            tf2::convert(transform, goal_pose.orientation);

            goal_pose.position.z -= blue_hexagon_size.at("height") / 3.0;
            goal_pose.position.x -= gripper_length + dist;
            break;
        case 2:
            // Green object has top pick
            transform.setRPY(0, M_PI_2, 0);
            end_orientation = obj_orientation * transform;
            end_orientation = end_orientation.normalize();
            tf2::convert(end_orientation, goal_pose.orientation);

            goal_pose.position.z += gripper_length - green_triangle_size.at("height") * (1 / 2) + dist;
            break;
        case 3:
            // Red object has top pick
            transform.setRPY(0, M_PI_2, -M_PI_2);
            end_orientation = obj_orientation * transform;
            end_orientation = end_orientation.normalize();
            tf2::convert(end_orientation, goal_pose.orientation);

            goal_pose.position.z += gripper_length - red_cube_size.at("side") / 2 + dist;
            break;
    }

    // open the gripper
    changeGripperState(false);

    // Point 5 --- Move the arm in the target position
    success = success & planAndExecuteInverseKinematics(goal_pose, "base_footprint");
    if (!success)
        return false;

    if (id == 1)
        goal_pose.position.x += dist;
    else
        goal_pose.position.z -= dist;
    // Point 6 --- Lower the and and grab the object
    success = success & planAndExecuteInverseKinematics(goal_pose, "base_footprint");
    if (!success)
        return false;

    //close the gripper
    changeGripperState(true);
//	ROS_INFO("CHIAMATA PRIMA DI ATTACH %s",obj_id.c_str());

    return success;
}

bool NodeC::planAndExecuteInverseKinematics(geometry_msgs::Pose &goal_pose, std::string frame) {

    geometry_msgs::PoseStamped targetPose;
    targetPose.header.frame_id = frame;
    targetPose.pose.position = goal_pose.position;
    targetPose.pose.orientation = goal_pose.orientation;
    // Create the move group interface for all the upcoming actions
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPoseReferenceFrame(frame);
    group_arm_torso.setPoseTarget(targetPose);
    group_arm_torso.setPlanningTime(50);

    ROS_INFO_STREAM("Planning to move " <<
                                        group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                                        group_arm_torso.getPlanningFrame());

    ROS_INFO("We plan to move the arm in x=%f y=%f z=%f ox=%f oy=%f oz=%f ow=%f", targetPose.pose.position.x,
             targetPose.pose.position.y,
             targetPose.pose.position.z, targetPose.pose.orientation.x, targetPose.pose.orientation.y, targetPose.pose.orientation.z,
             targetPose.pose.orientation.w);

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);
    group_arm_torso.setNumPlanningAttempts(100);

    // Get plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = bool(group_arm_torso.plan(my_plan));

    if (!success) {
        ROS_ERROR("Error finding plan");
        return false;
    }

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    ros::Time start = ros::Time::now();

    // Execute plan
    moveit::core::MoveItErrorCode e = group_arm_torso.move();
    if (!bool(e))
        ROS_ERROR("Error executing plan");
    else
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    return bool(e);
}


void NodeC::find_goal_object_pose(std::vector<int> ids, std::vector<geometry_msgs::Pose> object_poses, int id,
                                  geometry_msgs::Pose &pose) {
    for (int i = 0; i < ids.size(); i++) {
        // If the id is not the one of the april tag of our object, just skip it and go on with the loop
        if (ids[i] != id)
            continue;

        ROS_INFO("Object with required id found");

        // Get the pose (orientation and pose) of the object of the id we care about
        geometry_msgs::Pose object_pose = object_poses[i];
        tf2::Quaternion object_orientation(object_pose.orientation.x,
                                           object_pose.orientation.y,
                                           object_pose.orientation.z,
                                           object_pose.orientation.w);

        pose.position = object_pose.position;
        tf2::convert(object_orientation, pose.orientation);
    }
}

bool NodeC::lift_torso(bool lift) {
    ROS_INFO("Lifting torso");
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    std::vector<double> current_val = group_arm_torso.getCurrentJointValues();
    std::vector<std::string> joints = group_arm_torso.getJoints();

    // Leave all the joints with their same exact value, we only want to lift the torso
    std::map<std::string, double> target_position;
    for (int i = 0; i < current_val.size(); i++) {
        if (joints[i] != "torso_lift_joint")
            target_position[joints[i]] = current_val[i];
    }

    // If we need to lift than we set an elevated position, otherwise the robot goes down
    if (lift) {
        ROS_INFO("Robot lifting");
        target_position["torso_lift_joint"] = SAFE_POSE[0];
    } else {
        ROS_INFO("Robot lowering");
        target_position["torso_lift_joint"] = CLOSE_POSE[0];
    }

    bool success = planAndExecuteForwardKinematics(target_position);
    if (success)
        ROS_INFO("Lifting torso done");
    else {
        ROS_ERROR("The robot was not able to lift the torso");
        success = false;
    }
    return success;
}

bool NodeC::reach_safe_pos(bool open) {
    bool success = true;
    if (open) {
        // Set the coordinates of the safe position with the open arm
        ROS_INFO("Reaching safe position");
        std::map<std::string, double> target_position;
        target_position["torso_lift_joint"] = SAFE_POSE[0];
        target_position["arm_1_joint"] = SAFE_POSE[1];
        target_position["arm_2_joint"] = SAFE_POSE[2];
        target_position["arm_3_joint"] = SAFE_POSE[3];
        target_position["arm_4_joint"] = SAFE_POSE[4];
        target_position["arm_5_joint"] = SAFE_POSE[5];
        target_position["arm_6_joint"] = SAFE_POSE[6];
        target_position["arm_7_joint"] = SAFE_POSE[7];
        success = planAndExecuteForwardKinematics(target_position);
        if (success)
            ROS_INFO("Safe position reached");
        else {
            ROS_ERROR("Error in reaching the safe position");
            success = false;
        }
    } else {
        // Set the coordinates of the closing position of the arm, without changing the elevation of tiago's torso
        ROS_INFO("Closing robot arm");
        std::map<std::string, double> target_position;
        target_position["torso_lift_joint"] = SAFE_POSE[0];
        target_position["arm_1_joint"] = CLOSE_POSE[1];
        target_position["arm_2_joint"] = CLOSE_POSE[2];
        target_position["arm_3_joint"] = CLOSE_POSE[3];
        target_position["arm_4_joint"] = CLOSE_POSE[4];
        target_position["arm_5_joint"] = CLOSE_POSE[5];
        target_position["arm_6_joint"] = CLOSE_POSE[6];
        target_position["arm_7_joint"] = CLOSE_POSE[7];
        success = planAndExecuteForwardKinematics(target_position);
        if (success)
            ROS_INFO("Robot arm closed successfully");
        else {
            ROS_ERROR("Error in closing the arm");
            success = false;
        }
    }

    return success;
}


bool NodeC::place_routine(geometry_msgs::Pose goal_pose, int id) {
    // Offset (vertical or lateral) place
    double dist = 0.05;
    bool success = true;
    tf2::Quaternion transform;
    tf2::Quaternion end_orientation;
    ROS_INFO("Providing id: %d", id);

    //ROS_INFO("PLACE ROUTINE BEFORE SWITCH Cylinder position is: (%f, %f, %f) (%f, %f, %f, %f)", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,
    //         goal_pose.orientation.x,goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);

    tf::TransformListener listener;
    tf::StampedTransform transform_frames;
    tf::Pose tf_pose;

    listener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("base_footprint", "odom", ros::Time(0), transform_frames);
    tf::poseMsgToTF(goal_pose, tf_pose);
    tf_pose = transform_frames * tf_pose;
    tf::poseTFToMsg(tf_pose, goal_pose);

    goal_pose.orientation.x = goal_pose.orientation.y = goal_pose.orientation.z = goal_pose.orientation.w = 0;

    ROS_INFO("Cylinder position in base_footprint reference frame is: (%f, %f, %f) (%f, %f, %f, %f)", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,
             goal_pose.orientation.x,goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);

    switch (id) {
        case 1:
            // Blue object has lateral pick
            transform.setRPY(M_PI_2, 0, 0);
            tf2::convert(transform, goal_pose.orientation);

            goal_pose.position.z += place_table_enlargement_height/2 + 2.0 * blue_hexagon_size.at("height") / 3.0 + dist;
            goal_pose.position.x -= gripper_length;
            break;
        case 2:
            // Green object has top pick
            transform.setRPY(0, M_PI_2, 0);
            tf2::convert(transform, goal_pose.orientation);

            goal_pose.position.z +=
                    green_triangle_size.at("height") / 2.0 +  gripper_length + dist + place_table_enlargement_height / 2.0;
            break;
        case 3:
            // Red object has top pick
            transform.setRPY(0, M_PI_2, 0);
            tf2::convert(transform, goal_pose.orientation);

            goal_pose.position.z +=
                    red_cube_size.at("side") / 2.0 + gripper_length + dist + place_table_enlargement_height / 2.0;
            break;
    }

    ROS_INFO("The goal pose for the place base_footprint reference frame is: pos = (%f, %f, %f), or = (%f, %f, %f, %f)", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,
             goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);

    success = success & planAndExecuteInverseKinematics(goal_pose, "base_footprint");

    if(!success)
        return false;

    //open the gripper
    changeGripperState(false);

    return success;
}

/**
 * For the part without extra points
 */
void NodeC::find_cylinder_place(std::vector<moveit_msgs::CollisionObject> &collision_objects,
                                moveit::planning_interface::PlanningSceneInterface &current_scene,
                                int id, geometry_msgs::Pose &cylinder_pose) {
    switch (id) {
        // Blue
        case 1:
            // The poses are stored as R G B
            //Blue
            cylinder_pose.position.x = place_cylinder_poses[2][0];
            cylinder_pose.position.y = place_cylinder_poses[2][1];
            cylinder_pose.position.z = place_cylinder_size.at("height");
            break;
        case 2:
            // Green
            cylinder_pose.position.x = place_cylinder_poses[1][0];
            cylinder_pose.position.y = place_cylinder_poses[1][1];
            cylinder_pose.position.z = place_cylinder_size.at("height");
            break;
        case 3:
            // Red
            cylinder_pose.position.x = place_cylinder_poses[0][0];
            cylinder_pose.position.y = place_cylinder_poses[0][1];
            cylinder_pose.position.z = place_cylinder_size.at("height");
            break;
    }

    cylinder_pose.orientation.x = cylinder_pose.orientation.y = cylinder_pose.orientation.z = cylinder_pose.orientation.w = 0;
}

void NodeC::create_place_table_collision_object(std::vector<moveit_msgs::CollisionObject> &collision_objects,
                                                moveit::planning_interface::PlanningSceneInterface &current_scene,
                                                geometry_msgs::Pose cylinder_pose, float radius) {
    moveit_msgs::CollisionObject collision_object;
    shape_msgs::SolidPrimitive primitive;
    geometry_msgs::Pose cyl = cylinder_pose;

    // Create a cylinder with the appropriate radius
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_RADIUS] = place_cylinder_size.at("radius") + object_enlargement_base_cylinder / 2;
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = place_cylinder_size.at("height") + place_table_enlargement_height;
    cyl.position.z = cylinder_pose.position.z / 2.0; //The collision object is generated from the center of the cylinder

    // Add the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cyl);
    collision_object.operation = collision_object.ADD;
    collision_object.header.frame_id = "map";
    collision_object.id = "place_table";


    collision_objects.push_back(collision_object);
    current_scene.applyCollisionObjects(collision_objects);

    //ROS_INFO("CREEATE PLACE TABLE FUNCTION FINAL cylinder position is: (%f, %f, %f) (%f, %f, %f, %f)", cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z,
    //         cylinder_pose.orientation.x,cylinder_pose.orientation.y, cylinder_pose.orientation.z, cylinder_pose.orientation.w);

}

void NodeC::remove_collision_objects(moveit::planning_interface::PlanningSceneInterface &current_scene,
                                     std::vector<moveit_msgs::CollisionObject> &collision_objects) {
    // remove all the collision objects before moving
    std::vector<std::string> objects_ids;
    for (int i = 0; i < collision_objects.size(); i++) {
        objects_ids.push_back(collision_objects[i].id);
    }

    current_scene.removeCollisionObjects(objects_ids);
}

void NodeC::create_collision_object(std::vector<geometry_msgs::Pose> &poses, std::vector<int> &ids,
                                    std::vector<moveit_msgs::CollisionObject> &vec,
                                    moveit::planning_interface::PlanningSceneInterface &current_scene,
                                    std::string &obj_id, int id) {
    ROS_INFO("NUMBER OF ELEMENTS %d", ids.size());

    for (int i = 0; i < ids.size(); i++) {
        ROS_INFO("Object with id %d is getting added to the collision objects", ids[i]);

        // Accessing the current element
        int32_t currentId = ids[i];
        geometry_msgs::Pose tmp = poses[i];
        moveit_msgs::CollisionObject collision_obj;
        shape_msgs::SolidPrimitive primitive;

        // We create here the collision objects: the cylinder don't have any problem with the orientation since it's
        // always the same regardless of the orientation
        // while the green traingle is rotated, for this reason, it is necessary to compensate for its rotation.
        // also the cube may be rotated, so we need to handle also this case (for generality purposes)

        if (currentId == 1) {
            // Blue
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[primitive.CYLINDER_HEIGHT] =
                    NodeC::blue_hexagon_size.at("height") + 0.03;
            primitive.dimensions[primitive.CYLINDER_RADIUS] =
                    NodeC::blue_hexagon_size.at("radius") + 0.009;

            obj_id = "Hexagon";
            // The position is with respect to the center of the object
            tmp.position.z -= NodeC::blue_hexagon_size.at("height") / 2.0;
        } else if (currentId == 2) {
            // Green
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = NodeC::green_triangle_size.at("x")+0.005;
            primitive.dimensions[primitive.BOX_Y] = NodeC::green_triangle_size.at("y")+0.005;
            primitive.dimensions[primitive.BOX_Z] = NodeC::green_triangle_size.at("height") + object_enlargement_height;

            // Since the triangle is oriented differently, we need to rotate the collision object appropriately
            tf2::Quaternion triangle_orientation;
            tf2::convert(tmp.orientation, triangle_orientation);
            double r, p, y;
            tf2::Matrix3x3(triangle_orientation).getRPY(r, p, y);
            triangle_orientation.setRPY(0, 0, y);
            tmp.orientation = tf2::toMsg(triangle_orientation);

            obj_id = "Triangle";
            // The position is with respect to the center of the object
            tmp.position.z -= (NodeC::green_triangle_size.at("height") + object_enlargement_height) / 2.0;
        } else if (currentId == 3) {
            // Red
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = NodeC::red_cube_size.at("side") + 0.01;
            primitive.dimensions[primitive.BOX_Y] = NodeC::red_cube_size.at("side") + 0.01;
            primitive.dimensions[primitive.BOX_Z] = NodeC::red_cube_size.at("side") + 0.02;

            tf2::Quaternion cube_orientation;
            tf2::convert(tmp.orientation, cube_orientation);
            double r, p, y;
            tf2::Matrix3x3(cube_orientation).getRPY(r, p, y);
            cube_orientation.setRPY(0, 0, y);
            tmp.orientation = tf2::toMsg(cube_orientation);

            obj_id = "cube";
            // The position is with respect to the center of the object
            tmp.position.z -= NodeC::red_cube_size.at("side") / 2.0;
        } else {
            // Gold obstacles
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[primitive.CYLINDER_HEIGHT] =
                    NodeC::golden_obs_size.at("height") + NodeC::object_enlargement_height;
            primitive.dimensions[primitive.CYLINDER_RADIUS] =
                    NodeC::golden_obs_size.at("radius") + NodeC::object_enlargement_base / 2.0;

            obj_id = "obstacle" + std::to_string(currentId);
            // The position is with respect to the center of the object
            tmp.position.z -= NodeC::golden_obs_size.at("height") / 2.0;
        }

        collision_obj.primitives.push_back(primitive);
        collision_obj.primitive_poses.push_back(tmp);
        collision_obj.operation = collision_obj.ADD;
        collision_obj.header.frame_id = "base_footprint";
        collision_obj.id = obj_id;
        vec.push_back(collision_obj);
        //for(int i=0; i < vec.size(); i++){
        ROS_INFO("Object with %s id was added", vec[i].id.c_str());
        //}
        current_scene.applyCollisionObjects(vec);
    }

    moveit_msgs::CollisionObject collision_object;
    shape_msgs::SolidPrimitive primitive;
    geometry_msgs::Pose object_pose;
    tf2::Quaternion orientation;

    // Generating the collision object associated with the table

    collision_object.id = "table";
    collision_object.header.frame_id = "map";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = table_dim[0] + 0.08;
    primitive.dimensions[primitive.BOX_Y] = table_dim[1] + 0.08;
    primitive.dimensions[primitive.BOX_Z] = table_pose[2] - 0.02;

    object_pose.position.x = table_pose[0];
    object_pose.position.y = table_pose[1];
    object_pose.position.z = (table_dim[2] + table_pose[2]) / 2;
    orientation.setRPY(table_pose[3], table_pose[4], table_pose[5]);
    tf2::convert(orientation, object_pose.orientation);

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    vec.push_back(collision_object);

    current_scene.applyCollisionObject(collision_object);

    switch (id) {
        case 1:
            obj_id = "Hexagon";
            break;
        case 2:
            obj_id = "Triangle";
            break;
        case 3:
            obj_id = "cube";
            break;
    }

}

void NodeC::attach(const int id, moveit::planning_interface::PlanningSceneInterface &current_scene) {
    std::string obj_id;
    switch (id) {
        case 1:
            obj_id = "Hexagon";
            break;
        case 2:
            obj_id = "Triangle";
            break;
        case 3:
            obj_id = "cube";
            break;
    }
    //std::string str = obj_id.c_str();
    moveit_msgs::CollisionObject attach_object;
    attach_object.id = obj_id;


    // We need first to remove the collision object from the planning space
    ROS_INFO("Ready to remove the object %s from the enviroment", obj_id.c_str());
    attach_object.operation = attach_object.REMOVE;
    current_scene.applyCollisionObject(attach_object);
    ROS_INFO("The object was removed from the enviroment");

    //call node for attacher
    ros::NodeHandle nh_;
    ros::ServiceClient attachClient_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>(
            "/link_attacher_node/attach");
    gazebo_ros_link_attacher::Attach att;
    att.request.model_name_1 = "tiago";
    att.request.link_name_1 = "arm_7_link";


    att.request.model_name_2 = obj_id;
    att.request.link_name_2 = obj_id + "_link";

    //call the client
    if (attachClient_.call(att)) {
        if (att.response.ok) {
            ROS_INFO("Object attached ");
        } else {

            ROS_INFO("Object not attached");
        }
    } else {
        ROS_ERROR("Failed to call attach service server");
    }
}

void NodeC::detach(int id) {
    std::string obj_id;
    switch (id) {
        case 1:
            obj_id = "Hexagon";
            break;
        case 2:
            obj_id = "Triangle";
            break;
        case 3:
            obj_id = "cube";
            break;
    }
    // Initialize ROS node
    ros::NodeHandle nh;
    ROS_INFO("ID DETACH : %s", obj_id.c_str());
    // Create service client
    ros::ServiceClient detachClient = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    gazebo_ros_link_attacher::Attach det;

    // Set model and link names
    det.request.model_name_1 = "tiago";
    det.request.link_name_1 = "arm_7_link";

    det.request.model_name_2 = obj_id;
    det.request.link_name_2 = obj_id + "_link";

    // Call the client
    if (detachClient.call(det)) {
        if (det.response.ok) {
            ROS_INFO("Detach successful");
        } else {
            ROS_INFO("Detach not successful");
//            ROS_ERROR("Detach failed with error: %s", det.response.error_message.c_str());
        }
    } else {
        ROS_ERROR("Failed to call detach service");
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle nh;

    NodeC c("pick_place");

    ros::spin();
    return 0;
}