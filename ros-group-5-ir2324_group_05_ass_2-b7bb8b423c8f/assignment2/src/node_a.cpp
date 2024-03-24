#include <assignment2/node_a.h>


void NodeA::feedbackCb_pickplace(const assignment2::pickplaceFeedbackConstPtr & feedback){
    switch (feedback->status) {
        case NodeC::SAFE_POSE_REACHED:
            printf("The robot has reached the safe position\n");
            break;
        case NodeC::PICK_COMPLETED:
            printf("The robot successfully picked the object from the table\n");
            break;
        case NodeC::PLACE_COMPLETED:
            printf("The robot successfully placed the object on the cylinder\n");
            break;
        case NodeC::ROBOT_READY:
            printf("The robot has finished the requested routine and is free to move within the environment\n");
            break;
        case  NodeC::ERROR_SAFE_POSE:
            printf("The robot was not able to achieve the safe position\n");
            break;
        case NodeC::ERROR_PICK:
            printf("There was an error during the pick procedure\n");
            break;
        case NodeC::ERROR_PLACE:
            printf("There was an error while placing the object\n");
            break;
        case NodeC::ERROR_CLOSING:
            printf("The robot was not able to close itself for going back to a safe navigation in the environment\n");
            break;
        default: ROS_INFO ("Something went wrong, incorrect parameter was used\n");

    }

}

bool NodeA::move_to_point(const std::vector<double>& point) {
    actionlib::SimpleActionClient <assignment2::robotAction> ac("move_around", true);
    printf("Waiting for action server to start.\n");
    ac.waitForServer(); //will wait for infinite time until the server starts

    assignment2::robotGoal goal;

    goal.x = point[0];
    goal.y = point[1];
    goal.th = point[2];
    ROS_INFO("Going at position %f, %f, %f", goal.x, goal.y, goal.th);

    actionlib::SimpleClientGoalState state = ac.sendGoalAndWait(goal);
    if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Navigation succeeded");
    else {
        ROS_ERROR("There was an error achieving the desired pose");
        ROS_INFO("The pose was X= %f Y=%f", goal.x, goal.y);
        exit(1);
    }
    return true;
}

void NodeA::detection(std::vector <geometry_msgs::Pose> &detection_poses, std::vector<int> &ids) {
    ROS_INFO("Starting detection of the objects");

    ros::ServiceClient client = n_.serviceClient<assignment2::detection>("get_pose");
    assignment2::detection detected;

    //we set ready to start the detection
    detected.request.ready = true;

    //print
    if (client.call(detected)) {
        for (int i = 0; i < detected.response.pose_objects.size(); i++) {
            ROS_INFO("The object with id [%d] has: X = [%f], Y = [%f] , Z = [%f]", detected.response.ids[i],
                     detected.response.pose_objects[i].position.x,
                     detected.response.pose_objects[i].position.y, detected.response.pose_objects[i].position.z);
        }

        detection_poses = detected.response.pose_objects;
        ids = detected.response.ids;
    } else {
        exit(1);
    }
}

void NodeA::pick_object(std::vector <geometry_msgs::Pose> &obj_poses, std::vector<int> &ids, int id) {
    actionlib::SimpleActionClient <assignment2::pickplaceAction> client("pick_place", true);
    ROS_INFO("Created Node C client - Pick");
    client.waitForServer();

    assignment2::pickplaceGoal goal;
    // pick for true
    goal.pickplace = true;
    goal.object_poses = obj_poses;
    goal.ids = ids;
    goal.id = id;

    ROS_INFO("Sending request to action server");
    client.sendGoal(goal, actionlib::SimpleActionClient <assignment2::pickplaceAction>::SimpleDoneCallback(),
                                                             actionlib::SimpleActionClient <assignment2::pickplaceAction>::SimpleActiveCallback(),
                                                             NodeA::feedbackCb_pickplace);

    bool res = client.waitForResult();
    if (res) {
        assignment2::pickplaceResultConstPtr result = client.getResult();
        if (result->success)
            ROS_INFO("Pick action done successfully!");
        else {
            ROS_ERROR("Pick action failed");
            exit(1);
        }
    }
    else {
        ROS_ERROR("Pick action failed");
        exit(1);
    }
}

void NodeA::place_object(int id, float cylinder_radius, geometry_msgs::Pose& cylinder_pos) {
    actionlib::SimpleActionClient <assignment2::pickplaceAction> client("pick_place", true);
    ROS_INFO("Created Node C client - Place");
    client.waitForServer();

    assignment2::pickplaceGoal goal;

    //false to place
    goal.pickplace = false;
    goal.id = id;
    std::vector<geometry_msgs::Pose> cylinder_poses{cylinder_pos};
    ROS_INFO("Cylinder position is: (%f, %f, %f) (%f, %f, %f, %f)", cylinder_poses[0].position.x, cylinder_poses[0].position.y, cylinder_poses[0].position.z,
             cylinder_poses[0].orientation.x,cylinder_poses[0].orientation.y, cylinder_poses[0].orientation.z, cylinder_poses[0].orientation.w);
    goal.object_poses = cylinder_poses;
    goal.cylinder_radius = cylinder_radius;

    ROS_INFO("Sending request to action server");
    client.sendGoal(goal, actionlib::SimpleActionClient <assignment2::pickplaceAction>::SimpleDoneCallback(),
                    actionlib::SimpleActionClient <assignment2::pickplaceAction>::SimpleActiveCallback(),
                    NodeA::feedbackCb_pickplace);

    bool res = client.waitForResult();
    if (res) {
        assignment2::pickplaceResultConstPtr result = client.getResult();
        if(result->success)
            ROS_INFO("Place action done successfully!");
        else{
            ROS_ERROR("Place action failed");
            exit(1);
        }
    }
    else {
        ROS_ERROR("Place action failed");
        exit(1);
    }
}

void NodeA::fill_cylinder() {
    // formerly used for switching between bonus / non-bonus.
    // If you set it to false it won't work since that part wasn't part of development anymore. However,
    // we left it for completeness
    bool do_detection = true;

    if(do_detection){
        assignment2::laser_detect srv;
        ros::ServiceClient client = n_.serviceClient<assignment2::laser_detect>("detect_objects");

        srv.request.min_radius = MIN_RADIUS;
        srv.request.max_radius = MAX_RADIUS;

        if(client.call(srv)) {
            place_cylinder_poses = srv.response.cylinder_poses;
            //radii = srv.response.radii;
            ROS_INFO("Detection succeeded");
            for(int i = 0; i < place_cylinder_poses.size(); i++){
                ROS_INFO("Pose of cylinder %d: x=%f, y=%f, z=%f", i, place_cylinder_poses[i].x, place_cylinder_poses[i].y,
                         place_cylinder_poses[i].z);
            }
        }
        else{
            ROS_ERROR("Detection failed");
            exit(1);
        }

        if(place_cylinder_poses.size() > 3)
        {
            // We know that the circle detector may detect more circles than it should. We look for the cylinders that minimize the variance along y
            ROS_INFO("Too many circle detections, need to remove some. Removing the ones further away from the average y");
            /* Using only the mean was not enough
            float avg_y = 0.0;
            for(int i = 0; i < place_cylinder_poses.size(); i++){
                avg_y += place_cylinder_poses[i].y;
            }
            avg_y = avg_y / place_cylinder_poses.size();

            for(int i = 0; i < place_cylinder_poses.size(); i++){
                float offset_from_avg = abs(place_cylinder_poses[i].y - avg_y);
                if(offset_from_avg > 0.75){
                    place_cylinder_poses.erase(place_cylinder_poses.begin() + i);
                    ROS_INFO("Removed average at position %d", i);
                }
            }
            */

            float min_var = 10000;
            std::vector<int> correct_pillars_index;

            // look at groups of 3 points
            for(int i = 0; i < place_cylinder_poses.size(); i++) {
                for (int j = 0; j < place_cylinder_poses.size(); j++) {
                    if (i != j) {
                        for (int k = 0; k < place_cylinder_poses.size(); k++) {
                            if (i != k && j != k) {
                                std::vector<int> array_positions = {i, j, k};
                                float avg_y = 0.0;
                                for (int p = 0; p < array_positions.size(); p++)
                                    avg_y += place_cylinder_poses[array_positions[p]].y;
                                avg_y /= 3;

                                float var_y = 0.0;
                                for (int p = 0; p < array_positions.size(); p++)
                                    var_y += std::pow(place_cylinder_poses[array_positions[p]].y - avg_y, 2);

                                if (var_y < min_var) {
                                    ROS_INFO("The cylinders at index %d, %d, %d have better variance than the current ones. Variance = %f", i, j, k, var_y);
                                    min_var = var_y;
                                    correct_pillars_index = array_positions;
                                }
                            }
                        }
                    }
                }
            }

            ROS_INFO("The minimum variance is [%f], achieved by the cylinders at position %d, %d, %d", min_var,
                     correct_pillars_index[0], correct_pillars_index[1], correct_pillars_index[2]);

            place_cylinder_poses = {place_cylinder_poses[correct_pillars_index[0]],
                                    place_cylinder_poses[correct_pillars_index[1]],
                                    place_cylinder_poses[correct_pillars_index[2]]};

            ROS_INFO("Removal done");
        }

        // Sort the element in such a way the leftmost pillar from the robot point of view is in position 0 of the vector
        if(place_cylinder_poses[0].x > place_cylinder_poses[1].x) {
            std::swap(place_cylinder_poses[0], place_cylinder_poses[1]);
        }
        if(place_cylinder_poses[0].x > place_cylinder_poses[2].x) {
            std::swap(place_cylinder_poses[0], place_cylinder_poses[2]);
        }
        if(place_cylinder_poses[1].x > place_cylinder_poses[2].x) {
            std::swap(place_cylinder_poses[1], place_cylinder_poses[2]);
        }
    }

    /* This was for the part without extra points, not supported
    else {
        place_cylinder_poses[0].x = 6.57991 + 4.007396;
        place_cylinder_poses[0].y = place_cylinder_poses[1].y = place_cylinder_poses[2].y = -1.369981 + 1.015966;
        place_cylinder_poses[0].z = place_cylinder_poses[1].z = place_cylinder_poses[2].z = 0.69;
        place_cylinder_poses[1].x = 6.57991 + 5.007404;
        place_cylinder_poses[2].x = 6.57991 + 6.007146;

        radii = {0.21, 0.21, 0.21};
    }
     */
}

void NodeA::compute_cluster_center(geometry_msgs::Point point_cylinder, geometry_msgs::Point& center_cylinder){
    ROS_INFO("Computing the center of the cylinder...");

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Wait for a transform from base_link (robot reference frame) to odom (global reference frame)
    listener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("base_footprint", "odom", ros::Time(0), transform);
    // Get the coordinates of thfeedbacke position (0,0) of the robot in the global reference frame
    geometry_msgs::Point pos_rob;
    pos_rob.x = 0;
    pos_rob.y = 0;
    pos_rob.z = 0;

    tf::Point point_cylinder_tf;
    tf::pointMsgToTF(point_cylinder, point_cylinder_tf);
    point_cylinder_tf = transform * point_cylinder_tf;
    tf::pointTFToMsg(point_cylinder_tf, point_cylinder);

    //ROS_INFO("Pose cylinder in robot reference frame %f, %f, %f", point_cylinder.x, point_cylinder.y, point_cylinder.z);

    // Computing line coefficients
    double m = (pos_rob.y - point_cylinder.y) / (pos_rob.x - point_cylinder.x); // slope of the line

    double alpha = std::atan(m);
    double dx = CYLINDER_RADIUS * cos(alpha);
    double dy = CYLINDER_RADIUS * sin(alpha);

    double x_center = point_cylinder.x + dx;
    double y_center = point_cylinder.y + dy;

    center_cylinder.x = x_center;
    center_cylinder.y = y_center;
    center_cylinder.z = 0;

    //ROS_INFO("ROBOT REFERENCE FRAME CENTER: (%f, %f)", center_cylinder.x, center_cylinder.y);

    // Wait for a transform from base_link (robot reference frame) to odom (global reference frame)
    listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("odom", "base_link", ros::Time(0), transform);

    tf::Point center_cylinder_tf;
    tf::pointMsgToTF(center_cylinder, center_cylinder_tf);
    center_cylinder_tf = transform * center_cylinder_tf;
    tf::pointTFToMsg(center_cylinder_tf, center_cylinder);

    center_cylinder.z = CYLINDER_HEIGHT;

    ROS_INFO("Cylinder center in odom reference frame: (%f, %f, %f)", center_cylinder.x, center_cylinder.y, center_cylinder.z);
}


void NodeA::get_place_pose(int id, std::vector<double> &waypoint_place, float& cylinder_radius,
                    geometry_msgs::Pose& cylinder_pos) {

    // Filling the details of the cylinder in the map
    fill_cylinder( );

    ros::ServiceClient client = n_.serviceClient<assignment2::pose_place>("pose_place");
    assignment2::pose_place position;

    // sending the coordinates of the pillars => we order the coordinates of the pillars in fill_cylinder
    for (int i = 0; i < place_cylinder_poses.size(); i++) {
        position.request.pillars_poses.push_back(place_cylinder_poses[i]);
    }

    //sending id
    position.request.id = id;

    if (client.call(position)) {
        ROS_INFO("The position associated to the cylinder where the object should be placed is X = %f , Y = %f",
                 position.response.pose_place.x, position.response.pose_place.y);
        float new_y;

        geometry_msgs::Point point_cylinder = position.response.pose_place;
        geometry_msgs::Point center_cylinder;
        compute_cluster_center(point_cylinder, center_cylinder);

        // Send the robot in a position such that it is far enough for make the movement with the arm and place the object
        if (id != 1)
            new_y = center_cylinder.y - CYLINDER_RADIUS - 0.40;
        else
            new_y = center_cylinder.y - CYLINDER_RADIUS - 0.60;
        waypoint_place.push_back(center_cylinder.x);
        waypoint_place.push_back(new_y);
        waypoint_place.push_back(1.570796); //90 degrees

        cylinder_pos.position.x = center_cylinder.x;
        cylinder_pos.position.y = center_cylinder.y;
        cylinder_pos.position.z = center_cylinder.z;
        cylinder_pos.orientation.x = cylinder_pos.orientation.y = cylinder_pos.orientation.z = cylinder_pos.orientation.w = 0;

        ROS_INFO("Cylinder position is: (%f, %f, %f) (%f, %f, %f, %f)", cylinder_pos.position.x, cylinder_pos.position.y, cylinder_pos.position.z,
                 cylinder_pos.orientation.x,cylinder_pos.orientation.y, cylinder_pos.orientation.z, cylinder_pos.orientation.w);
    } else {
        ROS_INFO("The call has not succeded");
        exit(1);
    }
}

void NodeA::do_routine(std::vector<int> id_sequence){

    //after finishing the test phase
    for (int i = 0; i < id_sequence.size(); i++) {
        int cur_id = id_sequence[i];

        ros::NodeHandle n;
        ROS_INFO("Starting routine for object with id %d", cur_id);

        // Move to the waypoint avoid obstacle
        if (i == 0) {
            move_to_point(waypoint_1);

            // Going around the table if you need to pick the green object
            if (cur_id == 2)
                move_to_point(waypoint_3);
        }

        // Move the robot in the correct position around the table
        move_to_point(objects_positions[cur_id - 1]);

        std::vector<geometry_msgs::Pose> object_poses;
        std::vector<int> ids;
        // Do the detection of the april tags
        detection(object_poses, ids);

        // Pick up the object from the table
        pick_object(object_poses, ids, cur_id);

        // If you're not picking green, go away from the table in order not to collide with it
        if (cur_id != 2)
            move_to_point(waypoint_2);

        // Move to the waypoint where the robot will take the image to understand where the cylinder of the right color is
        move_to_point(waypoint_4);

        std::vector<double> waypoint_place;
        geometry_msgs::Pose cylinder_pos;
        float cylinder_radius = 0;

        // Get the position of the cylinder of the right color and the pose to reach for placing the object
        get_place_pose(cur_id, waypoint_place, cylinder_radius, cylinder_pos);

        // Move to the point in front of the right cylinder
        move_to_point(waypoint_place);

        // Place the object on the cylinder
        place_object(cur_id, cylinder_radius, cylinder_pos);

        // After placing the last object, stay still
        if(i == id_sequence.size() - 1)
            break;

        move_to_point(neg_waypoint_4);

        move_to_point(neg_waypoint_3);

        // If the next id is not the green one, then go to waypoint 2
        if(i < id_sequence.size() - 1 && id_sequence[i+1] != 2)
            move_to_point(neg_waypoint_2);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "action_client");
    ros::NodeHandle n;
    NodeA a;

    ros::ServiceClient client = n.serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv");
    tiago_iaslab_simulation::Objs srv;
    std::vector<int> id_sequence;

    srv.request.ready = true;
    srv.request.all_objs = true;

    // Wait for the server of the service to reply
    if (client.call(srv)) {
        ROS_INFO("The human node sent %d ids", srv.response.ids.size());
        // Insert in the vector of ids the received id
        id_sequence = srv.response.ids;
    } else {
        ROS_ERROR("Failed to call service human_objects_srv");
        return 1;
    }

    a.do_routine(id_sequence);

    return 0;
}