#include <assignment2/scan_clusterizer.h>
#include <algorithm>
#include <ros/ros.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <assignment2/utilities.h>
#include <tf/transform_listener.h>
float DISTANCE = 0.5; // The maximum distance of 2 points in order to be considered in the same cluster

bool ScannerClusterizer::are_same_cluster(const cv::Point2f &a, const cv::Point2f &b) {
    float l2 = Utilities::distance(a, b);
    return l2 < DISTANCE;
}

void ScannerClusterizer::cluster_centroid(std::unordered_map<int, std::vector<cv::Point2f>> &unremoved_cluster,
                                          std::unordered_map<int, cv::Point2f> &centroids) {
    //calculate the centroid
    for (const auto &entry: unremoved_cluster) {
        //Don't consider removed clusters
        if (entry.second.empty())
            continue;

        //Compute centroid only for valid clusters
        std::vector <cv::Point2f> points = entry.second;
        double x, y;
        ScannerClusterizer::cluster_avg(points, x, y);

        // Add the centroid of the cluster to the set of centroids
        centroids[entry.first] = cv::Point2f(x, y);
    }
}

void ScannerClusterizer::cluster_avg(std::vector <cv::Point2f> cluster, double &x, double &y) {
    x = y = .0;

    for (int i = 0; i < cluster.size(); i++) {
        x += cluster[i].x;
        y += cluster[i].y;
    }

    //average
    x /= cluster.size();
    y /= cluster.size();
}

int ScannerClusterizer::remove_close_centroids(std::unordered_map<int, std::vector<cv::Point2f>> &unremoved_cluster) {

    //Contains the position of the people: it is obtained by averaging the points that were included in the same cluster
    //(they are the points of both feet of a person)
    std::unordered_map<int, cv::Point2f> centroids;
    ScannerClusterizer::cluster_centroid(unremoved_cluster, centroids);

    int removed_clusters = 0;
    for (auto const &entry: unremoved_cluster) {
        // Ignore the empty clusters => they were previously removed
        if (entry.second.empty())
            continue;

        int c_num = entry.first;
        //remove condition
        double x = centroids[c_num].x;
        double y = centroids[c_num].y;
        double dist = std::sqrt(x * x + y * y);

        // We remove the clusters that have a centroid that is really close to the lidar origin (0,0): these are
        // some measurements that are inside the chassis of the robot
        if (dist <= 0.04) {
            unremoved_cluster[c_num].clear();
            removed_clusters++;
        }
    }

    return removed_clusters;
}

void
ScannerClusterizer::read_scanner(std::vector<float> ranges, std::vector <cv::Point2f> &coordinates, float min_angle,
                                 float thetaStep) {
    // Vector needed by cv::partition() for getting the cluster labels
    std::vector<int> labels;

    //put only the values !=inf, corresponding to actual readings of the scanner
    for (int i = 0; i < ranges.size(); i++) {
        //The points must be at some minimum distance from the robot
        if (!isinf(ranges[i])) {
            // Converting the points from polar coordinates to cartesian coordinates
            coordinates.push_back(cv::Point2f(ranges[i] * cos(min_angle + i * thetaStep),
                                              ranges[i] * sin(min_angle + i * thetaStep)));
        }
    }
}

int ScannerClusterizer::compute_clusters(std::vector <cv::Point2f> coordinates, std::vector <cv::Point2f> &res,
                                         float distance, std::vector<float>& radii, float min_radius, float max_radius) {

    std::vector<int> labels; //Vector containing the cluster label of each point
    DISTANCE = distance; // Set the maximum distance value for which two elements are considered to be in the same cluster

    // Getting the clusters according to the decision rule defined in are_same_cluster
    int n_clusters = cv::partition(coordinates, labels, ScannerClusterizer::are_same_cluster);

    // take the created the clusters and create a map to store the points in each cluster
    std::unordered_map<int, std::vector<cv::Point2f>> cluster_dict;
    for (int i = 0; i < labels.size(); i++) {
        int l = labels[i];
        cv::Point2f pt = coordinates[i];

        // Inserting the point with label l in the corresponding cluster
        cluster_dict[l].push_back(pt);
    }

    ScannerClusterizer::remove_small_clusters(cluster_dict);
    ScannerClusterizer::remove_close_centroids(cluster_dict);

    Utilities::keep_circles(cluster_dict, min_radius, max_radius, res, radii);

    //re-initialize
    n_clusters=0;

    for (auto const &entry: cluster_dict)
        if(!entry.second.empty())
            n_clusters++;

    std::unordered_map<int, cv::Point2f> centroids;
    compute_cluster_average(cluster_dict, centroids);

    std::unordered_map<int, cv::Point2f> closest_point;
    find_closest_point(cluster_dict, centroids, closest_point);

    std::vector<cv::Point2f> result;
    for(auto const &entry: closest_point) {
        result.push_back(entry.second);
    }

    res = result;

    ROS_INFO("number of remaining cluster %d", n_clusters);

    return res.size();
}

void ScannerClusterizer::remove_small_clusters(std::unordered_map<int, std::vector<cv::Point2f>> &cluster_dict) {
    for (const auto &entry: cluster_dict) {
        if (entry.second.size() <= ScannerClusterizer::MIN_CLUSTER_SIZE) {
            cluster_dict[entry.first].clear();
        }
    }
}

void ScannerClusterizer::compute_cluster_average(std::unordered_map<int, std::vector<cv::Point2f>>& cluster_dict, std::unordered_map<int, cv::Point2f>& result){

    for (auto const &entry: cluster_dict) {
        if (!entry.second.empty()) {
            cv::Point2f sum = {0.0, 0.0};
            for (int i = 0; i < entry.second.size(); i++)
                sum += entry.second[i] ;
            cv::Point2f average = {sum.x / entry.second.size(), sum.y / entry.second.size()};
            result[entry.first] = average;
        }
    }
}

void ScannerClusterizer::find_closest_point(std::unordered_map<int, std::vector<cv::Point2f>>& cluster_dict,
                                                    std::unordered_map<int, cv::Point2f>& centroids,
                                                    std::unordered_map<int, cv::Point2f>& closest_point){
    for (const auto &entry: centroids){
        int cluster_id = entry.first;
        std::vector<cv::Point2f> cluster = cluster_dict[cluster_id];

        float min_dist = 10000;
        cv::Point2f min_dist_point;
        for(int j = 0; j < cluster.size(); j++){
            float dist = Utilities::distance(cluster[j], entry.second);
            if(dist < min_dist)
            {
                min_dist = dist;
                min_dist_point = cluster[j];
            }
        }

        closest_point[entry.first] = min_dist_point;
    }
}