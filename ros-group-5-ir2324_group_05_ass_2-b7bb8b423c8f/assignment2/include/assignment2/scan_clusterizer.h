#ifndef SCAN_CLUSTERIZER_H
#define SCAN_CLUSTERIZER_H

#include <unordered_map>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core/operations.hpp>
#include <cv_bridge/cv_bridge.h>


namespace ScannerClusterizer{
    // SETTING VARIABLES FOR THIS ENVIRONMENT
    const int MIN_CLUSTER_SIZE = 5; // The minimum size of a cluster to be considered valid

    /// Function used by partition to determine if the 2 points in input should belong to the same cluster
    /// \param a first point
    /// \param b second point
    /// \return True if the 2 points shoudl belong to the same cluster, false otherwise
    bool are_same_cluster(const cv::Point2f &a, const cv::Point2f &b);

    /// Given a cluster of points, returns the average of the points belonging to that cluster
    /// \param cluster set of points included in a cluster
    /// \param x
    /// \param y
    void cluster_avg(std::vector<cv::Point2f> cluster, double &x, double &y);

    /// Reads the ranges provided by the scanner and returns the corresponding cartesian coordinates
    /// \param ranges input sequence of readings
    /// \param coordinates cartesian coordinates corresponding to each "valid" reading
    /// \param min_angle starting angle of the laser's scan
    /// \param thetaStep increment step of the angle
    void read_scanner(std::vector<float> ranges, std::vector<cv::Point2f> &coordinates, float min_angle, float thetaStep);

    /// Computes the clusters given the coordinates of the point in the 2D space.
    /// Returns the coordinates of centers of the clusters as well as the number of clusters
    /// \param coordinates points to clusterize
    /// \param res reference vector, it will contain the cluster centers at the end of the algorithm
    /// \return the number of clusters found
    int compute_clusters(std::vector <cv::Point2f> coordinates, std::vector <cv::Point2f> &res,
                         float distance, std::vector<float>& radii, float min_radius, float max_radius);

    /// Removes the clusters that have a centroid too close to the centroid of the robot. This is needed since
    /// the lidar is inside the robot and part of the robot's chassis is detected by the laser.
    /// \param cluster_dict set of cluster points
    /// \param centroids the centroid corresponding to each cluster
    /// \return the number of removed centroids
    int remove_close_centroids(std::unordered_map<int,std::vector<cv::Point2f>> &cluster_dict);

    /// Given a set of cluster points, for each of them it computes the centroid
    /// \param cluster_dict set of cluster points
    /// \param centroids out parameter, filled by this function
    void cluster_centroid(std::unordered_map<int,std::vector<cv::Point2f>> &cluster_dict, std::unordered_map<int, cv::Point2f> &centroids);

    /// Converts the cluster points set into a centroid array, removing the label of the cluster
    /// \param centroids pair (cluster label, centroid)
    /// \param cluster_dict centroids map
    //void extract_centroids(std::unordered_map<int, std::vector<cv::Point2f>> &cluster_dict, std::vector<cv::Point2f>& centroids_vec);

    /// Removes all the small clusters that have less than MIN_CLUSTER_SIZE points
    /// \param cluster_dict set of cluster points
    void remove_small_clusters(std::unordered_map<int, std::vector<cv::Point2f>>& cluster_dict);

    void compute_cluster_average(std::unordered_map<int, std::vector<cv::Point2f>>& cluster_dict, std::unordered_map<int, cv::Point2f>& result);

    void find_closest_point(std::unordered_map<int, std::vector<cv::Point2f>>& cluster_dict, std::unordered_map<int, cv::Point2f>& centroids,
                            std::unordered_map<int, cv::Point2f>& closest_point);
};

#endif //SCAN_CLUSTERIZER_H
