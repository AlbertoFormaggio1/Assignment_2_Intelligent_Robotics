
#ifndef INTELLIGENT_ROBOTICS_ASSIGNMENT1_UTILITIES_H
#define INTELLIGENT_ROBOTICS_ASSIGNMENT1_UTILITIES_H
#include <unordered_map>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core/operations.hpp>
#include <cv_bridge/cv_bridge.h>

namespace Utilities{
    //initialize parameters
    const float IMAGE_RESOLUTION = 0.05; // The resolution to generate the image of the environment;
    //const float MAX_RADIUS = 0.5; // Maximum radius for an obstacle to be considered as a circle

    /// Understands if the given points form a circle
    /// \param points set of points
    /// \param center center of the circle
    /// \param r radius of the circle
    /// \return true if the points describe a circle
    bool is_circle(std::vector<cv::Point2f> points, cv::Point2f center, float r, float max_radius);

    /// Finds the coordinates the top left and bottom right coordinates of a box containing all the points of the given
    /// cluster
    /// \param cluster the cluster points
    /// \param min_x
    /// \param min_y
    /// \param max_x
    /// \param max_y
    void find_max_min_coord(std::vector<cv::Point2f> cluster, double& min_x, double& min_y, double& max_x, double& max_y);

    /// Given the clusters, converts each of them to a Black and White image, applies the Hough transform and removes the
    /// cluster if at least one line is detected. This is because, if one line is detected, then you have a straight wall
    /// rather than a cylindrical object
    /// \param cluster_dict The set of clusters
    void keep_circles(std::unordered_map<int, std::vector<cv::Point2f>> &cluster_dict, float min_radius,
                      float max_radius, std::vector<cv::Point2f> &centers, std::vector<float>& radii);

    ///build an image given the points
    void build_image(std::vector<cv::Point2f>& points, cv::Mat& img);

    ///finds a circle passing through three points
    bool find_circle(cv::Point2f first, cv::Point2f mid, cv::Point2f last, float& xc, float& yc, float& r);

    /// Computes the L2 distance between 2 points
    /// \param a first point
    /// \param b second point
    /// \return L2 distance between a and b
    float distance(const cv::Point2f &a, const cv::Point2f &b);

    /// Given a set of points, it returns the lines found in the image
    /// \param cluster_points the cluster points over which you should look for lines
    /// \param lines the resulting lines
    void detect_lines(std::vector<cv::Point2f>& cluster_points, std::vector<cv::Point2f>& lines);
}




#endif //INTELLIGENT_ROBOTICS_ASSIGNMENT1_UTILITIES_H
