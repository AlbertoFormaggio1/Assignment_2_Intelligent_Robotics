#include <assignment2/utilities.h>
#include <assignment2/scan_clusterizer.h>
#include <ros/ros.h>
#include <cmath>
#include <math.h>

void Utilities::build_image(std::vector<cv::Point2f> &points, cv::Mat &img) {
    double min_x, min_y, max_x, max_y;

    // Get the bounding box containing the full image
    Utilities::find_max_min_coord(points, min_x, min_y, max_x, max_y);

    //Create the image corresponding to each cluster
    //ROS_INFO("%f %f %f %f", min_x, max_x, min_y, max_y);
    int width = static_cast<int>((max_x - min_x) / IMAGE_RESOLUTION) + 10;
    int height = static_cast<int>((max_y - min_y) / IMAGE_RESOLUTION) + 10;
    //ROS_INFO("Width: %d, Height: %d", width, height);

    //image with only 0 (black image)
    cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));
    // Generate an image where there is a white pixel for each element that was read by the lidar
    for (int i = 0; i < points.size(); i++) {
        int x_image = static_cast<int>((points[i].x - min_x) / IMAGE_RESOLUTION) + 5;
        int y_image = static_cast<int>((points[i].y - min_y) / IMAGE_RESOLUTION) + 5;

        image.at<uchar>(y_image, x_image) = 255;
    }

    img = image;
}

void Utilities::detect_lines(std::vector<cv::Point2f> &cluster_points, std::vector<cv::Point2f> &lines) {
    // Find and remove the lines
    cv::Mat image_before_lines;
    Utilities::build_image(cluster_points, image_before_lines);
    //Apply the Hough Lines Transform to detect lines in the image

    cv::HoughLines(image_before_lines, lines, 1, CV_PI / 180, 20);
}


void Utilities::keep_circles(std::unordered_map<int, std::vector<cv::Point2f>> &cluster_dict, float min_radius,
                             float max_radius, std::vector<cv::Point2f> &centers, std::vector<float>& radii) {
    //given a cluster transform it into a binary image

    for (const auto &entry: cluster_dict) {

        // If the vector of points is empty, it means the cluster was removed from the set of relevant clusters
        if (entry.second.empty())
            continue;

        // Get the cluster
        std::vector<cv::Point2f> cluster_points = entry.second;
        std::vector<cv::Point2f> lines;

        Utilities::detect_lines(cluster_points, lines);

        // If no line was found, save the cluster, it probably contains a circle. Otherwise, simply don't do anything
        // (this would discard the cluster)
        if (!lines.empty()) {
            ROS_INFO("%d cluster was removed", entry.first);
            cluster_dict[entry.first].clear();
        } else {
            // Hough circles doesn't work

            // Get 3 points to fit a circle
            cv::Point2f first;
            cv::Point2f mid;
            cv::Point2f last;

            first = cluster_points[4];
            mid = cluster_points[cluster_points.size() / 2];
            last = cluster_points[cluster_points.size() - 4];

            //coordinates of the circle if exists
            float x;
            float y;
            //ray of the circle if exists
            float r;
            ROS_INFO("Number of points in the cluster %d", cluster_points.size());
            //try to fit a circle , otherwise it is just a straight line or a noisy pattern
            if (!Utilities::find_circle(first, mid, last, x, y, r)) {
                ROS_INFO("%d cluster was removed, it is a perfectly straight line", entry.first);
                cluster_dict[entry.first].clear();
            } else {
                cv::Point2f c(x, y);
                ROS_INFO("The center is xc = %f; yc = %f", c.x, c.y);
                ROS_INFO("The radius is %f", r);
                if (r < min_radius || r > max_radius) {
                    ROS_INFO("%d cluster has radius out of range, circle discarded", entry.first);
                    cluster_dict[entry.first].clear();
                }
                else if (!Utilities::is_circle(cluster_points, c, r, max_radius)) {
                    ROS_INFO("%d cluster was removed, it is too far from being a circle", entry.first);
                    cluster_dict[entry.first].clear();
                }
                else {
                    centers.push_back(c);
                    radii.push_back(r);
                }
            }
            ROS_INFO("_------------------------END OBJECT-----------------------------");
        }
    }
}

float Utilities::distance(const cv::Point2f &a, const cv::Point2f &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}


bool Utilities::is_circle(std::vector<cv::Point2f> points, cv::Point2f center, float r, float max_radius) {
    // if the radius of the circle is larger than a given threshold, then it's likely that the circle was fitted over
    // 3 points belonging to a line
    if (r > max_radius)
        return false;

    float angle_90 = 1.57;
    float th_angle = 0.1;
    // Reject corners
    for(int i = 1; i < points.size() - 1; i++){
        float m1 = (points[i].y - points[i - 1].y) / (points[i].x - points[i - 1].x);
        float m2 = (points[i].y - points[i + 1].y) / (points[i].x - points[i + 1].x);

        float angle = std::atan(abs((m1 - m2)/(1. + m1*m2)));

        if(abs(angle-angle_90) < th_angle) {
            ROS_INFO("Corner detected: %f", angle);
            return false;
        }
    }


    float MAE = 0;
    float threshold = 0.15;

    // Compute the Mean Absolute Error
    for (int i = 0; i < points.size(); i++) {
        float dist = Utilities::distance(center, points[i]);
        MAE += std::abs(r - dist);
    }

    MAE /= points.size();
    ROS_INFO("%f", MAE);

    // It's a circle only if the MAE is below a given threshold
    return MAE <= threshold;
}

bool Utilities::find_circle(cv::Point2f first, cv::Point2f mid, cv::Point2f last, float &xc, float &yc, float &r) {
    float th = 0.01;

    //Compute the slope of the 2 lines
    float s1 = (mid.y - first.y) / (mid.x - first.x);
    float s2 = (last.y - mid.y) / (last.x - mid.x);

    // Computing the midpoint of each segment
    float my1 = (mid.y + first.y) / 2;
    float mx1 = (mid.x + first.x) / 2;
    float my2 = (mid.y + last.y) / 2;
    float mx2 = (mid.x + last.x) / 2;
    // if the slopes of the 2 lines are too close, it is probably a line
    if (std::abs(s1 - s2) <= th) {
        return false;
    }

    // Compute the coordinates of the center and the radius
    xc = (s2 * mx2 - s1 * mx1 + my2 - my1) / (s2 - s1);
    yc = -s1 * (xc - mx1) + my1;
    r = distance(cv::Point2f(xc, yc), cv::Point2f(first.x, first.y));
    return true;
}

void
Utilities::find_max_min_coord(std::vector<cv::Point2f> cluster, double &min_x, double &min_y, double &max_x,
                              double &max_y) {
    min_x = min_y = 10000;
    max_x = max_y = -1000;

    //Find boundaries of the image to detect the size of a square fully containing the cluster
    for (int i = 0; i < cluster.size(); i++) {
        if (cluster[i].x < min_x)
            min_x = cluster[i].x;
        if (cluster[i].x > max_x)
            max_x = cluster[i].x;

        if (cluster[i].y < min_y)
            min_y = cluster[i].y;
        if (cluster[i].y > max_y)
            max_y = cluster[i].y;
    }
}

