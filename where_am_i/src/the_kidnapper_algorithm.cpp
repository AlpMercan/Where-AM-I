#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <thread>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <vector>

class Matching {
public:
    static void publishTransform(double center_x, double center_y) {
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped t;

        t.header.stamp = ros::Time::now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        // Convert pixel coordinates to map coordinates
        auto [map_x, map_y] = pixelToMapConversion(center_x, center_y);

        // Set the position
        t.transform.translation.x = map_x;
        t.transform.translation.y = map_y;
        t.transform.translation.z = 0.0;

        // Set the orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        br.sendTransform(t);
    }

    static std::pair<double, double> pixelToMapConversion(double pixel_x, double pixel_y) {
        return {-pixel_x * 0.05, -pixel_y * 0.05}; // pixel to real life measurement conversion
    }

    static void publishInitialPose(double x, double y, double theta) {
        ros::NodeHandle nh;
        ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
        ros::Duration(1).sleep(); // Give time for the publisher to set up

        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = ros::Time::now();
        initial_pose.header.frame_id = "map";

        initial_pose.pose.pose.position.x = x;
        initial_pose.pose.pose.position.y = y;
        initial_pose.pose.pose.position.z = 0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, theta);
        initial_pose.pose.pose.orientation.x = quaternion.x();
        initial_pose.pose.pose.orientation.y = quaternion.y();
        initial_pose.pose.pose.orientation.z = quaternion.z();
        initial_pose.pose.pose.orientation.w = quaternion.w();

        initial_pose_pub.publish(initial_pose);
    }

    static void matchTemplate(const std::string& image_path, const std::string& template_path) {
        cv::Mat bg = cv::imread(image_path);
        cv::cvtColor(bg, bg, cv::COLOR_BGR2RGB);
        cv::Mat face = cv::imread(template_path);
        cv::cvtColor(face, face, cv::COLOR_BGR2RGB);

        int height = face.rows;
        int width = face.cols;

        cv::Mat result;
        cv::matchTemplate(bg, face, result, cv::TM_CCOEFF);

        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

        cv::Point top_left = maxLoc;
        double center_x1 = top_left.x + width / 2;
        double center_y1 = top_left.y + height / 2;
        double center_x = std::abs((-center_x1)) * 0.05;
        double center_y = std::abs((-center_y1)) * 0.05;
        cv::Point bottom_right = top_left + cv::Point(width, height);

        // Publish the transform and initial pose
        // publishTransform(center_x, center_y); // Uncomment if needed
        publishInitialPose(center_x, center_y, 0);
        cv::rectangle(bg, top_left, bottom_right, cv::Scalar(255, 0, 0), 10);

        // Display the images
        cv::imshow("Result of Template Matching", result);
        cv::imshow("Match Point", bg);
        cv::waitKey(0);
    }
};


class Map {
public:
    static cv::Scalar mapValueToColor(int value) {
        if (value == -1) {
            // Unknown or unexplored
            return cv::Scalar(128, 128, 128); // Gray
        } else if (value <= 30) {
            // Free space (0-30)
            return cv::Scalar(255, 255, 255); // White
        } else if (value <= 70) {
            // Unknown space (30-70)
            return cv::Scalar(0, 0, 255); // Red
        } else {
            // Occupied space (70-100)
            return cv::Scalar(0, 255, 0); // Green
        }
    }

    static void callback(const nav_msgs::OccupancyGrid::ConstPtr& data) {
        int width = data->info.width;
        int height = data->info.height;
        std::vector<int8_t> data_vector(data->data.begin(), data->data.end());

        cv::Mat color_image(height, width, CV_8UC3);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                cv::Scalar color = mapValueToColor(data_vector[index]);
                color_image.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }

        cv::imwrite("/home/alp/catkin_ws/src/turtlebot3_kalman_filter/src/local_colored.png", color_image);
    }

    static void listener() {
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("/move_base/local_costmap/costmap", 1000, callback);
        ros::spin();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Kidnapping_Prevention_Node");
    std::thread listener_thread(&Map::listener);
    listener_thread.detach(); // Detach the thread so that it runs independently

    std::string image_path = "/home/alp/catkin_ws/src/turtlebot3_kalman_filter/src/alp.png";
    std::string template_path = "/home/alp/catkin_ws/src/turtlebot3_kalman_filter/src/local_colored.png";
    
    try {
        Matching::matchTemplate(image_path, template_path);
    } catch (const ros::Exception& e) {
        ROS_ERROR("ROS exception: %s", e.what());
    }
    ros::shutdown();
    ROS_INFO("Location Founded");
    

    return 0;
}
