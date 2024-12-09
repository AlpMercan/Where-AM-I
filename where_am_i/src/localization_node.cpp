#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <mutex>

class Map {
private:
    static std::mutex mtx;
    static cv::Mat latest_costmap;
    static bool map_received;
    
    static cv::Vec3b mapValueToColor(int8_t value) {
        if (value == -1) return cv::Vec3b(128, 128, 128);
        else if (value <= 30) return cv::Vec3b(255, 255, 255);
        else if (value <= 70) return cv::Vec3b(0, 0, 255);
        else return cv::Vec3b(0, 255, 0);
    }

public:
    static void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        int width = std::sqrt(msg->data.size());
        int height = width;
        
        cv::Mat color_image(height, width, CV_8UC3);
        
        #pragma omp parallel for collapse(2)
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                int idx = y * width + x;
                color_image.at<cv::Vec3b>(y, x) = mapValueToColor(msg->data[idx]);
            }
        }
        
        {
            std::lock_guard<std::mutex> lock(mtx);
            latest_costmap = color_image.clone();
            map_received = true;
        }
        
        cv::imwrite("/home/alp/catkin_ws/src/Where-AM-I/where_am_i/src/local_colored.png", color_image);
    }
    
    static cv::Mat getLatestCostmap() {
        std::lock_guard<std::mutex> lock(mtx);
        return latest_costmap.clone();
    }
    
    static bool isMapReceived() {
        std::lock_guard<std::mutex> lock(mtx);
        return map_received;
    }
};

std::mutex Map::mtx;
cv::Mat Map::latest_costmap;
bool Map::map_received = false;

class Matching {
private:
    ros::NodeHandle nh_;
    ros::Publisher initial_pose_pub_;
    tf2_ros::TransformBroadcaster br_;
    
    std::pair<double, double> pixelToMapConversion(double pixel_x, double pixel_y) {
        return {-pixel_x * 0.05, -pixel_y * 0.05};
    }

public:
    Matching() : initial_pose_pub_(nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10)) {}

    void publishInitialPose(double x, double y, double theta) {
        geometry_msgs::PoseWithCovarianceStamped pose;
        
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        
        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.position.z = 0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.pose.pose.orientation.x = q.x();
        pose.pose.pose.orientation.y = q.y();
        pose.pose.pose.orientation.z = q.z();
        pose.pose.pose.orientation.w = q.w();
        
        initial_pose_pub_.publish(pose);
    }
    
    void matchTemplate(const std::string& image_path, const cv::Mat& local_map) {
        cv::Mat global_map = cv::imread(image_path);
        cv::cvtColor(global_map, global_map, cv::COLOR_BGR2RGB);
        
        cv::Mat result;
        cv::matchTemplate(global_map, local_map, result, cv::TM_CCOEFF);
        
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);
        
        cv::Point top_left = max_loc;
        int width = local_map.cols;
        int height = local_map.rows;
        
        double center_x1 = top_left.x + width / 2;
        double center_y1 = top_left.y + height / 2;
        double center_x = std::abs(-center_x1) * 0.05;
        double center_y = std::abs(-center_y1) * 0.05;
        
        publishInitialPose(center_x, center_y, 0);
        
        // Optional: Visualize results
        cv::rectangle(global_map, top_left, cv::Point(top_left.x + width, top_left.y + height), cv::Scalar(255, 0, 0), 2);
        cv::imshow("Match Result", global_map);
        cv::waitKey(1000); // Show the result for 1 second
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Kidnapping_Prevention_Node");
    ros::NodeHandle nh;
    
    ros::Subscriber map_sub = nh.subscribe("/move_base/local_costmap/costmap", 1, Map::callback);
    
    Matching localization;
    std::string image_path = "/home/alp/catkin_ws/src/Where-AM-I/where_am_i/src/GLOBAL.png";
    
    // Wait for the first map to be received
    ros::Rate rate(10);
    while (ros::ok() && !Map::isMapReceived()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    // Process once and exit
    cv::Mat local_map = Map::getLatestCostmap();
    if (!local_map.empty()) {
        localization.matchTemplate(image_path, local_map);
    }
    
    return 0;
}