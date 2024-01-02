#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense> 

#include <cstdlib>
#include <cmath>

class AnomalyDetector {
public:
    AnomalyDetector() {
        ros::NodeHandle nh;

        imu_subscriber = nh.subscribe("/imu", 1000, &AnomalyDetector::imuCallback, this);

        acceleration_threshold = 10.0; // Modify as needed acording to max values of your robot
        angular_velocity_threshold = 10.0; // Modify as needed acording to max values of your robot
        first_start = true;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& data) {
        double linear_acceleration = std::sqrt(std::pow(data->linear_acceleration.x, 2) +
                                               std::pow(data->linear_acceleration.y, 2) +
                                               std::pow(data->linear_acceleration.z, 2));

        double angular_velocity = std::sqrt(std::pow(data->angular_velocity.x, 2) +
                                            std::pow(data->angular_velocity.y, 2) +
                                            std::pow(data->angular_velocity.z, 2));

        if (linear_acceleration > acceleration_threshold ||
            angular_velocity > angular_velocity_threshold ||
            first_start) {
            ROS_INFO("Anomaly detected in IMU data! Activating kidnapper algorithm.");
            first_start = false;
            activateKidnapperAlgorithm();
        } else {
            ROS_INFO("IMU data within normal range.");
        }
    }

private:
    ros::Subscriber imu_subscriber;
    double acceleration_threshold;
    double angular_velocity_threshold;
    bool first_start;

    void activateKidnapperAlgorithm() {
        ROS_INFO("Activating kidnapper algorithm as a separate process.");
        std::system("/home/alp/catkin_ws/src/turtlebot3_kalman_filter/src/the_kidnapper_algorithm");
}

};
int main(int argc, char **argv) {
    ros::init(argc, argv, "anomaly_detector_node_cpp"); 

    AnomalyDetector anomalyDetector; 

    ros::spin();

    return 0;
}
