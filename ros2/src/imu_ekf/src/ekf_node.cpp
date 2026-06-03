#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imu_ekf/ekf.hpp"
#include "nav_msgs/msg/odometry.hpp"

class EKFNode : public rclcpp::Node
{
public:
    EKFNode() : Node("ekf_node") {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/raw", 10, 
            std::bind(&EKFNode::imuCallback, this, std::placeholders::_1));
    }
private:
    EKF ekf_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // get current timestamp
        rclcpp::Time current_stamp = rclcpp::Time(msg->header.stamp);
        // check whether last time stamp was 0
        if(last_stamp_.nanoseconds() == 0) {
            last_stamp_ = current_stamp;
            return;
        }
        // get accel values from imu
        Eigen::Vector3d accel(
            msg->linear_acceleration.x, 
            msg->linear_acceleration.y, 
            msg->linear_acceleration.z);
        // get gyro values from imu
        Eigen::Vector3d gyro(
            msg->angular_velocity.x, 
            msg->angular_velocity.y, 
            msg->angular_velocity.z);
        // compute time delta
        double dt = (current_stamp - last_stamp_).seconds();
        ekf_.predict(accel, gyro, dt);
        last_stamp_ = current_stamp;
    }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}