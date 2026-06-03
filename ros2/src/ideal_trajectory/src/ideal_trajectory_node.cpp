#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rclcpp::Node>("ideal_trajectory_node"));
    rclcpp::shutdown();
    return 0;
}