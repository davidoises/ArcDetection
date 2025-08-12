#include "detection_model.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

// System includes
#include <memory> // std::make_shared

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARC::DETECT::DetectionModel>());
    rclcpp::shutdown();

    return 0;
}