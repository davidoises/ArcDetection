#include "dma_model.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

// System includes
#include <memory> // std::make_shared

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARC::DMA::DMAModel>());
    rclcpp::shutdown();

    return 0;
}