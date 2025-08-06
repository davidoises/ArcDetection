#pragma once

// System includes
#include <string> // std::string

// ROS includes
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

namespace ARC::DEFS
{
    using AdcDataType = std_msgs::msg::Float32;
    using DetectionDataType = std_msgs::msg::Bool;

    const std::string current_measurement_topic = "arc/current_measurement";
    const std::string detection_resul_topic = "arc/detection_result";
}