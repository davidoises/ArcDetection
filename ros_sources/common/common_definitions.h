#pragma once

// System includes
#include <string> // std::string

// ROS includes
#include "std_msgs/msg/float32.hpp"

namespace ARC::DEFS
{
    using MsgType = std_msgs::msg::Float32;

    const std::string current_measurement_topic = "arc/current_measurement";
}