#pragma once

#include "common_definitions.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

namespace ARC::DETECT
{
    class DetectionModel : public rclcpp::Node
    {
        public:

            using MsgType = ARC::DEFS::MsgType;

            DetectionModel();

        private:

            void process(const MsgType::UniquePtr msg);

            rclcpp::Subscription<MsgType>::SharedPtr subscription_;
    };
}