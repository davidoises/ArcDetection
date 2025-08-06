#pragma once

#include "common_definitions.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

namespace ARC::DETECT
{
    class DetectionModel : public rclcpp::Node
    {
        public:

            using InputMsgType = ARC::DEFS::AdcDataType;
            using OutputMsgType = ARC::DEFS::DetectionDataType;

            DetectionModel();

        private:

            // Output data
            OutputMsgType message_;

            void process(const InputMsgType::UniquePtr msg);

            rclcpp::Subscription<InputMsgType>::SharedPtr subscription_;
            rclcpp::Publisher<OutputMsgType>::SharedPtr publisher_;
    };
}