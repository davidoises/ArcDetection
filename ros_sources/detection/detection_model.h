#pragma once

#include "common_definitions.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

// System includes
#include <array> // std::array

namespace ARC::DETECT
{
    class DetectionModel : public rclcpp::Node
    {
        public:

            using InputMsgType = ARC::DEFS::AdcDataType;
            using OutputMsgType = ARC::DEFS::DetectionDataType;

            DetectionModel();

        private:

            // Containers for data processing operations
            std::array<float, 512> fft_buffer_;

            // Output data
            OutputMsgType message_;

            void process(const InputMsgType::UniquePtr msg);

            rclcpp::Subscription<InputMsgType>::SharedPtr subscription_;
            rclcpp::Publisher<OutputMsgType>::SharedPtr publisher_;
    };
}