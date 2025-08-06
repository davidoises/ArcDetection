#pragma once

#include "common_definitions.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

namespace ARC::ADC
{
    class ADCModel : public rclcpp::Node
    {
        public:
            using MsgType = ARC::DEFS::MsgType;

            ADCModel();

        private:

            void process();

            // Output data
            MsgType message_;

            // Periodic execution of the adc process
            rclcpp::TimerBase::SharedPtr timer_;
            std::chrono::milliseconds execution_period_;

            rclcpp::Publisher<MsgType>::SharedPtr publisher_;
    };
}