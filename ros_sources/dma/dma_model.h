#pragma once

#include "common_definitions.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

// System includes
#include <array> // std::array

namespace ARC::DMA
{
    class DMAModel final : public rclcpp::Node
    {
        public:

            using InputMsgType = ARC::DEFS::AdcDataType;
            using OutputMsgType = ARC::DEFS::DmaDataType;

            DMAModel();

        private:

            // FFT data storage
            OutputMsgType fft_rolling_buffer_;
            uint32_t fft_index_;
            uint32_t prev_input_index_;

            // ROS messaging objects
            rclcpp::Subscription<InputMsgType>::SharedPtr subscription_;
            rclcpp::Publisher<OutputMsgType>::SharedPtr fft_buffer_publisher_;

            /**
             * @brief Callback to process new data received from adc.
             * 
             * @param input_msg The message received on the measurement topic
             */
            void process(const InputMsgType::UniquePtr input_msg);
    };
}