#pragma once

#include "common_definitions.h"
#include "detection_types.h"

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

            // Some parameters that define the model behavior
            static constexpr std::size_t BUFFER_SIZE = 512u;
            static constexpr std::size_t WINDOW_SIZE = 15u;

            // Containers for data processing operations
            std::array<float, BUFFER_SIZE> fft_buffer_;
            size_t fft_buffer_index_;
            std::array<ArcFeatures_U, WINDOW_SIZE> detection_window_;

            // Output data
            OutputMsgType message_;

            rclcpp::Subscription<InputMsgType>::SharedPtr subscription_;
            rclcpp::Publisher<OutputMsgType>::SharedPtr publisher_;

            /**
             * @brief Callback to process new data received from sensors.
             * 
             * @param input_msg The message received on the measurement topic
             */
            void process(const InputMsgType::UniquePtr input_msg);

            /**
             * @brief Triggered on buffer full, it extracts the FFT results.
             * 
             */
            void process_buffer();
    };
}