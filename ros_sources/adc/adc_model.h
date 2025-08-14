#pragma once

#include "common_definitions.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

// System includes
#include <string> // std::string
#include <chrono> // std::chrono::microseconds

namespace ARC::ADC
{
    class ADCModel final : public rclcpp::Node
    {
        public:
            using MsgType = DEFS::AdcDataType;

            explicit ADCModel(const std::string base_path);

            /**
             * @brief Loads a CSV file as a vector of vectors of floats.
             *        Doesn't perform any format checks besides the conversion to float.
             *
             * @return bool Return true on success
             */
            bool load_file();

            /**
             * @brief Displays the format and some of the lines from the loaded file.
             * 
             * @param number_of_lines   Number of rows to be printed. Defaults to 5.
             */
            void show_file(const size_t number_of_lines = 5u) const;

        private:

            /**
             * @brief Periodic function that will publish the data to the corresponding topic.
             * 
             */
            void process();

            // File to read the data
            std::string base_path_;
            std::string file_name_;

            // Output data
            MsgType message_;

            // Loaded data from csv
            std::vector<std::vector<double>> input_file_rows_;

            // Line index to iterate through the file in a periodic call
            size_t row_index_;

            // Periodic execution of the adc process
            rclcpp::TimerBase::SharedPtr timer_;
            std::chrono::microseconds execution_period_{0};

            rclcpp::Publisher<MsgType>::SharedPtr publisher_;
    };
}