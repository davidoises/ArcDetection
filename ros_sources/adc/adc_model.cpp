#include "adc_model.h"

// System includes
#include <filesystem> // std::filesystem::exists
#include <fstream> // std::ifstream
#include <sstream> // std::stringstream, std::ostringstream
#include <vector> // std::vector
#include <iomanip>  // std::fixed, std::setprecision, std::setw


using namespace std::chrono_literals;

namespace ARC::ADC
{

    ADCModel::ADCModel() :
        Node("adc_model"),
        row_index_(0u),
        execution_period_(500us)
    {
        publisher_ = create_publisher<MsgType>(ARC::DEFS::current_measurement_topic, 10);

        timer_ = create_wall_timer(execution_period_, [this](){
            process();
        });
    }

    void ADCModel::process()
    {
        if (publisher_->get_subscription_count() < 2u)
        {
            // Wait for PlotJuggler and the detection model
            return;
        }

        if (row_index_ == 0u)
        {
            RCLCPP_INFO(get_logger(), "Starting to publish data");
        }

        if (row_index_ < input_file_rows_.size())
        {
            message_.data = input_file_rows_[row_index_][1u];
            RCLCPP_DEBUG(get_logger(), "Data: '%f'", message_.data);
            publisher_->publish(message_);
            row_index_++;
        }
        else
        {
            // Terminate the node. Nothing else to do here.
            RCLCPP_INFO(get_logger(), "No more lines to publish after %li lines processed", row_index_);
            timer_->cancel();
            rclcpp::shutdown();
        }
    }

    bool ADCModel::load_file(const std::string& file_name)
    {
        // First check if file exists
        if (!std::filesystem::exists(file_name))
        {
            RCLCPP_ERROR(get_logger(), "File not found: '%s'", file_name.c_str());
            return false;
        }

        // Open the file
        RCLCPP_INFO(get_logger(), "Opening file: '%s'", file_name.c_str());
        std::ifstream file(file_name);

        // Check if opened correctly
        if (!file.is_open())
        {
            return false;
        }

        // Process the file
        RCLCPP_INFO(get_logger(), "Processing the file");
        std::string line;
        while (std::getline(file, line))
        {
            // Create a row from a csv line
            std::vector<float> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ','))
            {
                // Safe conversion to float
                try
                {
                    const float value = std::stof(cell);
                    row.push_back(value);
                }
                catch(const std::invalid_argument& e)
                {
                    RCLCPP_ERROR(get_logger(), "Error parsing file in '%s' for cell %s at line %li", e.what(), cell.c_str(), input_file_rows_.size());
                    return false;
                }
            }

            // Store the row
            input_file_rows_.push_back(row);
        }

        RCLCPP_INFO(get_logger(), "Successfully parsed the file");
        return true;
    }

    void ADCModel::show_file(const size_t number_of_lines) const
    {
        // Display file structure
        const size_t number_of_rows = input_file_rows_.size();
        const size_t number_of_columns = (number_of_rows > 0u) ? input_file_rows_[0u].size() : 0u;
        RCLCPP_INFO(get_logger(), "File contains %li rows and %li columns", number_of_rows, number_of_columns);

        // Print line by line
        size_t line_count = 0u;
        for (const std::vector<float>& row : input_file_rows_)
        {

            // Stop after reaching the specified number of lines
            if (line_count == number_of_lines)
            {
                break;
            }

            // Formate the line into a human readable string
            std::ostringstream oss;
            for (const float value : row) {
                oss << std::fixed << std::setprecision(6) << std::setw(10) << value << " | ";
            }

            line_count++;

            RCLCPP_INFO(get_logger(), "Line %li: %s", line_count, oss.str().c_str());
        }
    }
}
