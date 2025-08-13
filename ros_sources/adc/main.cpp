#include "adc_model.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"

// System includes
#include <memory> // std::make_shared
#include <string> // std::string

int main(int argc, char * argv[])
{

    // Find path of current directory to process the input file
    const std::string file_path = __FILE__;
    const std::string dir_path = file_path.substr(0, file_path.find_last_of("/\\"));
    // const std::string input_file_path = dir_path + "/../../arc_data/raw/test_A0_B3_C3_D5_A300_B150_C200_D330_A0.00017_B0.00017_C0.00014_D0.00015_arc1.csv";
    const std::string input_file_path = dir_path + "/../../arc_data/raw/test_A0_B3_C3_D8_A300_B150_C200_D330_A1e-05_B6e-05_C0.00014_D0.00015_arc1.csv";

    rclcpp::init(argc, argv);

    auto adc = std::make_shared<ARC::ADC::ADCModel>();
    const bool success = adc->load_file(input_file_path);

    adc->show_file();

    if (success)
    {
        rclcpp::spin(adc);
        rclcpp::shutdown();   
    }

    return 0;
}