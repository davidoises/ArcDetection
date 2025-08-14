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
    const std::string dir_path = file_path.substr(0, file_path.find_last_of("/\\")) + "/../../arc_data/raw/";

    rclcpp::init(argc, argv);

    auto adc = std::make_shared<ARC::ADC::ADCModel>(dir_path);

    const bool success = adc->load_file();
    adc->show_file();

    if (success)
    {
        rclcpp::spin(adc);
        rclcpp::shutdown();   
    }

    return 0;
}