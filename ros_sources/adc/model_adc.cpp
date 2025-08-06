#include "model_adc.h"

using namespace std::chrono_literals;

namespace ARC::ADC
{

    ADCModel::ADCModel() :
        Node("adc_model"),
        execution_period_(500ms)
    {
        publisher_ = create_publisher<MsgType>(ARC::DEFS::current_measurement_topic, 10);

        timer_ = create_wall_timer(execution_period_, [this](){
            // this->process();
            process();
        });
    }

    void ADCModel::process()
    {
        message_.data++;
        RCLCPP_INFO(get_logger(), "Publishing: '%f'", message_.data);
        publisher_->publish(message_);
    }
}
