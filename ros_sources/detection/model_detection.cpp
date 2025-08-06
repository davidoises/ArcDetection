#include "model_detection.h"

namespace ARC::DETECT
{

    DetectionModel::DetectionModel() :
        Node("detection_model")
    {
        subscription_ = this->create_subscription<InputMsgType>(ARC::DEFS::current_measurement_topic, 10, [this](InputMsgType::UniquePtr msg){
            process(std::move(msg));
        });

        publisher_ = create_publisher<OutputMsgType>(ARC::DEFS::detection_resul_topic, 10);
    }

    void DetectionModel::process(const InputMsgType::UniquePtr msg)
    {
        static size_t input_count = 0u;

        RCLCPP_DEBUG(get_logger(), "Data: '%f'", msg->data);

        message_.data = false;
        if (input_count >= 30000)
        {
            message_.data = true;
        }
        
        publisher_->publish(message_);

        input_count++;
    }

}