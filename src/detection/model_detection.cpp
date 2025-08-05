#include "model_detection.h"

namespace ARC::DETECT
{

    DetectionModel::DetectionModel() :
        Node("detection_model")
    {
        subscription_ = this->create_subscription<MsgType>("current_measurement", 10, [this](MsgType::UniquePtr msg){
            process(std::move(msg));
        });
    }

    void DetectionModel::process(const MsgType::UniquePtr msg)
    {
        RCLCPP_INFO(get_logger(), "I heard: '%f'", msg->data);
    }

}