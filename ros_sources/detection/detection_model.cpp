#include "detection_model.h"

namespace ARC::DETECT
{

    DetectionModel::DetectionModel() :
        Node("detection_model"),
        fft_buffer_{},
        fft_buffer_index_{0}
    {
        subscription_ = this->create_subscription<InputMsgType>(ARC::DEFS::current_measurement_topic, 10, [this](InputMsgType::UniquePtr msg){
            process(std::move(msg));
        });

        publisher_ = create_publisher<OutputMsgType>(ARC::DEFS::detection_resul_topic, 10);
    }

    void DetectionModel::process(const InputMsgType::UniquePtr input_msg)
    {
        RCLCPP_DEBUG(get_logger(), "Data: '%f'", input_msg->data);

        if (fft_buffer_index_ < BUFFER_SIZE)
        {
            fft_buffer_[fft_buffer_index_] = input_msg->data;
            fft_buffer_index_++;
        }
        else
        {
            process_buffer();
            fft_buffer_index_ = 0u;
        }
        
    }

    void DetectionModel::process_buffer()
    {
        if (message_.data)
        {
            message_.data = false;
        }
        else
        {
            message_.data = true;
        }
        publisher_->publish(message_);
    }

}