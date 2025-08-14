#include "dma_model.h"


namespace ARC::DMA
{

    DMAModel::DMAModel() :
        Node("dma_model"),
        fft_index_(0u),
        prev_input_index_(0u),
        subscription_
        (
            create_subscription<InputMsgType>(DEFS::CURRENT_MEASUREMENT_TOPIC, 10, [this](InputMsgType::UniquePtr msg){
                process(std::move(msg));
            })
        ),
        fft_buffer_publisher_(create_publisher<OutputMsgType>(DEFS::DMA_BUFFER_TOPIC, 10))
    {
    }

    void DMAModel::process(const InputMsgType::UniquePtr input_msg)
    {
        // Keep pushing until buffer full
        if (fft_rolling_buffer_.data.size() < DEFS::FFT_BUFFER_SIZE)
        {
            fft_rolling_buffer_.data.push_back(input_msg->data);
        }
        else
        {
            // Shift elements left by SPECGRAM_STEP
            std::move(fft_rolling_buffer_.data.begin() + DEFS::SPECGRAM_STEP,
                      fft_rolling_buffer_.data.end(),
                      fft_rolling_buffer_.data.begin());

            // Resize to remove trailing elements
            fft_rolling_buffer_.data.resize(DEFS::SPECGRAM_OVERLAP);

            fft_rolling_buffer_.data.push_back(input_msg->data);
        }

        // Process the buffer once full
        if (fft_rolling_buffer_.data.size() == DEFS::FFT_BUFFER_SIZE)
        {
            // Measure the index difference every time the FFT is processed
            const int32_t measured_step = input_msg->index - prev_input_index_;
            prev_input_index_ = input_msg->index;

            // This is a check to ensure that the buffer is getting processed every STEP inputs.
            // Skipping data can cause problems.                
            if (measured_step > static_cast<int32_t>(DEFS::SPECGRAM_STEP) && input_msg->index != DEFS::FFT_BUFFER_SIZE-1U)
            {
                RCLCPP_ERROR(get_logger(), "Incorrect step: %i, shutting down", measured_step);
                rclcpp::shutdown();
            }

            RCLCPP_DEBUG(get_logger(), "Step: %i Buffer size: %li Time: %f", measured_step, fft_rolling_buffer_.data.size(), input_msg->time);

            // Publish the buffer
            fft_rolling_buffer_.index = fft_index_;
            fft_buffer_publisher_->publish(fft_rolling_buffer_);

            fft_index_++;
        }
    }
    
}