#include "detection_model.h"

// System includes
#include <chrono> // std::chrono
#include <thread> // std::this_thread
#include <deque> // std::deque
#include <cmath> // std::fabs, std::log10

namespace ARC::DETECT
{

    DetectionModel::DetectionModel() :
        Node("detection_model"),
        fft_buffer_{},
        fft_buffer_index_{0}
    {
        subscription_ = create_subscription<InputMsgType>(ARC::DEFS::current_measurement_topic, 10, [this](InputMsgType::UniquePtr msg){
            process(std::move(msg));
        });

        result_publisher_ = create_publisher<OutputMsgType>(ARC::DEFS::detection_result_topic, 10);
        fft_publisher_ = create_publisher<ARC::DEFS::FFTDataType>(ARC::DEFS::detection_fft_topic, 10);
        raw_features_publisher_ = create_publisher<ARC::DEFS::MLFeaturesDataType>(ARC::DEFS::raw_features_topic, 10);
        features_publisher_ = create_publisher<ARC::DEFS::MLFeaturesDataType>(ARC::DEFS::features_topic, 10);

        fft_algo_plan_ = fftw_plan_dft_r2c_1d(BUFFER_SIZE, fft_buffer_.data(), fft_res_.data(), FFTW_ESTIMATE);
    }

    DetectionModel::~DetectionModel()
    {
        fftw_destroy_plan(fft_algo_plan_);
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
        // Perform FFT
        fftw_execute(fft_algo_plan_);

        // Extract selected bins and sum from the full FFT
        ArcFeatures_U extracted_features;
        for (std::size_t i = 0; i < BUFFER_SIZE / 2; ++i)
        {
            // Calculate the magnitude
            const double real = fft_res_[i][0];
            const double imag = fft_res_[i][1];
            const double magnitude = real * real + imag * imag;
            const double magnitude_db = 10.0 * std::log10(magnitude + 1e-12); // Avoid log(0)

            // Store in the fft logging message
            fft_message_.magnitude = static_cast<float>(magnitude_db);
            fft_message_.frequency = static_cast<float>(i+1u) * BIN_WIDTH;

            // For the FFT sum take only the bins in the range of interest
            if (fft_message_.frequency > BINS_LOWER_LIMIT && fft_message_.frequency < BINS_UPPER_LIMIT)
            {
                extracted_features.data.abs_sum += std::fabs(fft_message_.magnitude);
            }

            // Select bins based on feature engineering
            if (i == 21u)
            {
                extracted_features.data.bin_10k = fft_message_.magnitude;
            }
            if (i == 26u)
            {
                extracted_features.data.bin_13k = fft_message_.magnitude;
            }
            if (i == 32u)
            {
                extracted_features.data.bin_16k = fft_message_.magnitude;
            }
            if (i == 46u)
            {
                extracted_features.data.bin_22k = fft_message_.magnitude;
            }

            // Publishing for FFT visualization
            fft_publisher_->publish(fft_message_);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }

        // Publish raw features for visualization
        raw_features_publisher_->publish(extracted_features.data);

        // Keep pushing until window full
        if (detection_window_.size() < WINDOW_SIZE)
        {
            detection_window_.push_back(extracted_features);
        }
        else
        {
            // If the window is full start shifting data and adding new elements at the end
            detection_window_.pop_front();
            detection_window_.push_back(extracted_features);
            preprocess_features();
        }


        // if (message_.data)
        // {
        //     message_.data = false;
        // }
        // else
        // {
        //     message_.data = true;
        // }
        // publisher_->publish(message_);
    }

    void DetectionModel::preprocess_features()
    {

    }

}