#include "detection_model.h"

// System includes
#include <chrono> // std::chrono
#include <thread> // std::this_thread
#include <deque> // std::deque
#include <cmath> // std::fabs, std::log10, std::cos
#include <sstream> // std::ostringstream

namespace ARC::DETECT
{

    DetectionModel::DetectionModel() :
        Node("detection_model"),
        prev_index_(0u)
    {
        // ROS objects

        subscription_ = create_subscription<InputMsgType>(ARC::DEFS::current_measurement_topic, 10, [this](InputMsgType::UniquePtr msg){
            process(std::move(msg));
        });

        result_publisher_ = create_publisher<OutputMsgType>(ARC::DEFS::detection_result_topic, 10);
        fft_publisher_ = create_publisher<ARC::DEFS::FFTDataType>(ARC::DEFS::detection_fft_topic, 10);
        raw_features_publisher_ = create_publisher<ARC::DEFS::MLFeaturesDataType>(ARC::DEFS::raw_features_topic, 10);
        features_publisher_ = create_publisher<ARC::DEFS::MLFeaturesDataType>(ARC::DEFS::features_topic, 10);

        // FFT configuration
        fft_algo_plan_ = fftw_plan_dft_r2c_1d(BUFFER_SIZE, fft_buffer_.data(), fft_res_.data(), FFTW_ESTIMATE);

        // Hann window for spectrogram
        for (size_t i = 0u; i < BUFFER_SIZE; i++)
        {
            hann_window[i] = 0.5 - 0.5 * std::cos(2.0 * M_PI * static_cast<double>(i) / static_cast<double>(BUFFER_SIZE - 1u));
        }
    }

    DetectionModel::~DetectionModel()
    {
        fftw_destroy_plan(fft_algo_plan_);
    }

    void DetectionModel::process(const InputMsgType::UniquePtr input_msg)
    {

        // Keep pushing until buffer full
        if (fft_rolling_buffer_.size() < BUFFER_SIZE)
        {
            fft_rolling_buffer_.push_back(input_msg->data);
        }
        else
        {
            // If the buffer is full start shifting data by step size and adding new elements at the end
            fft_rolling_buffer_.erase(fft_rolling_buffer_.begin(), fft_rolling_buffer_.begin() + STEP);
            fft_rolling_buffer_.push_back(input_msg->data);
        }

        // Process the buffer once full
        if (fft_rolling_buffer_.size() == BUFFER_SIZE)
        {
            // Measure the index difference every time the FFT is processed
            const int32_t measured_step = input_msg->index - prev_index_;
            prev_index_ = input_msg->index;

            // This is a check to ensure that the buffer is getting processed every STEP inputs.
            // Skipping data can cause problems.                
            if (measured_step > static_cast<int32_t>(STEP) && input_msg->index != BUFFER_SIZE-1U)
            {
                RCLCPP_ERROR(get_logger(), "Incorrect step: %i", measured_step);
            }

            RCLCPP_DEBUG(get_logger(), "Step: %i Times: %f Data: '%f", measured_step, input_msg->time, input_msg->data);

            // A copy is made so that in the future the process feature could be done in a separate thread
            // an allow the rolling buffer to keep storing data in parallel
            std::copy(
                fft_rolling_buffer_.begin(),
                fft_rolling_buffer_.begin()+BUFFER_SIZE,
                fft_buffer_.begin()
            );
            process_buffer();
        }
        
    }

    void DetectionModel::process_buffer()
    {
        // Apply the Hann window to the buffer
        for (size_t i = 0u; i < BUFFER_SIZE; i++)
        {
            fft_buffer_[i] *= hann_window[i];
        }

        // Perform FFT
        fftw_execute(fft_algo_plan_);

        // Extract selected bins and sum from the full FFT
        ArcFeatures_U extracted_features;
        for (std::size_t i = 0; i < BUFFER_SIZE/2u; i++)
        {
            // Calculate the magnitude
            const double real = fft_res_[i][0];
            const double imag = fft_res_[i][1];
            const double magnitude = (real * real + imag * imag) / static_cast<double>(BUFFER_SIZE * BUFFER_SIZE);
            const double frequency = static_cast<double>(i) * BIN_WIDTH;

            // For the FFT sum take only the bins in the range of interest
            if (frequency > BINS_LOWER_LIMIT && frequency < BINS_UPPER_LIMIT)
            {
                extracted_features.data.abs_sum += std::fabs(magnitude);
            }

            // Select bins based on feature engineering
            if (i == 22u)
            {
                extracted_features.data.bin_10k = magnitude;
            }
            if (i == 25u)
            {
                extracted_features.data.bin_13k = magnitude;
            }
            if (i == 31u)
            {
                extracted_features.data.bin_16k = magnitude;
            }
            if (i == 33u)
            {
                extracted_features.data.bin_22k = magnitude;
            }

            // Store in the fft logging message
            fft_message_.magnitude = 10.0 * std::log10(magnitude + 1e-12); // Avoid log(0)
            fft_message_.frequency = frequency;
            // Publishing for FFT visualization
            // fft_publisher_->publish(fft_message_);
            // std::this_thread::sleep_for(std::chrono::microseconds(500));
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
            process_features();
        }
    }

    void DetectionModel::process_features()
    {
        // RCLCPP_DEBUG(get_logger(), "Raw: ");
        // for (size_t i = 0u; i < detection_window_.size(); i++)
        // {
        //     const ArcFeatures_U& raw_feats = detection_window_[i];

        //     std::ostringstream oss;
        //     for (size_t j = 0u; j < raw_feats.cols.size(); j++)
        //     {
        //         oss << std::fixed << std::scientific << std::setprecision(8) << std::setw(10) << raw_feats.cols[j] << " | ";
        //     }
        //     RCLCPP_DEBUG(get_logger(), "index %li: %s", i, oss.str().c_str());
        // }

        // Find the min and max for the first half of the detection window
        ArcFeatures_U min_values = detection_window_.front();
        ArcFeatures_U max_values = detection_window_.front();
        for (size_t i = 0u; i < detection_window_.size()/2u; i++)
        {
            const ArcFeatures_U& raw_feats = detection_window_[i];
            for (size_t j = 0u; j < raw_feats.cols.size(); j++)
            {
                // Compare for min value
                if (raw_feats.cols[j] < min_values.cols[j])
                {
                    min_values.cols[j] = raw_feats.cols[j];
                }

                // Compare for max value
                if (raw_feats.cols[j] > max_values.cols[j])
                {
                    max_values.cols[j] = raw_feats.cols[j];
                }
            }
        }

        // RCLCPP_DEBUG(get_logger(), "Min: ");
        // {
        //     std::ostringstream oss;
        //     for (size_t j = 0u; j < min_values.cols.size(); j++)
        //     {
        //         oss << std::fixed << std::scientific << std::setprecision(8) << std::setw(10) << min_values.cols[j] << " | ";
        //     }
        //     RCLCPP_DEBUG(get_logger(), "%s", oss.str().c_str());
        // }

        // RCLCPP_DEBUG(get_logger(), "Max: ");
        // {
        //     std::ostringstream oss;
        //     for (size_t j = 0u; j < max_values.cols.size(); j++)
        //     {
        //         oss << std::fixed << std::scientific << std::setprecision(8) << std::setw(10) << max_values.cols[j] << " | ";
        //     }
        //     RCLCPP_DEBUG(get_logger(), "%s", oss.str().c_str());
        // }

        // Apply min-max scaling to the whole window
        std::array<ArcFeatures_U, WINDOW_SIZE> scaled_window;
        for (size_t i = 0u; i < detection_window_.size(); i++)
        {
            ArcFeatures_U scaled_feats = detection_window_[i];
            for (size_t j = 0u; j < scaled_feats.cols.size(); j++)
            {
                scaled_feats.cols[j] = (scaled_feats.cols[j] - min_values.cols[j])/(max_values.cols[j] - min_values.cols[j]);
            }
            scaled_window[i] = scaled_feats;
        }

        // RCLCPP_DEBUG(get_logger(), "Scaled: ");
        // for (size_t i = 0u; i < scaled_window.size(); i++)
        // {
        //     const ArcFeatures_U& raw_feats = scaled_window[i];

        //     std::ostringstream oss;
        //     for (size_t j = 0u; j < raw_feats.cols.size(); j++)
        //     {
        //         oss << std::fixed << std::setprecision(10) << std::setw(10) << raw_feats.cols[j] << " | ";
        //     }
        //     RCLCPP_DEBUG(get_logger(), "index %li: %s", i, oss.str().c_str());
        // }

        ml_input_ = {};
        // Sum all the scaled features into a single value (aggregate them)
        for (size_t i = scaled_window.size()/2u; i < scaled_window.size(); i++)
        {
            const ArcFeatures_U& scaled_feats = scaled_window[i];
            for (size_t j = 0u; j < scaled_feats.cols.size(); j++)
            {
                ml_input_.cols[j] += scaled_feats.cols[j];
            }
        }

        // Divide the sums to get the average
        for (size_t j = 0u; j < ml_input_.cols.size(); j++)
        {
            ml_input_.cols[j] /= static_cast<float>(scaled_window.size()/2u);
        }

        // Publish features for visualization
        features_publisher_->publish(ml_input_.data);

        process_ml_model();
    }

    void DetectionModel::process_ml_model()
    {
        float result = 0.0f;
        for (size_t i = 0u; i < ml_input_.cols.size(); i++)
        {
            result += ml_input_.cols[i]*ML_COEFS[i];
        }
        result += ML_INTERCEPT;

        bool detected = false;
        if (result > 0.0f)
        {
            detected = true;
        }

        result_message_.decision_function = result;
        result_message_.result = detected;
        result_publisher_->publish(result_message_);
    }

}