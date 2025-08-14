#pragma once

#include "common_definitions.h"
#include "detection_types.h"

// FFTW library
#include <fftw3.h>

// ROS includes
#include "rclcpp/rclcpp.hpp"

// System includes
#include <array> // std::array

namespace ARC::DETECT
{
    class DetectionModel final : public rclcpp::Node
    {
        public:

            using InputMsgType = DEFS::DmaDataType;
            using OutputMsgType = DEFS::DetectionDataType;

            DetectionModel();
            ~DetectionModel();

        private:

            // Some parameters that define the model behavior
            static constexpr std::size_t WINDOW_SIZE = 30u;
            static constexpr float BINS_LOWER_LIMIT = 10000;
            static constexpr float BINS_UPPER_LIMIT = 50000;

            static constexpr std::array<float, 5u> ML_COEFS = {0.00092834f, 0.00093834f, 0.00080337f, 0.00131207f, 0.00125999f};
            static constexpr float ML_INTERCEPT = -1.00331676f;

            // Containers for SVM model computation
            std::deque<ArcFeatures_U> detection_window_;
            ArcFeatures_U ml_input_;

            // Used by FFTW to calculate fft
            std::array<fftw_complex, DEFS::FFT_BUFFER_SIZE> fft_res_;
            fftw_plan fft_algo_plan_;

            // Containers for spectrogram computation
            std::array<double, DEFS::FFT_BUFFER_SIZE> fft_buffer_;
            std::array<double, DEFS::FFT_BUFFER_SIZE> hann_window;
            uint32_t prev_index_;

            // Output data
            OutputMsgType result_message_;
            DEFS::FFTDataType fft_message_;

            // ROS messaging objects
            rclcpp::Subscription<InputMsgType>::SharedPtr subscription_;
            rclcpp::Publisher<OutputMsgType>::SharedPtr result_publisher_;
            rclcpp::Publisher<DEFS::FFTDataType>::SharedPtr fft_publisher_;
            rclcpp::Publisher<DEFS::MLFeaturesDataType>::SharedPtr raw_features_publisher_;
            rclcpp::Publisher<DEFS::MLFeaturesDataType>::SharedPtr features_publisher_;

            /**
             * @brief Callback to process new data received from sensors.
             * 
             * @param input_msg The message received on the measurement topic
             */
            void process(const InputMsgType::UniquePtr input_msg);

            /**
             * @brief Triggered on buffer full, it extracts the FFT results.
             * 
             */
            void process_buffer();

            /**
             * @brief Triggered when theres new elements in the detection window.
             *        It processes the features in the window so that they can be used by the
             *        detection model.
             * 
             */
            void process_features();

            /**
             * @brief Triggered once features are ready for the SVM model. Executes the model based
             *        on the training resulting weights.
             * 
             */
            void process_ml_model();
    };
}