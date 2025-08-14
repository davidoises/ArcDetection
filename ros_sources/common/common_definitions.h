#pragma once

// Custom message
#include "arc_detection/msg/fft.hpp"
#include "arc_detection/msg/fft_buffer.hpp"
#include "arc_detection/msg/ml_features.hpp"
#include "arc_detection/msg/ml_result.hpp"
#include "arc_detection/msg/indexed_series.hpp"
#include "arc_detection/msg/time_series.hpp"

// ROS includes
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

namespace ARC::DEFS
{
    // Data types
    using AdcDataType = arc_detection::msg::TimeSeries;
    using DmaDataType = arc_detection::msg::FFTBuffer;
    using DetectionDataType = arc_detection::msg::MLResult;
    using FFTDataType = arc_detection::msg::FFT;
    using MLFeaturesDataType = arc_detection::msg::MLFeatures;

    // Topics
    constexpr const char* CURRENT_MEASUREMENT_TOPIC    = "arc/current_measurement";
    constexpr const char* DMA_BUFFER_TOPIC             = "arc/dma_buffer";
    constexpr const char* DETECTION_RESULT_TOPIC       = "arc/detection_result";
    constexpr const char* DETECTION_FFT_TOPIC          = "arc/detection_fft";
    constexpr const char* RAW_FEATURES_TOPIC           = "arc/raw_features";
    constexpr const char* FEATURES_TOPIC               = "arc/features";

    // Spectrogram parameters
    constexpr float  SAMPLING_RATE       = 250000;
    constexpr size_t FFT_BUFFER_SIZE     = 512u;
    constexpr float  FFT_BIN_WIDTH       = SAMPLING_RATE/static_cast<float>(FFT_BUFFER_SIZE);
    constexpr size_t SPECGRAM_OVERLAP    = 128u;
    constexpr size_t SPECGRAM_STEP       = FFT_BUFFER_SIZE - SPECGRAM_OVERLAP;
}