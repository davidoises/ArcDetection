#pragma once

// Custom message
#include "arc_detection/msg/fft.hpp"
#include "arc_detection/msg/ml_features.hpp"
#include "arc_detection/msg/ml_result.hpp"
#include "arc_detection/msg/indexed_series.hpp"
#include "arc_detection/msg/time_series.hpp"

// ROS includes
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

// System includes
#include <string> // std::string

namespace ARC::DEFS
{
    // using AdcDataType = arc_detection::msg::IndexedSeries;
    using AdcDataType = arc_detection::msg::TimeSeries;
    using DetectionDataType = arc_detection::msg::MLResult;
    using FFTDataType = arc_detection::msg::FFT;
    using MLFeaturesDataType = arc_detection::msg::MLFeatures;

    const std::string current_measurement_topic = "arc/current_measurement";
    const std::string detection_result_topic = "arc/detection_result";
    const std::string detection_fft_topic = "arc/detection_fft";
    const std::string raw_features_topic = "arc/raw_features";
    const std::string features_topic = "arc/features";
}