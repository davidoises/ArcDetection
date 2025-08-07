#pragma once

#include <array> // std::array

namespace ARC::DETECT
{
    struct ArcFeatures
    {
        float bin_10k = 0.0f;
        float bin_13k = 0.0f;
        float bin_16k = 0.0f;
        float bin_22k = 0.0f;
        float abs_sum = 0.0f;
    };

    union ArcFeatures_U
    {
        ArcFeatures data = {};
        std::array<float, 5u> cols;
    };
    
}