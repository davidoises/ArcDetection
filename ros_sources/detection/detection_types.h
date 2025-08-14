#pragma once

#include "common_definitions.h"

#include <array> // std::array

namespace ARC::DETECT
{

    union ArcFeatures_U
    {
        ARC::DEFS::MLFeaturesDataType data{};
        std::array<double, 5u> cols;
    };
    
}