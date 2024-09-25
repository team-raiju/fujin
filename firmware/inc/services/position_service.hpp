#pragma once
#include "utils/math.h"

namespace services::position {

/// @section Constants
#define WHEEL_RADIUS_CM    (1.27)
#define WHEEL_PERIMETER_CM (M_TWOPI * WHEEL_RADIUS_CM)

#define WHEEL_TO_ENCODER_RATIO    (1.0) // 1 rotation of the wheel equals to WHEEL_TO_ENCODER_RATIO rotations of the encoder
#define ENCODER_PPR               (1024.0) // Pulses in one rotation of the encoder
#define PULSES_PER_WHEEL_ROTATION (WHEEL_TO_ENCODER_RATIO * ENCODER_PPR)

#define WHEELS_DIST_CM        (7.0)
#define ENCODER_DIST_CM_PULSE (WHEEL_PERIMETER_CM / PULSES_PER_WHEEL_ROTATION)


/// @section Interface definition
/// @brief Initializes the position service
void init(void);

/// @brief Resets the position service
void reset(void);

/// @brief  @brief Updates the position variables
void update(void);


/// @brief Returns the travelled distance in cm
float get_travelled_dist_cm(void);


}
