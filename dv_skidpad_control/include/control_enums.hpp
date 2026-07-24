#pragma once

#include <string>

namespace skidpad_control
{

// =============================================================================
//                              DRIVE PHASE
// =============================================================================

enum class DrivePhase
{
    WAITING = 0,
    DRIVE,
    BRAKING,
    COASTING,
    FINISHED,
    EMERGENCY_STOP,
    EMERGENCY_STOP_HARD
};


// =============================================================================
//                         LATERAL CONTROLLER TYPE
// =============================================================================

enum class LateralControllerType
{
    NONE = 0,
    LTV_MPC_UNBOUNDED
};


// =============================================================================
//                              CONVERSION
// =============================================================================

std::string phaseToString(DrivePhase phase);

} // namespace skidpad_control
