#include "acc_types.hpp"

namespace acc_launch_control
{

std::string phaseToString(DrivePhase phase)
{
    switch (phase)
    {
        case DrivePhase::WAITING: return "WAITING";
        case DrivePhase::LAUNCH: return "LAUNCH";
        case DrivePhase::ACCELERATION: return "ACCELERATION";
        case DrivePhase::SPEED_HOLD: return "SPEED_HOLD";
        case DrivePhase::BRAKING: return "BRAKING";
        case DrivePhase::COASTING: return "COASTING";
        case DrivePhase::EMERGENCY_BRAKE: return "EMERGENCY_BRAKE";
        case DrivePhase::FINISHED: return "FINISHED";
    }

    return "UNKNOWN";
}


std::string lateralControllerToString(LateralControllerType type)
{
    switch (type)
    {
        case LateralControllerType::NONE: return "NONE";
        case LateralControllerType::LTV_MPC_UNBOUNDED:
            return "LTV_MPC_UNBOUNDED";
    }

    return "UNKNOWN";
}


std::string longitudinalControllerModeToString(LongitudinalControllerMode mode)
{
    switch (mode)
    {
        case LongitudinalControllerMode::NONE:
            return "NONE";

        case LongitudinalControllerMode::MAP_LAUNCH_ACCELERATION_BRAKE:
            return "MAP_LAUNCH_ACCELERATION_BRAKE";
    }

    return "UNKNOWN";
}


std::string emergencyBrakeModeToString(EmergencyBrakeMode mode)
{
    switch (mode)
    {
        case EmergencyBrakeMode::NONE:
            return "NONE";

        case EmergencyBrakeMode::FIXED_EMERGENCY_BRAKE:
            return "FIXED_EMERGENCY_BRAKE";
    }

    return "UNKNOWN";
}

} // namespace acc_launch_control
