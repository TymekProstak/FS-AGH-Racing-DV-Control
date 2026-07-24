#include "control_types.hpp"   // <- podmień na nazwę swojego pliku .hpp

namespace dv_control
{

std::string phaseToString(DrivePhase phase)
{
    switch (phase)
    {
        case DrivePhase::WAITING:
            return "WAITING";

        case DrivePhase::DRIVE:
            return "DRIVE";

        case DrivePhase::EMERGENCY_BRAKE:
            return "EMERGENCY_BRAKE";

        case DrivePhase::COASTING:
            return "COASTING";

        case DrivePhase::FINISHED:
            return "FINISHED";
        case DrivePhase::FINAL_BRAKE:
            return "FINAL_BRAKE";

        default:
            return "UNKNOWN_DRIVE_PHASE";
    }
}

std::string lateralControllerToString(LateralControllerType type)
{
    switch (type)
    {
        case LateralControllerType::NONE:
            return "NONE";

        case LateralControllerType::MPC_UNBOUNDED:
            return "MPC_UNBOUNDED";

        default:
            return "UNKNOWN_LATERAL_CONTROLLER";
    }
}

std::string longitudinalControllerModeToString(LongitudinalControllerMode mode)
{
    switch (mode)
    {
        case LongitudinalControllerMode::NONE:
            return "NONE";

        case LongitudinalControllerMode::CONSTANT_SPEED:
            return "CONSTANT_SPEED";

        case LongitudinalControllerMode::VELOCITY_PROFILE:
            return "VELOCITY_PROFILE";

        default:
            return "UNKNOWN_LONGITUDINAL_CONTROLLER_MODE";
    }
}

std::string emergencyBrakeModeToString(EmergencyBrakeMode mode)
{
    switch (mode)
    {
        case EmergencyBrakeMode::NONE:
            return "NONE";

        case EmergencyBrakeMode::RAMP:
            return "RAMP";

        default:
            return "UNKNOWN_EMERGENCY_BRAKE_MODE";
    }
}

} // namespace dv_control
