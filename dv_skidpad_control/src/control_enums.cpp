#include "control_enums.hpp"

namespace skidpad_control
{

std::string phaseToString(DrivePhase phase)
{
    switch (phase)
    {
        case DrivePhase::WAITING:
            return "waiting";

        case DrivePhase::DRIVE:
            return "drive";

        case DrivePhase::BRAKING:
            return "braking";

        case DrivePhase::COASTING:
            return "coasting";

        case DrivePhase::FINISHED:
            return "finished";

        case DrivePhase::EMERGENCY_STOP:
            return "emergency_stop";

        case DrivePhase::EMERGENCY_STOP_HARD:
            return "emergency_stop_hard";
    }

    return "unknown";
}

} // namespace skidpad_control
