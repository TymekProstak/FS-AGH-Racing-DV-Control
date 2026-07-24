#pragma once

#include "ParamBank.hpp"
#include "pid.hpp"

namespace skidpad_control
{

// =============================================================================
//                              PID CONFIG
// =============================================================================

PIDParams makeSpeedPidParams(const ParamBank& P);


// =============================================================================
//                          LONGITUDINAL HELPERS
// =============================================================================

double computeResistanceFeedforwardTorqueNm(const ParamBank& P,
                                            double vx_mps);


double clampTorqueByDrivetrainLimits(const ParamBank& P,
                                     double torque_cmd_Nm,
                                     double vx_mps);


double torqueNmToSignedPercent(const ParamBank& P,
                               double torque_Nm);

} // namespace skidpad_control