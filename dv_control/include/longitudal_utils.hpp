#pragma once

#include "ParamBank.hpp"
#include "pid.hpp"
#include "math_utilis.hpp"

namespace dv_control
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


double computeAccelerationFeedforwardTorqueNm(const ParamBank& P,
                                            double ax_mps2,
                                            double vx_mps);

} // namespace dv_control