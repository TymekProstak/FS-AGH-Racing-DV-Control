#pragma once

#include <Eigen/Dense>

#include <geometry_msgs/PoseArray.h>

namespace skidpad_control
{

// =============================================================================
//                              MESSAGE ADAPTERS
// =============================================================================

void copyPathMessageToEigen(const geometry_msgs::PoseArray& msg,
                            Eigen::VectorXd& X,
                            Eigen::VectorXd& Y);

} // namespace skidpad_control