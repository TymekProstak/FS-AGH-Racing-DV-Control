#include "ros_msg_adapters.hpp"

#include <cstddef>

namespace skidpad_control
{

// =============================================================================
//                              PATH MESSAGE ADAPTER
// =============================================================================

void copyPathMessageToEigen(const geometry_msgs::PoseArray& msg,
                            Eigen::VectorXd& X,
                            Eigen::VectorXd& Y)
{
    /*
        The path interface is geometry_msgs/PoseArray.
    */

    const std::size_t n = msg.poses.size();

    if (n < 2)
    {
        X.resize(0);
        Y.resize(0);
        return;
    }

    X.resize(static_cast<int>(n));
    Y.resize(static_cast<int>(n));

    for (std::size_t i = 0; i < n; ++i)
    {
        X(static_cast<int>(i)) = msg.poses[i].position.x;
        Y(static_cast<int>(i)) = msg.poses[i].position.y;
    }
}

} // namespace skidpad_control
