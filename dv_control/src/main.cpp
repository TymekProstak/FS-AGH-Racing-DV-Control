#include <ros/ros.h>
#include <ros/package.h>

#include "ParamBank.hpp"
#include "wrapper.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dv_control_node");
    ros::NodeHandle nh;

    try
    {
        std::string config_path;

        if (!nh.getParam("config_path", config_path))
        {
            config_path =
                ros::package::getPath("dv_control") +
                "/config/Params/control_param.json";
        }

        dv_control::ParamBank param(config_path);

        std::cout
            << "[INIT][OK] ParamBank loaded from: "
            << config_path
            << std::endl;

        dv_control::Controller controller(nh, param);

        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL_STREAM("[dv_control_node] Exception: " << e.what());
        return 1;
    }

    return 0;
}
