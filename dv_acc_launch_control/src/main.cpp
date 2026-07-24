#include <ros/ros.h>
#include <ros/package.h>

#include <exception>
#include <string>

#include "wrapper.hpp"
#include "ParamBank.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "acc_launch_controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        std::string config_path;

        const std::string package_path =
            ros::package::getPath("dv_acc_launch_control");

        if (package_path.empty())
        {
            ROS_ERROR_STREAM(
                "[ACC Launch Main] Cannot find ROS package: dv_acc_launch_control"
            );
            return 1;
        }

        const std::string default_config_path =
            package_path + "/config/params.json";

        pnh.param<std::string>(
            "config_path",
            config_path,
            default_config_path
        );

        ROS_INFO_STREAM(
            "[ACC Launch Main] Loading config from: " << config_path
        );

        acc_launch_control::AccLaunchConfig cfg =
            acc_launch_control::loadAccLaunchConfigFromFile(config_path);

        bool print_config = false;

        pnh.param<bool>(
            "print_config",
            print_config,
            false
        );

        if (print_config)
        {
            acc_launch_control::printAccLaunchConfigSummary(cfg);
            cfg.params.printAll();
        }

        acc_launch_control::Controller controller(nh, cfg.params);

        ROS_INFO_STREAM("[ACC Launch Main] Node started.");

        /*
            Wszystkie callbacki idą teraz normalnie przez główną kolejkę ROS:
                - pathCallback
                - poseCallback
                - odometryCallback
                - dvBoardCallback
                - angleSensorCallback
                - cubeMarsStatusCallback

            Główna logika sterowania jest wykonywana w poseCallback().
        */
        ros::spin();

        ROS_INFO_STREAM("[ACC Launch Main] Node shutting down.");

        return 0;
    }
    catch (const std::exception& e)
    {
        ROS_FATAL_STREAM(
            "[ACC Launch Main] Fatal exception: " << e.what()
        );
        return 1;
    }
    catch (...)
    {
        ROS_FATAL_STREAM(
            "[ACC Launch Main] Unknown fatal exception."
        );
        return 1;
    }
}
