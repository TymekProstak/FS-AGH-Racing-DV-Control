#include <ros/ros.h>
#include <ros/package.h>
#include "wrapper.hpp"  

int main(int argc, char** argv)
{
    // Inicjalizacja węzła ROS
    ros::init(argc, argv, "dv_skidpad_control_node");
    ros::NodeHandle nh;

   	std::string pkg_path = ros::package::getPath("dv_skidpad_control");

   	// Złożenie pełnej ścieżki do pliku config.json
	std::string config_path = pkg_path + "/config/params.json";

    std::cout << "[INIT] Opening param file: " << config_path << std::endl;
    skidpad_control::ParamBank param_;
    try {
        std::ifstream f(config_path);
        if (!f.is_open()) {
            std::cerr << "[INIT][FAIL] Cannot open param file." << std::endl;
            throw std::runtime_error("Nie mogę otworzyć pliku parametrów: " + config_path);
        }

        nlohmann::json J;
        f >> J;
        
        param_ = skidpad_control::buildSkidpadParamBank(J);
        std::cout << "[INIT][OK] ParamBank built. Count=" << param_.size() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[INIT][FAIL] Parameter loading failed. what(): " << e.what() << std::endl;
        throw;
    }

	skidpad_control::Controller controller(nh, param_);
    // Standardowa pętla ROS obsługująca callbacki
    ros::spin();
    return 0;
}
