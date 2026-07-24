#pragma once

#include <nlohmann/json.hpp>

#include <cstddef>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace acc_launch_control
{

// =============================================================================
//                                  PARAM BANK
// =============================================================================

struct ParamBank
{
    std::vector<std::string> names;
    std::unordered_map<std::string, int> idx;
    std::vector<double> values;

    std::vector<std::string> string_names;
    std::unordered_map<std::string, int> string_idx;
    std::vector<std::string> string_values;

    int add(const std::string& name, const double val)
    {
        const auto it = idx.find(name);

        if (it != idx.end())
        {
            values[static_cast<std::size_t>(it->second)] = val;
            return it->second;
        }

        const int k =
            static_cast<int>(names.size());

        names.push_back(name);
        idx[name] = k;
        values.push_back(val);

        return k;
    }

    int addString(const std::string& name, const std::string& val)
    {
        const auto it = string_idx.find(name);

        if (it != string_idx.end())
        {
            string_values[static_cast<std::size_t>(it->second)] = val;
            return it->second;
        }

        const int k =
            static_cast<int>(string_names.size());

        string_names.push_back(name);
        string_idx[name] = k;
        string_values.push_back(val);

        return k;
    }

    int i(const std::string& name) const
    {
        const auto it = idx.find(name);

        if (it == idx.end())
        {
            throw std::runtime_error(
                "ParamBank: missing numeric key '" + name + "'"
            );
        }

        return it->second;
    }

    int iString(const std::string& name) const
    {
        const auto it = string_idx.find(name);

        if (it == string_idx.end())
        {
            throw std::runtime_error(
                "ParamBank: missing string key '" + name + "'"
            );
        }

        return it->second;
    }

    double get(const std::string& name) const
    {
        return values.at(static_cast<std::size_t>(i(name)));
    }

    bool getBool(const std::string& name) const
    {
        return get(name) != 0.0;
    }

    int getInt(const std::string& name) const
    {
        const double value = get(name);
        const double rounded = std::round(value);

        if (!std::isfinite(value) ||
            std::abs(value - rounded) > 1.0e-9)
        {
            throw std::runtime_error(
                "ParamBank: key '" + name + "' is not an integer"
            );
        }

        return static_cast<int>(rounded);
    }

    std::string getString(const std::string& name) const
    {
        return string_values.at(static_cast<std::size_t>(iString(name)));
    }

    void set(const std::string& name, const double v)
    {
        values.at(static_cast<std::size_t>(i(name))) = v;
    }

    void setString(const std::string& name, const std::string& v)
    {
        string_values.at(static_cast<std::size_t>(iString(name))) = v;
    }

    std::size_t size() const
    {
        return values.size();
    }

    std::size_t stringSize() const
    {
        return string_values.size();
    }

    void printAll() const
    {
        std::cout << "==== acc_launch_control::ParamBank numeric params ====\n";

        for (std::size_t k = 0; k < names.size(); ++k)
        {
            std::cout << names[k] << " = " << values[k] << "\n";
        }

        std::cout << "==== acc_launch_control::ParamBank string params ====\n";

        for (std::size_t k = 0; k < string_names.size(); ++k)
        {
            std::cout << string_names[k] << " = " << string_values[k] << "\n";
        }
    }
};


// =============================================================================
//                              FULL CONFIG SUMMARY
// =============================================================================

struct AccLaunchConfig
{
    ParamBank params;

    std::string maps_root;

    std::vector<double> mu_x_cases;

    std::vector<double> S_total_cases_m;

    std::string ipopt_mu_strategy = "default";
};


// =============================================================================
//                                  JSON HELPERS
// =============================================================================

inline std::string topLevelKeysToString(const nlohmann::json& J)
{
    if (!J.is_object())
    {
        return "<root is not an object>";
    }

    std::ostringstream ss;
    bool first = true;

    for (auto it = J.begin(); it != J.end(); ++it)
    {
        if (!first)
        {
            ss << ", ";
        }

        first = false;
        ss << it.key();
    }

    return ss.str();
}

inline void requireRootObjectWithKey(
    const nlohmann::json& J,
    const std::string& key
)
{
    if (!J.is_object())
    {
        throw std::runtime_error(
            "JSON: root must be an object, but it is not"
        );
    }

    if (!J.contains(key))
    {
        throw std::runtime_error(
            "JSON: missing top-level object '" + key +
            "'. Available top-level keys: [" + topLevelKeysToString(J) + "]"
        );
    }

    if (!J.at(key).is_object())
    {
        throw std::runtime_error(
            "JSON: top-level key '" + key + "' must be an object"
        );
    }
}

inline void validateAccLaunchRootShape(const nlohmann::json& J)
{
    requireRootObjectWithKey(J, "general");
    requireRootObjectWithKey(J, "frequency");
    requireRootObjectWithKey(J, "launch_map");
    requireRootObjectWithKey(J, "steering_limit");
    requireRootObjectWithKey(J, "lateral_control");
    requireRootObjectWithKey(J, "model");
    requireRootObjectWithKey(J, "map_generator");
    requireRootObjectWithKey(J, "speed_pid");
}

inline const nlohmann::json& JgetNodeReq(
    const nlohmann::json& J,
    const std::string& path
)
{
    const std::size_t pos =
        path.find('.');

    if (pos == std::string::npos)
    {
        if (!J.is_object())
        {
            throw std::runtime_error(
                "JSON: parent is not an object while reading '" + path + "'"
            );
        }

        if (!J.contains(path))
        {
            throw std::runtime_error(
                "JSON: missing required key '" + path + "'"
            );
        }

        return J.at(path);
    }

    const std::string head =
        path.substr(0, pos);

    const std::string tail =
        path.substr(pos + 1);

    if (!J.is_object())
    {
        throw std::runtime_error(
            "JSON: parent is not an object while reading '" + path + "'"
        );
    }

    if (!J.contains(head))
    {
        throw std::runtime_error(
            "JSON: missing object '" + head + "' while reading '" + path + "'"
        );
    }

    return JgetNodeReq(
        J.at(head),
        tail
    );
}


inline double JgetReq(
    const nlohmann::json& J,
    const std::string& path
)
{
    const auto& node =
        JgetNodeReq(J, path);

    if (!node.is_number())
    {
        throw std::runtime_error(
            "JSON: key '" + path + "' is not a number"
        );
    }

    return node.get<double>();
}


inline double JgetReqBoolOrNumberAsDouble(
    const nlohmann::json& J,
    const std::string& path
)
{
    const auto& node =
        JgetNodeReq(J, path);

    if (node.is_boolean())
    {
        return node.get<bool>() ? 1.0 : 0.0;
    }

    if (node.is_number())
    {
        return node.get<double>() != 0.0 ? 1.0 : 0.0;
    }

    throw std::runtime_error(
        "JSON: key '" + path + "' is neither bool nor number"
    );
}


inline std::string JgetReqString(
    const nlohmann::json& J,
    const std::string& path
)
{
    const auto& node =
        JgetNodeReq(J, path);

    if (!node.is_string())
    {
        throw std::runtime_error(
            "JSON: key '" + path + "' is not a string"
        );
    }

    return node.get<std::string>();
}


inline std::vector<double> JgetReqDoubleArray(
    const nlohmann::json& J,
    const std::string& path
)
{
    const auto& node =
        JgetNodeReq(J, path);

    if (!node.is_array())
    {
        throw std::runtime_error(
            "JSON: key '" + path + "' is not an array"
        );
    }

    std::vector<double> out;
    out.reserve(node.size());

    for (const auto& x : node)
    {
        if (!x.is_number())
        {
            throw std::runtime_error(
                "JSON: array '" + path + "' contains non-number value"
            );
        }

        out.push_back(
            x.get<double>()
        );
    }

    return out;
}


inline double JgetReqSafe(
    const nlohmann::json& J,
    const std::string& path
)
{
    try
    {
        return JgetReq(J, path);
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "\n[JSON ERROR] while reading double path: "
            << path
            << "\n  what(): "
            << e.what()
            << "\n"
            << std::endl;

        throw;
    }
}


inline double JgetReqBoolOrNumberAsDoubleSafe(
    const nlohmann::json& J,
    const std::string& path
)
{
    try
    {
        return JgetReqBoolOrNumberAsDouble(J, path);
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "\n[JSON ERROR] while reading bool/number path: "
            << path
            << "\n  what(): "
            << e.what()
            << "\n"
            << std::endl;

        throw;
    }
}


inline std::string JgetReqStringSafe(
    const nlohmann::json& J,
    const std::string& path
)
{
    try
    {
        return JgetReqString(J, path);
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "\n[JSON ERROR] while reading string path: "
            << path
            << "\n  what(): "
            << e.what()
            << "\n"
            << std::endl;

        throw;
    }
}


inline std::vector<double> JgetReqDoubleArraySafe(
    const nlohmann::json& J,
    const std::string& path
)
{
    try
    {
        return JgetReqDoubleArray(J, path);
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "\n[JSON ERROR] while reading double array path: "
            << path
            << "\n  what(): "
            << e.what()
            << "\n"
            << std::endl;

        throw;
    }
}


// =============================================================================
//                                  ADD HELPERS
// =============================================================================

inline void addReq(
    ParamBank& P,
    const nlohmann::json& J,
    const std::string& path
)
{
    P.add(
        path,
        JgetReqSafe(J, path)
    );
}

inline void addReqBoolOrNumberAsDouble(
    ParamBank& P,
    const nlohmann::json& J,
    const std::string& path
)
{
    P.add(
        path,
        JgetReqBoolOrNumberAsDoubleSafe(J, path)
    );
}

inline void addReqString(
    ParamBank& P,
    const nlohmann::json& J,
    const std::string& path
)
{
    P.addString(
        path,
        JgetReqStringSafe(J, path)
    );
}

inline void addMappedReq(
    ParamBank& P,
    const nlohmann::json& J,
    const std::string& param_key_used_by_code,
    const std::string& exact_json_path
)
{
    /*
        Strict deterministic mapping for this exact params.json.

        - exact_json_path is the real key in the JSON file.
        - param_key_used_by_code is the key that existing controller classes
          already call with P.get(...).

        There is no fallback and no probing of alternative paths.
    */
    P.add(
        param_key_used_by_code,
        JgetReqSafe(J, exact_json_path)
    );
}


// =============================================================================
//                          BUILD PARAM BANK FROM CURRENT params.json
// =============================================================================

inline ParamBank buildAccLaunchParamBank(
    const nlohmann::json& J
)
{
    validateAccLaunchRootShape(J);

    ParamBank P;

    // -------------------------------------------------------------------------
    // General
    // -------------------------------------------------------------------------

    addReq(P, J, "general.s_total");
    addReq(P, J, "general.vx_max");
    addReq(P, J, "general.mu_x");
    addReq(P, J, "general.mu_y");
    addReq(P, J, "general.finished_encoder_speed_mps");
    addReq(P, J, "general.coast_below_speed_mps");
    addReq(P, J, "general.brake_margin_m");

    addReqBoolOrNumberAsDouble(P, J, "general.use_emergency_check");
    addReqBoolOrNumberAsDouble(P, J, "general.use_cube_mars_encoder_check");
    addReqBoolOrNumberAsDouble(P, J, "general.use_cube_mars_following_check");
    addReqBoolOrNumberAsDouble(P, J, "general.use_path_planner_check");
    addReqBoolOrNumberAsDouble(P, J, "general.use_dynamic_state_check");
    addReqBoolOrNumberAsDouble(P, J, "general.use_ins_pose_check");
    addReqBoolOrNumberAsDouble(P, J, "general.use_ins_stability_check");
    addReqBoolOrNumberAsDouble(P, J, "general.use_ins_sliding_velocity_check");
    addReqBoolOrNumberAsDouble(P, J, "general.print_console_debug_info");

    // -------------------------------------------------------------------------
    // Frequencies
    // -------------------------------------------------------------------------

    addReq(P, J, "frequency.steer_cmd_loop_hz");

    // -------------------------------------------------------------------------
    // Launch map
    // -------------------------------------------------------------------------

    addReqString(P, J, "launch_map.maps_root");
    addReq(P, J, "launch_map.vx_lookup_grid_step_mps");
    addReq(P, J, "launch_map.control_loop_hz");

    // -------------------------------------------------------------------------
    // Steering and the single lateral controller
    // -------------------------------------------------------------------------

    addReq(P, J, "steering_limit.max_steer");
    addReq(P, J, "steering_limit.max_steer_rate");
    addReq(P, J, "lateral_control.activation_speed_mps");

    addReq(P, J, "model.steering_system.steer_natural_freq");
    addReq(P, J, "model.steering_system.steer_damping");

    addReq(P, J, "model.ltv_mpc_unbounded.N");
    addReq(P, J, "model.ltv_mpc_unbounded.Q_psi");
    addReq(P, J, "model.ltv_mpc_unbounded.Q_ey");
    addReq(P, J, "model.ltv_mpc_unbounded.Q_vy");
    addReq(P, J, "model.ltv_mpc_unbounded.Q_r");
    addReq(P, J, "model.ltv_mpc_unbounded.R_d_delta");
    addReq(P, J, "model.ltv_mpc_unbounded.R_tv_yaw_moment");
    addReq(P, J, "model.ltv_mpc_unbounded.R_d_tv_yaw_moment");
    addReq(P, J, "model.ltv_mpc_unbounded.v_min");
    addReq(P, J, "model.ltv_mpc_unbounded.hessian_regularization");
    addReq(P, J, "model.ltv_mpc_unbounded.terminal_scale");

    // -------------------------------------------------------------------------
    // Model - body
    // -------------------------------------------------------------------------

    addReq(P, J, "model.body.m");
    addReq(P, J, "model.body.g");
    addReq(P, J, "model.body.Iz");
    addReq(P, J, "model.body.l_f");
    addReq(P, J, "model.body.l_r");
    addReq(P, J, "model.body.h_cg");
    addReq(P, J, "model.body.h1_roll");
    addReq(P, J, "model.body.h2_roll");
    addReq(P, J, "model.body.lambda_phi_elastic_lateral");

    addReq(P, J, "model.body.C_l_f");
    addReq(P, J, "model.body.C_l_r");

    addReq(P, J, "model.body.Cf");
    addReq(P, J, "model.body.Df");
    addReq(P, J, "model.body.Bf");

    addReq(P, J, "model.body.Cr");
    addReq(P, J, "model.body.Dr");
    addReq(P, J, "model.body.Br");

    addReqBoolOrNumberAsDouble(P, J, "model.body.use_aero");

    addReq(P, J, "model.body.Cd");
    addReq(P, J, "model.body.rolling_resistance_coeff");
    addReq(P, J, "model.body.resistance_constant_N");
    addReq(P, J, "model.body.resistance_linear_N_per_mps");
    addReq(P, J, "model.body.track_width");
    addReq(P, J, "model.body.Cl1");
    addReq(P, J, "model.body.Cl2");

    // -------------------------------------------------------------------------
    // Model - tire
    // -------------------------------------------------------------------------

    addReq(P, J, "model.tire.R_tire");
    addReq(P, J, "model.tire.I_wheel");
    addReq(P, J, "model.tire.Ckappa_tanh");
    addReq(P, J, "model.tire.v_threshold");
    addReq(P, J, "model.tire.relaxation_length_long");
    addReq(P, J, "model.tire.kappa_limit");

    // -------------------------------------------------------------------------
    // Model - drivetrain
    // -------------------------------------------------------------------------

    addReq(P, J, "model.drivetrain.M_wheel_drive_max_Nm");
    addReq(P, J, "model.drivetrain.P_wheel_drive_max_W");
    addReqBoolOrNumberAsDouble(P, J, "model.drivetrain.use_drive_power_limit");

    addReq(P, J, "model.drivetrain.M_wheel_brake_max_Nm");
    addReq(P, J, "model.drivetrain.P_wheel_brake_max_W");
    addReqBoolOrNumberAsDouble(P, J, "model.drivetrain.use_brake_power_limit");

    addReq(P, J, "model.drivetrain.gear_ratio");
    addReq(P, J, "model.drivetrain.motor_torque_time_constant");

    addReq(P, J, "model.drivetrain.M_dot_up");
    addReq(P, J, "model.drivetrain.M_dot_down");

    addReq(P, J, "model.drivetrain.M_engine_max_Nm_usage_drive_total");
    addReq(P, J, "model.drivetrain.M_engine_max_Nm_usage_brake_total");

    addReq(P, J, "model.drivetrain.wheel_max_torque_engine_drive_Nm");
    addReq(P, J, "model.drivetrain.wheel_max_torque_engine_brake_Nm");

    // -------------------------------------------------------------------------
    // Model - mass transfer
    // -------------------------------------------------------------------------

    addReqBoolOrNumberAsDouble(P, J, "model.mass_transfer.enabled");
    addReq(P, J, "model.mass_transfer.anti_dive");
    addReq(P, J, "model.mass_transfer.anti_squat");
    addReq(P, J, "model.mass_transfer.tau_load_s");
    addReq(P, J, "model.mass_transfer.minimum_wheel_load_N");
    addReq(P, J, "model.mass_transfer.Fz_min_N");
    addReq(P, J, "model.mass_transfer.Fz_max_N");

    // -------------------------------------------------------------------------
    // Map generator
    // -------------------------------------------------------------------------

    addReq(P, J, "map_generator.S");
    addReq(P, J, "map_generator.dt_control");
    addReq(P, J, "map_generator.dt_dynamics");

    addReq(P, J, "map_generator.M_cmd_min");
    addReq(P, J, "map_generator.M_cmd_max");
    addReq(P, J, "map_generator.M_cmd_prev0");
    addReq(P, J, "map_generator.M_cmd_dot_up_max");
    addReq(P, J, "map_generator.M_cmd_dot_down_max");

    addReq(P, J, "map_generator.weights.distance");
    addReq(P, J, "map_generator.weights.slip_slack");
    addReq(P, J, "map_generator.weights.torque_rate");

    addReq(P, J, "map_generator.solver.max_iter");
    addReq(P, J, "map_generator.solver.tol");
    addReq(P, J, "map_generator.solver.acceptable_tol");
    addReq(P, J, "map_generator.solver.acceptable_iter");

    // -------------------------------------------------------------------------
    // Speed hold PID
    // -------------------------------------------------------------------------

    addReq(P, J, "speed_pid.Kp");
    addReq(P, J, "speed_pid.Ki");
    addReq(P, J, "speed_pid.Kd");
    addReq(P, J, "speed_pid.saturation_lower_Nm");
    addReq(P, J, "speed_pid.saturation_upper_Nm");
    addReq(P, J, "speed_pid.anti_windup_gain");
    addReq(P, J, "speed_pid.leak_time_scale");
    addReqBoolOrNumberAsDouble(P, J, "speed_pid.use_output_rate_limit");
    addReq(P, J, "speed_pid.output_rate_up");
    addReq(P, J, "speed_pid.output_rate_down");

    // Aliases required by the shared dv_control LTV MPC implementation.
    P.add(
        "model.frequency.steer_cmd_loop_hz",
        P.get("frequency.steer_cmd_loop_hz")
    );
    P.add(
        "model.steering_limit.max_steer",
        P.get("steering_limit.max_steer")
    );
    P.add(
        "model.steering_limit.max_steer_rate",
        P.get("steering_limit.max_steer_rate")
    );
    P.add("general.general_use_jaca_torque_vectoring", 0.0);
    P.add("general.general_use_torque_vectoring", 0.0);

    return P;
}


// =============================================================================
//                              BUILD FULL CONFIG
// =============================================================================

inline AccLaunchConfig buildAccLaunchConfig(
    const nlohmann::json& J
)
{
    validateAccLaunchRootShape(J);

    AccLaunchConfig cfg;

    cfg.params =
        buildAccLaunchParamBank(J);

    cfg.maps_root =
        JgetReqStringSafe(
            J,
            "launch_map.maps_root"
        );

    cfg.mu_x_cases =
        JgetReqDoubleArraySafe(
            J,
            "map_generator.mu_x_cases"
        );

    cfg.S_total_cases_m =
        {
            JgetReqSafe(
                J,
                "map_generator.S"
            )
        };

    return cfg;
}


// =============================================================================
//                                  FILE LOADING
// =============================================================================

inline nlohmann::json loadJsonFile(
    const std::string& config_path
)
{
    std::ifstream f(config_path);

    if (!f.good())
    {
        throw std::runtime_error(
            "Cannot open JSON config file: " + config_path
        );
    }

    nlohmann::json J;

    try
    {
        f >> J;
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(
            "Failed to parse JSON config file: " + config_path +
            "; what(): " + std::string(e.what())
        );
    }

    return J;
}


inline AccLaunchConfig loadAccLaunchConfigFromFile(
    const std::string& config_path
)
{
    try
    {
        const nlohmann::json J =
            loadJsonFile(config_path);

        return buildAccLaunchConfig(J);
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "\n[ACC LAUNCH CONFIG ERROR]"
            << "\n  config_path: "
            << config_path
            << "\n  what(): "
            << e.what()
            << "\n"
            << std::endl;

        throw;
    }
}


inline void printAccLaunchConfigSummary(
    const AccLaunchConfig& cfg
)
{
    std::cout << "==== acc_launch_control::AccLaunchConfig ====\n";

    std::cout << "maps_root:               " << cfg.maps_root << "\n";
    std::cout << "ipopt_mu_strategy:       " << cfg.ipopt_mu_strategy << "\n";

    std::cout << "mu_x_cases:              ";
    for (const double x : cfg.mu_x_cases)
    {
        std::cout << x << " ";
    }
    std::cout << "\n";

    std::cout << "S_total_cases_m:         ";
    for (const double x : cfg.S_total_cases_m)
    {
        std::cout << x << " ";
    }
    std::cout << "\n";

    std::cout << "numeric params:          " << cfg.params.size() << "\n";
    std::cout << "string params:           " << cfg.params.stringSize() << "\n";
}

} // namespace acc_launch_control
