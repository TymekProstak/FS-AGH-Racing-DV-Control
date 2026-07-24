#pragma once

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace skidpad_control
{

//// ============================================================
////  ParamBank
//// ============================================================

struct ParamBank
{
    std::vector<std::string> names;
    std::unordered_map<std::string, int> idx;
    std::vector<double> values;

    int add(const std::string& name, double val)
    {
        auto it = idx.find(name);

        if (it != idx.end()) {
            values[it->second] = val;
            return it->second;
        }

        const int k = static_cast<int>(names.size());

        names.push_back(name);
        idx[name] = k;
        values.push_back(val);

        return k;
    }

    int i(const std::string& name) const
    {
        auto it = idx.find(name);

        if (it == idx.end()) {
            throw std::runtime_error("ParamBank: missing numeric key '" + name + "'");
        }

        return it->second;
    }

    double get(const std::string& name) const
    {
        return values.at(i(name));
    }

    bool getBool(const std::string& name) const
    {
        return get(name) != 0.0;
    }

    int getInt(const std::string& name) const
    {
        const double v =
            get(name);

        if (!std::isfinite(v))
        {
            throw std::runtime_error(
                "ParamBank: key '" + name + "' is not finite"
            );
        }

        const double rounded =
            std::round(v);

        if (std::abs(v - rounded) > 1.0e-9)
        {
            throw std::runtime_error(
                "ParamBank: key '" + name + "' is not integer-like"
            );
        }

        return static_cast<int>(
            std::llround(rounded)
        );
    }

    void set(const std::string& name, double v)
    {
        values.at(i(name)) = v;
    }

    std::size_t size() const
    {
        return values.size();
    }

    void printAll() const
    {
        std::cout << "==== skidpad_control::ParamBank params ====\n";

        for (std::size_t k = 0; k < names.size(); ++k) {
            std::cout << names[k] << " = " << values[k] << "\n";
        }
    }
};


//// ============================================================
////  SkidpadConfig
//// ============================================================

struct SkidpadConfig
{
    ParamBank params;
};


//// ============================================================
////  JSON helpers
//// ============================================================

inline const nlohmann::json& JgetNodeReq(const nlohmann::json& J,
                                         const std::string& path)
{
    const std::size_t pos = path.find('.');

    if (pos == std::string::npos) {
        if (!J.contains(path)) {
            throw std::runtime_error("JSON: missing required key '" + path + "'");
        }

        return J.at(path);
    }

    const std::string head = path.substr(0, pos);
    const std::string tail = path.substr(pos + 1);

    if (!J.contains(head)) {
        throw std::runtime_error("JSON: missing object '" + head + "'");
    }

    return JgetNodeReq(J.at(head), tail);
}


inline double JgetReq(const nlohmann::json& J, const std::string& path)
{
    const auto& node = JgetNodeReq(J, path);

    if (!node.is_number()) {
        throw std::runtime_error("JSON: key '" + path + "' is not a number");
    }

    return node.get<double>();
}


inline bool JgetReqBool(const nlohmann::json& J, const std::string& path)
{
    const auto& node = JgetNodeReq(J, path);

    if (node.is_boolean()) {
        return node.get<bool>();
    }

    if (node.is_number()) {
        return node.get<double>() != 0.0;
    }

    throw std::runtime_error("JSON: key '" + path + "' is not a bool or numeric bool");
}


inline double JgetReqSafe(const nlohmann::json& J, const std::string& path)
{
    try {
        return JgetReq(J, path);
    } catch (const std::exception& e) {
        std::cerr << "\n[JSON ERROR] while reading double path: " << path
                  << "\n  what(): " << e.what() << "\n" << std::endl;
        throw;
    }
}


inline bool JgetReqBoolSafe(const nlohmann::json& J, const std::string& path)
{
    try {
        return JgetReqBool(J, path);
    } catch (const std::exception& e) {
        std::cerr << "\n[JSON ERROR] while reading bool path: " << path
                  << "\n  what(): " << e.what() << "\n" << std::endl;
        throw;
    }
}


//// ============================================================
////  Add helpers
//// ============================================================

inline void addReq(ParamBank& P,
                   const nlohmann::json& J,
                   const std::string& path)
{
    P.add(path, JgetReqSafe(J, path));
}


inline void addReqBoolAsDouble(ParamBank& P,
                               const nlohmann::json& J,
                               const std::string& path)
{
    P.add(path, JgetReqBoolSafe(J, path) ? 1.0 : 0.0);
}


//// ============================================================
////  Build ParamBank from JSON
//// ============================================================

inline void flattenNumericParameters(
    ParamBank& P,
    const nlohmann::json& node,
    const std::string& prefix = {}
)
{
    if (node.is_object())
    {
        for (auto it = node.begin(); it != node.end(); ++it)
        {
            const std::string key =
                prefix.empty()
                    ? it.key()
                    : prefix + "." + it.key();

            flattenNumericParameters(P, it.value(), key);
        }

        return;
    }

    if (node.is_boolean())
    {
        P.add(prefix, node.get<bool>() ? 1.0 : 0.0);
        return;
    }

    if (node.is_number())
    {
        const double value =
            node.get<double>();

        if (!std::isfinite(value))
        {
            throw std::runtime_error(
                "JSON: non-finite numeric key '" + prefix + "'"
            );
        }

        P.add(prefix, value);
        return;
    }

    throw std::runtime_error(
        "JSON: unsupported non-numeric key '" + prefix + "'"
    );
}


inline ParamBank buildSkidpadParamBank(const nlohmann::json& J)
{
    ParamBank P;
    flattenNumericParameters(P, J);

    /*
     * Compatibility alias still consumed by the allocator. It is derived,
     * not configured as a second source of truth.
     */
    P.add(
        "model.drivetrain.max_motor_torque",
        P.get("model.drivetrain.M_wheel_drive_max_Nm")
    );

    return P;
}


//// ============================================================
////  Build full config
//// ============================================================

inline SkidpadConfig buildSkidpadConfig(const nlohmann::json& J)
{
    SkidpadConfig cfg;

    cfg.params = buildSkidpadParamBank(J);

    return cfg;
}


//// ============================================================
////  File loading
//// ============================================================

inline nlohmann::json loadJsonFile(const std::string& config_path)
{
    std::ifstream f(config_path);

    if (!f.good()) {
        throw std::runtime_error("Cannot open JSON config file: " + config_path);
    }

    nlohmann::json J;
    f >> J;

    return J;
}


inline SkidpadConfig loadSkidpadConfigFromFile(const std::string& config_path)
{
    try {
        const nlohmann::json J = loadJsonFile(config_path);
        return buildSkidpadConfig(J);
    } catch (const std::exception& e) {
        std::cerr << "\n[SKIDPAD CONFIG ERROR]"
                  << "\n  config_path: " << config_path
                  << "\n  what(): " << e.what()
                  << "\n" << std::endl;
        throw;
    }
}


inline void printSkidpadConfigSummary(const SkidpadConfig& cfg)
{
    std::cout << "==== skidpad_control::SkidpadConfig ====\n";
    std::cout << "numeric params: " << cfg.params.size() << "\n";
}

} // namespace skidpad_control
