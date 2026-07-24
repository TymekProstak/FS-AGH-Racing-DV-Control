#pragma once

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dv_control
{

class ParamBank
{
public:
    ParamBank() = default;

    explicit ParamBank(const std::string& json_path)
    {
        loadFromFile(json_path);
    }

    void loadFromFile(const std::string& json_path)
    {
        std::ifstream file(json_path);

        if (!file.is_open())
        {
            throw std::runtime_error(
                "[ParamBank] Cannot open config file: " + json_path
            );
        }

        try
        {
            file >> data_;
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error(
                "[ParamBank] Failed to parse JSON file '" +
                json_path + "': " + std::string(e.what())
            );
        }
    }

    double get(const std::string& key) const
    {
        const nlohmann::json& node =
            getNodeByDotKey(key);

        if (!node.is_number())
        {
            throw std::runtime_error(
                "[ParamBank] Parameter '" + key + "' is not numeric"
            );
        }

        const double value =
            node.get<double>();

        if (!std::isfinite(value))
        {
            throw std::runtime_error(
                "[ParamBank] Parameter '" + key + "' is not finite"
            );
        }

        return value;
    }

    bool getBool(const std::string& key) const
    {
        const nlohmann::json& node =
            getNodeByDotKey(key);

        if (node.is_boolean())
        {
            return node.get<bool>();
        }

        if (node.is_number())
        {
            const double value =
                node.get<double>();

            if (!std::isfinite(value))
            {
                throw std::runtime_error(
                    "[ParamBank] Bool-like parameter '" + key + "' is not finite"
                );
            }

            if (std::abs(value - 0.0) < 1.0e-12)
            {
                return false;
            }

            if (std::abs(value - 1.0) < 1.0e-12)
            {
                return true;
            }

            throw std::runtime_error(
                "[ParamBank] Bool-like parameter '" + key +
                "' must be true/false or 0.0/1.0"
            );
        }

        throw std::runtime_error(
            "[ParamBank] Parameter '" + key + "' is not bool-like"
        );
    }

    int getInt(const std::string& key) const
    {
        const double value =
            get(key);

        const double rounded =
            std::round(value);

        if (std::abs(value - rounded) > 1.0e-9)
        {
            throw std::runtime_error(
                "[ParamBank] Parameter '" + key + "' must be integer-like"
            );
        }

        return static_cast<int>(std::llround(rounded));
    }

    bool has(const std::string& key) const
    {
        const nlohmann::json* node =
            &data_;

        const std::vector<std::string> parts =
            splitDotKey(key);

        for (const std::string& part : parts)
        {
            if (!node->is_object())
            {
                return false;
            }

            auto it =
                node->find(part);

            if (it == node->end())
            {
                return false;
            }

            node =
                &(*it);
        }

        return true;
    }

private:
    nlohmann::json data_;

private:
    static std::vector<std::string> splitDotKey(const std::string& key)
    {
        std::vector<std::string> parts;
        std::stringstream ss(key);
        std::string part;

        while (std::getline(ss, part, '.'))
        {
            if (part.empty())
            {
                throw std::runtime_error(
                    "[ParamBank] Invalid empty token in key: " + key
                );
            }

            parts.push_back(part);
        }

        if (parts.empty())
        {
            throw std::runtime_error(
                "[ParamBank] Empty parameter key"
            );
        }

        return parts;
    }

    const nlohmann::json& getNodeByDotKey(const std::string& key) const
    {
        const nlohmann::json* node =
            &data_;

        const std::vector<std::string> parts =
            splitDotKey(key);

        std::string current_path;

        for (const std::string& part : parts)
        {
            if (!current_path.empty())
            {
                current_path += ".";
            }

            current_path += part;

            if (!node->is_object())
            {
                throw std::runtime_error(
                    "[ParamBank] Path '" + current_path +
                    "' is not an object while reading '" + key + "'"
                );
            }

            auto it =
                node->find(part);

            if (it == node->end())
            {
                throw std::runtime_error(
                    "[ParamBank] Missing required parameter: '" + key + "'"
                );
            }

            node =
                &(*it);
        }

        return *node;
    }
};

} // namespace dv_control