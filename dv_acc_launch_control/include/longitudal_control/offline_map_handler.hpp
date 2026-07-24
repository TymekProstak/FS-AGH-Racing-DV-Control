#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace acc_runtime_maps
{

// =============================================================================
//                              PUBLIC TYPES
// =============================================================================

struct LaunchPhaseEndInfo
{
    bool valid = false;

    double t_s = 0.0;
    double s_m = 0.0;
    double vx_mps = 0.0;
    double ax_mps2 = 0.0;

    std::string reason;
};

struct LaunchRuntimeCommand
{
    bool valid = false;

    // Sample point.
    double t_phase_s = 0.0;
    double t_global_s = 0.0;
    double s_global_m = 0.0;
    double vx_mps = 0.0;
    double ax_model_mps2 = 0.0;

    // Wheel speeds / slip references.
    double omega_front_radps = 0.0;
    double omega_rear_radps = 0.0;

    double v_front_wheel_mps = 0.0;
    double v_rear_wheel_mps = 0.0;

    double v_slip_front_mps = 0.0;
    double v_slip_rear_mps = 0.0;

    double kappa_front = 0.0;
    double kappa_rear = 0.0;

    double slack_front_mps = 0.0;
    double slack_rear_mps = 0.0;

    // Main runtime command, all in wheel-side Nm.
    double M_total_cmd_Nm = 0.0;
    double M_front_total_cmd_Nm = 0.0;
    double M_rear_total_cmd_Nm = 0.0;

    double M_front_per_wheel_cmd_Nm = 0.0;
    double M_rear_per_wheel_cmd_Nm = 0.0;

    double M_total_act_Nm = 0.0;
    double M_front_total_act_Nm = 0.0;
    double M_rear_total_act_Nm = 0.0;

    // Forces / loads.
    double Fx_front_total_N = 0.0;
    double Fx_rear_total_N = 0.0;
    double Fx_total_N = 0.0;
    double F_resistance_N = 0.0;

    double Fz_front_total_N = 0.0;
    double Fz_rear_total_N = 0.0;

    double Fz_FL_N = 0.0;
    double Fz_FR_N = 0.0;
    double Fz_RL_N = 0.0;
    double Fz_RR_N = 0.0;

    // Power debug.
    double P_front_cmd_W = 0.0;
    double P_rear_cmd_W = 0.0;
    double P_total_cmd_W = 0.0;
    double P_total_cmd_kW = 0.0;

    double P_front_act_W = 0.0;
    double P_rear_act_W = 0.0;
    double P_total_act_W = 0.0;
    double P_total_act_kW = 0.0;

    double P_total_drive_limit_W = 0.0;
    double M_total_limit_Nm = 0.0;
    double M_axle_limit_Nm = 0.0;

    // Flags from CSV.
    bool power_limited_cmd = false;
    bool torque_limited_front = false;
    bool torque_limited_rear = false;

    bool initial_phase_end_flag = false;
    std::string initial_phase_region;

    LaunchPhaseEndInfo initial_phase_end;
};


// =============================================================================
//                         LAUNCH RUNTIME MAP HANDLER
// =============================================================================

class LaunchRuntimeMapHandler
{
public:
    LaunchRuntimeMapHandler(const std::string& maps_root_or_launch_dir,
                            double mu,
                            double s_total_m)
        : maps_root_or_launch_dir_(maps_root_or_launch_dir)
        , requested_mu_(mu)
        , requested_s_total_m_(s_total_m)
    {
        loadExactLaunchMap();
    }

    bool isValid() const
    {
        return loaded_;
    }

    const std::string& loadedPath() const
    {
        return loaded_path_;
    }

    double getMu() const
    {
        return loaded_ ? mu_x_ : 0.0;
    }

    double getSTotal() const
    {
        return loaded_ ? S_total_m_ : 0.0;
    }

    double getLaunchDuration() const
    {
        if (!loaded_ || rows_by_time_.empty())
        {
            return 0.0;
        }

        return rows_by_time_.back().t_phase_s;
    }

    double getLaunchEndDistance() const
    {
        if (!loaded_ || rows_by_distance_.empty())
        {
            return 0.0;
        }

        return rows_by_distance_.back().s_global_m;
    }

    double getLaunchEndSpeed() const
    {
        if (!loaded_ || rows_by_speed_.empty())
        {
            return 0.0;
        }

        return rows_by_speed_.back().vx_mps;
    }

    LaunchPhaseEndInfo getInitialLaunchEndInfo() const
    {
        return initial_phase_end_;
    }

    LaunchRuntimeCommand getLaunchCommandByTime(double t_phase_s) const
    {
        return sampleRows(rows_by_time_, Domain::Time, t_phase_s);
    }

    LaunchRuntimeCommand getLaunchCommandByDistance(double s_global_m) const
    {
        return sampleRows(rows_by_distance_, Domain::Distance, s_global_m);
    }

    LaunchRuntimeCommand getLaunchCommandBySpeed(double vx_mps) const
    {
        return sampleRows(rows_by_speed_, Domain::Speed, vx_mps);
    }

    LaunchRuntimeCommand getLaunchCommand(const std::string& mode,
                                          double value) const
    {
        if (mode == "time" || mode == "t" || mode == "TIME")
        {
            return getLaunchCommandByTime(value);
        }

        if (mode == "distance" || mode == "s" || mode == "DISTANCE")
        {
            return getLaunchCommandByDistance(value);
        }

        if (mode == "speed" || mode == "vx" || mode == "SPEED")
        {
            return getLaunchCommandBySpeed(value);
        }

        std::cerr
            << "[LaunchRuntimeMapHandler] ERROR: unknown sampling mode='"
            << mode
            << "'. Expected: time/t, distance/s, speed/vx. Returning zero.\n";

        return LaunchRuntimeCommand{};
    }

private:
    enum class Domain
    {
        Time,
        Distance,
        Speed
    };

    struct RuntimeRow
    {
        double t_phase_s = 0.0;
        double t_global_s = 0.0;
        double s_global_m = 0.0;
        double vx_mps = 0.0;
        double ax_model_mps2 = 0.0;

        double omega_front_radps = 0.0;
        double omega_rear_radps = 0.0;

        double v_front_wheel_mps = 0.0;
        double v_rear_wheel_mps = 0.0;

        double v_slip_front_mps = 0.0;
        double v_slip_rear_mps = 0.0;

        double kappa_front = 0.0;
        double kappa_rear = 0.0;

        double slack_front_mps = 0.0;
        double slack_rear_mps = 0.0;

        double M_total_cmd_Nm = 0.0;
        double M_front_total_cmd_Nm = 0.0;
        double M_rear_total_cmd_Nm = 0.0;
        double M_front_per_wheel_cmd_Nm = 0.0;
        double M_rear_per_wheel_cmd_Nm = 0.0;

        double M_total_act_Nm = 0.0;
        double M_front_total_act_Nm = 0.0;
        double M_rear_total_act_Nm = 0.0;

        double Fx_front_total_N = 0.0;
        double Fx_rear_total_N = 0.0;
        double Fx_total_N = 0.0;
        double F_resistance_N = 0.0;

        double Fz_front_total_N = 0.0;
        double Fz_rear_total_N = 0.0;
        double Fz_FL_N = 0.0;
        double Fz_FR_N = 0.0;
        double Fz_RL_N = 0.0;
        double Fz_RR_N = 0.0;

        double P_front_cmd_W = 0.0;
        double P_rear_cmd_W = 0.0;
        double P_total_cmd_W = 0.0;
        double P_total_cmd_kW = 0.0;

        double P_front_act_W = 0.0;
        double P_rear_act_W = 0.0;
        double P_total_act_W = 0.0;
        double P_total_act_kW = 0.0;

        double P_total_drive_limit_W = 0.0;
        double M_total_limit_Nm = 0.0;
        double M_axle_limit_Nm = 0.0;

        double power_limited_cmd = 0.0;
        double torque_limited_front = 0.0;
        double torque_limited_rear = 0.0;

        double initial_phase_end_flag = 0.0;
        std::string initial_phase_region;

        double initial_phase_end_t_s = 0.0;
        double initial_phase_end_s_m = 0.0;
        double initial_phase_end_vx_mps = 0.0;
        double initial_phase_end_ax_mps2 = 0.0;
        std::string initial_phase_end_reason;
    };

private:
    std::string maps_root_or_launch_dir_;

    double requested_mu_ = 0.0;
    double requested_s_total_m_ = 0.0;

    bool loaded_ = false;
    mutable bool printed_not_loaded_error_ = false;

    std::string loaded_path_;

    double mu_x_ = 0.0;
    double S_total_m_ = 0.0;

    std::vector<RuntimeRow> rows_by_time_;
    std::vector<RuntimeRow> rows_by_distance_;
    std::vector<RuntimeRow> rows_by_speed_;

    LaunchPhaseEndInfo initial_phase_end_;

private:
    static double clamp01(double x)
    {
        if (x < 0.0)
        {
            return 0.0;
        }

        if (x > 1.0)
        {
            return 1.0;
        }

        return x;
    }

    static double lerp(double a, double b, double alpha)
    {
        return a + alpha * (b - a);
    }

    static std::string replaceChar(std::string s, char from, char to)
    {
        std::replace(s.begin(), s.end(), from, to);
        return s;
    }

    static std::string makeFloatTag(double value, const std::string& prefix)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << value;

        std::string value_str = ss.str();
        value_str = replaceChar(value_str, '.', 'p');
        value_str = replaceChar(value_str, '-', 'm');

        return prefix + "_" + value_str;
    }

    static std::string joinPath(const std::string& a, const std::string& b)
    {
        if (a.empty())
        {
            return b;
        }

        if (a.back() == '/' || a.back() == '\\')
        {
            return a + b;
        }

        return a + "/" + b;
    }

    std::string expectedFileName() const
    {
        return
            "launch_"
            + makeFloatTag(requested_mu_, "mu")
            + "_"
            + makeFloatTag(requested_s_total_m_, "S")
            + ".csv";
    }

    std::string resolveExactRuntimePath() const
    {
        const std::string filename = expectedFileName();

        const std::string as_root = joinPath(
            joinPath(maps_root_or_launch_dir_, "launch_runtime_profiles"),
            filename
        );

        if (fileExists(as_root))
        {
            return as_root;
        }

        const std::string as_launch_dir = joinPath(
            maps_root_or_launch_dir_,
            filename
        );

        if (fileExists(as_launch_dir))
        {
            return as_launch_dir;
        }

        return "";
    }

    static bool fileExists(const std::string& path)
    {
        std::ifstream f(path.c_str());
        return f.good();
    }

    static std::vector<std::string> splitCsvLine(const std::string& line)
    {
        std::vector<std::string> out;
        std::string current;
        bool in_quotes = false;

        for (std::size_t i = 0; i < line.size(); ++i)
        {
            const char c = line[i];

            if (c == '"')
            {
                in_quotes = !in_quotes;
                continue;
            }

            if (c == ',' && !in_quotes)
            {
                out.push_back(current);
                current.clear();
                continue;
            }

            current.push_back(c);
        }

        out.push_back(current);
        return out;
    }

    static std::unordered_map<std::string, std::size_t>
    makeHeaderIndex(const std::vector<std::string>& header)
    {
        std::unordered_map<std::string, std::size_t> idx;

        for (std::size_t i = 0; i < header.size(); ++i)
        {
            idx[header[i]] = i;
        }

        return idx;
    }

    static std::string getString(const std::vector<std::string>& cols,
                                 const std::unordered_map<std::string, std::size_t>& idx,
                                 const std::string& name,
                                 const std::string& default_value = "")
    {
        const auto it = idx.find(name);

        if (it == idx.end())
        {
            return default_value;
        }

        if (it->second >= cols.size())
        {
            return default_value;
        }

        return cols[it->second];
    }

    static double getDouble(const std::vector<std::string>& cols,
                            const std::unordered_map<std::string, std::size_t>& idx,
                            const std::string& name,
                            double default_value = 0.0)
    {
        const std::string s = getString(cols, idx, name, "");

        if (s.empty())
        {
            return default_value;
        }

        try
        {
            return std::stod(s);
        }
        catch (...)
        {
            return default_value;
        }
    }

    void printMapNotLoadedErrorOnce() const
    {
        if (printed_not_loaded_error_)
        {
            return;
        }

        std::cerr
            << "[LaunchRuntimeMapHandler] ERROR: launch runtime map is not loaded. "
            << "Returning zero command.\n";

        printed_not_loaded_error_ = true;
    }

    void loadExactLaunchMap()
    {
        const std::string path = resolveExactRuntimePath();

        if (path.empty())
        {
            std::cerr
                << "[LaunchRuntimeMapHandler] ERROR: exact launch runtime map was not found for "
                << "mu=" << requested_mu_
                << ", S_total=" << requested_s_total_m_
                << " m. Expected file: "
                << expectedFileName()
                << " in '"
                << maps_root_or_launch_dir_
                << "' or in '"
                << joinPath(maps_root_or_launch_dir_, "launch_runtime_profiles")
                << "'. Handler will return zeros.\n";

            loaded_ = false;
            return;
        }

        std::ifstream file(path.c_str());

        if (!file.good())
        {
            std::cerr
                << "[LaunchRuntimeMapHandler] ERROR: cannot open launch runtime map: "
                << path
                << ". Handler will return zeros.\n";

            loaded_ = false;
            return;
        }

        std::string header_line;

        if (!std::getline(file, header_line))
        {
            std::cerr
                << "[LaunchRuntimeMapHandler] ERROR: empty CSV: "
                << path
                << ". Handler will return zeros.\n";

            loaded_ = false;
            return;
        }

        const auto header = splitCsvLine(header_line);
        const auto idx = makeHeaderIndex(header);

        std::string line;
        bool first_data_row = true;

        while (std::getline(file, line))
        {
            if (line.empty())
            {
                continue;
            }

            const auto cols = splitCsvLine(line);
            const std::string phase = getString(cols, idx, "phase", "");

            if (phase != "launch")
            {
                continue;
            }

            RuntimeRow row;

            row.t_phase_s = getDouble(cols, idx, "t_phase_s");
            row.t_global_s = getDouble(cols, idx, "t_global_s");
            row.s_global_m = getDouble(cols, idx, "s_global_m");
            row.vx_mps = getDouble(cols, idx, "vx_mps");
            row.ax_model_mps2 = getDouble(cols, idx, "ax_model_mps2");

            row.omega_front_radps = getDouble(cols, idx, "omega_front_radps");
            row.omega_rear_radps = getDouble(cols, idx, "omega_rear_radps");

            row.v_front_wheel_mps = getDouble(cols, idx, "v_front_wheel_mps");
            row.v_rear_wheel_mps = getDouble(cols, idx, "v_rear_wheel_mps");

            row.v_slip_front_mps = getDouble(cols, idx, "v_slip_front_mps");
            row.v_slip_rear_mps = getDouble(cols, idx, "v_slip_rear_mps");

            row.kappa_front = getDouble(cols, idx, "kappa_front");
            row.kappa_rear = getDouble(cols, idx, "kappa_rear");

            row.slack_front_mps = getDouble(cols, idx, "slack_front_mps");
            row.slack_rear_mps = getDouble(cols, idx, "slack_rear_mps");

            row.M_total_cmd_Nm = getDouble(cols, idx, "M_total_cmd_Nm");
            row.M_front_total_cmd_Nm = getDouble(cols, idx, "M_front_total_cmd_Nm");
            row.M_rear_total_cmd_Nm = getDouble(cols, idx, "M_rear_total_cmd_Nm");
            row.M_front_per_wheel_cmd_Nm = getDouble(cols, idx, "M_front_per_wheel_cmd_Nm");
            row.M_rear_per_wheel_cmd_Nm = getDouble(cols, idx, "M_rear_per_wheel_cmd_Nm");

            row.M_total_act_Nm = getDouble(cols, idx, "M_total_act_Nm");
            row.M_front_total_act_Nm = getDouble(cols, idx, "M_front_total_act_Nm");
            row.M_rear_total_act_Nm = getDouble(cols, idx, "M_rear_total_act_Nm");

            row.Fx_front_total_N = getDouble(cols, idx, "Fx_front_total_N");
            row.Fx_rear_total_N = getDouble(cols, idx, "Fx_rear_total_N");
            row.Fx_total_N = getDouble(cols, idx, "Fx_total_N");
            row.F_resistance_N = getDouble(cols, idx, "F_resistance_N");

            row.Fz_front_total_N = getDouble(cols, idx, "Fz_front_total_N");
            row.Fz_rear_total_N = getDouble(cols, idx, "Fz_rear_total_N");
            row.Fz_FL_N = getDouble(cols, idx, "Fz_FL_N");
            row.Fz_FR_N = getDouble(cols, idx, "Fz_FR_N");
            row.Fz_RL_N = getDouble(cols, idx, "Fz_RL_N");
            row.Fz_RR_N = getDouble(cols, idx, "Fz_RR_N");

            row.P_front_cmd_W = getDouble(cols, idx, "P_front_cmd_W");
            row.P_rear_cmd_W = getDouble(cols, idx, "P_rear_cmd_W");
            row.P_total_cmd_W = getDouble(cols, idx, "P_total_cmd_W");
            row.P_total_cmd_kW = getDouble(cols, idx, "P_total_cmd_kW");

            row.P_front_act_W = getDouble(cols, idx, "P_front_act_W");
            row.P_rear_act_W = getDouble(cols, idx, "P_rear_act_W");
            row.P_total_act_W = getDouble(cols, idx, "P_total_act_W");
            row.P_total_act_kW = getDouble(cols, idx, "P_total_act_kW");

            row.P_total_drive_limit_W = getDouble(cols, idx, "P_total_drive_limit_W");
            row.M_total_limit_Nm = getDouble(cols, idx, "M_total_limit_Nm");
            row.M_axle_limit_Nm = getDouble(cols, idx, "M_axle_limit_Nm");

            row.power_limited_cmd = getDouble(cols, idx, "power_limited_cmd");
            row.torque_limited_front = getDouble(cols, idx, "torque_limited_front");
            row.torque_limited_rear = getDouble(cols, idx, "torque_limited_rear");

            row.initial_phase_end_flag = getDouble(cols, idx, "initial_phase_end_flag");
            row.initial_phase_region = getString(cols, idx, "initial_phase_region", "");
            row.initial_phase_end_t_s = getDouble(cols, idx, "initial_phase_end_t_s");
            row.initial_phase_end_s_m = getDouble(cols, idx, "initial_phase_end_s_m");
            row.initial_phase_end_vx_mps = getDouble(cols, idx, "initial_phase_end_vx_mps");
            row.initial_phase_end_ax_mps2 = getDouble(cols, idx, "initial_phase_end_ax_mps2");
            row.initial_phase_end_reason = getString(cols, idx, "initial_phase_end_reason", "");

            if (first_data_row)
            {
                mu_x_ = getDouble(cols, idx, "mu_x");
                S_total_m_ = getDouble(cols, idx, "S_total_m");

                initial_phase_end_.valid = true;
                initial_phase_end_.t_s = row.initial_phase_end_t_s;
                initial_phase_end_.s_m = row.initial_phase_end_s_m;
                initial_phase_end_.vx_mps = row.initial_phase_end_vx_mps;
                initial_phase_end_.ax_mps2 = row.initial_phase_end_ax_mps2;
                initial_phase_end_.reason = row.initial_phase_end_reason;

                first_data_row = false;
            }

            rows_by_time_.push_back(row);
        }

        if (rows_by_time_.empty())
        {
            std::cerr
                << "[LaunchRuntimeMapHandler] ERROR: CSV has no launch rows: "
                << path
                << ". Handler will return zeros.\n";

            loaded_ = false;
            return;
        }

        auto by_time = [](const RuntimeRow& a, const RuntimeRow& b)
        {
            return a.t_phase_s < b.t_phase_s;
        };

        std::sort(rows_by_time_.begin(), rows_by_time_.end(), by_time);

        rows_by_distance_ = rows_by_time_;
        rows_by_speed_ = rows_by_time_;

        std::sort(rows_by_distance_.begin(), rows_by_distance_.end(),
                  [](const RuntimeRow& a, const RuntimeRow& b)
                  {
                      return a.s_global_m < b.s_global_m;
                  });

        std::sort(rows_by_speed_.begin(), rows_by_speed_.end(),
                  [](const RuntimeRow& a, const RuntimeRow& b)
                  {
                      return a.vx_mps < b.vx_mps;
                  });

        loaded_path_ = path;
        loaded_ = true;

        std::cout
            << "[LaunchRuntimeMapHandler] Loaded launch map: "
            << loaded_path_
            << " | mu=" << mu_x_
            << " | S_total=" << S_total_m_
            << " | rows=" << rows_by_time_.size()
            << " | initial_end_t=" << initial_phase_end_.t_s
            << " | initial_end_s=" << initial_phase_end_.s_m
            << " | initial_end_vx=" << initial_phase_end_.vx_mps
            << " | reason=" << initial_phase_end_.reason
            << "\n";
    }

    static double rowDomainValue(const RuntimeRow& r, Domain domain)
    {
        if (domain == Domain::Time)
        {
            return r.t_phase_s;
        }

        if (domain == Domain::Distance)
        {
            return r.s_global_m;
        }

        return r.vx_mps;
    }

    LaunchRuntimeCommand sampleRows(const std::vector<RuntimeRow>& rows,
                                    Domain domain,
                                    double value) const
    {
        if (!loaded_)
        {
            printMapNotLoadedErrorOnce();
            return LaunchRuntimeCommand{};
        }

        if (rows.empty())
        {
            return LaunchRuntimeCommand{};
        }

        constexpr double eps = 1.0e-9;

        const double x_min = rowDomainValue(rows.front(), domain);
        const double x_max = rowDomainValue(rows.back(), domain);

        if (value < x_min - eps)
        {
            return LaunchRuntimeCommand{};
        }

        if (value <= x_min)
        {
            return toCommand(rows.front(), true);
        }

        if (value >= x_max)
        {
            return toCommand(rows.back(), true);
        }

        auto it = std::lower_bound(
            rows.begin(),
            rows.end(),
            value,
            [domain](const RuntimeRow& r, double v)
            {
                return rowDomainValue(r, domain) < v;
            }
        );

        if (it == rows.begin())
        {
            return toCommand(rows.front(), true);
        }

        if (it == rows.end())
        {
            return toCommand(rows.back(), true);
        }

        const RuntimeRow& b = *it;
        const RuntimeRow& a = *(it - 1);

        const double xa = rowDomainValue(a, domain);
        const double xb = rowDomainValue(b, domain);

        const double alpha = std::abs(xb - xa) > 1.0e-12
            ? clamp01((value - xa) / (xb - xa))
            : 0.0;

        return toCommand(interpolateRow(a, b, alpha), true);
    }

    static RuntimeRow interpolateRow(const RuntimeRow& a,
                                     const RuntimeRow& b,
                                     double alpha)
    {
        RuntimeRow r;

        r.t_phase_s = lerp(a.t_phase_s, b.t_phase_s, alpha);
        r.t_global_s = lerp(a.t_global_s, b.t_global_s, alpha);
        r.s_global_m = lerp(a.s_global_m, b.s_global_m, alpha);
        r.vx_mps = lerp(a.vx_mps, b.vx_mps, alpha);
        r.ax_model_mps2 = lerp(a.ax_model_mps2, b.ax_model_mps2, alpha);

        r.omega_front_radps = lerp(a.omega_front_radps, b.omega_front_radps, alpha);
        r.omega_rear_radps = lerp(a.omega_rear_radps, b.omega_rear_radps, alpha);

        r.v_front_wheel_mps = lerp(a.v_front_wheel_mps, b.v_front_wheel_mps, alpha);
        r.v_rear_wheel_mps = lerp(a.v_rear_wheel_mps, b.v_rear_wheel_mps, alpha);

        r.v_slip_front_mps = lerp(a.v_slip_front_mps, b.v_slip_front_mps, alpha);
        r.v_slip_rear_mps = lerp(a.v_slip_rear_mps, b.v_slip_rear_mps, alpha);

        r.kappa_front = lerp(a.kappa_front, b.kappa_front, alpha);
        r.kappa_rear = lerp(a.kappa_rear, b.kappa_rear, alpha);

        r.slack_front_mps = lerp(a.slack_front_mps, b.slack_front_mps, alpha);
        r.slack_rear_mps = lerp(a.slack_rear_mps, b.slack_rear_mps, alpha);

        r.M_total_cmd_Nm = lerp(a.M_total_cmd_Nm, b.M_total_cmd_Nm, alpha);
        r.M_front_total_cmd_Nm = lerp(a.M_front_total_cmd_Nm, b.M_front_total_cmd_Nm, alpha);
        r.M_rear_total_cmd_Nm = lerp(a.M_rear_total_cmd_Nm, b.M_rear_total_cmd_Nm, alpha);
        r.M_front_per_wheel_cmd_Nm = lerp(a.M_front_per_wheel_cmd_Nm, b.M_front_per_wheel_cmd_Nm, alpha);
        r.M_rear_per_wheel_cmd_Nm = lerp(a.M_rear_per_wheel_cmd_Nm, b.M_rear_per_wheel_cmd_Nm, alpha);

        r.M_total_act_Nm = lerp(a.M_total_act_Nm, b.M_total_act_Nm, alpha);
        r.M_front_total_act_Nm = lerp(a.M_front_total_act_Nm, b.M_front_total_act_Nm, alpha);
        r.M_rear_total_act_Nm = lerp(a.M_rear_total_act_Nm, b.M_rear_total_act_Nm, alpha);

        r.Fx_front_total_N = lerp(a.Fx_front_total_N, b.Fx_front_total_N, alpha);
        r.Fx_rear_total_N = lerp(a.Fx_rear_total_N, b.Fx_rear_total_N, alpha);
        r.Fx_total_N = lerp(a.Fx_total_N, b.Fx_total_N, alpha);
        r.F_resistance_N = lerp(a.F_resistance_N, b.F_resistance_N, alpha);

        r.Fz_front_total_N = lerp(a.Fz_front_total_N, b.Fz_front_total_N, alpha);
        r.Fz_rear_total_N = lerp(a.Fz_rear_total_N, b.Fz_rear_total_N, alpha);
        r.Fz_FL_N = lerp(a.Fz_FL_N, b.Fz_FL_N, alpha);
        r.Fz_FR_N = lerp(a.Fz_FR_N, b.Fz_FR_N, alpha);
        r.Fz_RL_N = lerp(a.Fz_RL_N, b.Fz_RL_N, alpha);
        r.Fz_RR_N = lerp(a.Fz_RR_N, b.Fz_RR_N, alpha);

        r.P_front_cmd_W = lerp(a.P_front_cmd_W, b.P_front_cmd_W, alpha);
        r.P_rear_cmd_W = lerp(a.P_rear_cmd_W, b.P_rear_cmd_W, alpha);
        r.P_total_cmd_W = lerp(a.P_total_cmd_W, b.P_total_cmd_W, alpha);
        r.P_total_cmd_kW = lerp(a.P_total_cmd_kW, b.P_total_cmd_kW, alpha);

        r.P_front_act_W = lerp(a.P_front_act_W, b.P_front_act_W, alpha);
        r.P_rear_act_W = lerp(a.P_rear_act_W, b.P_rear_act_W, alpha);
        r.P_total_act_W = lerp(a.P_total_act_W, b.P_total_act_W, alpha);
        r.P_total_act_kW = lerp(a.P_total_act_kW, b.P_total_act_kW, alpha);

        r.P_total_drive_limit_W = lerp(a.P_total_drive_limit_W, b.P_total_drive_limit_W, alpha);
        r.M_total_limit_Nm = lerp(a.M_total_limit_Nm, b.M_total_limit_Nm, alpha);
        r.M_axle_limit_Nm = lerp(a.M_axle_limit_Nm, b.M_axle_limit_Nm, alpha);

        r.power_limited_cmd = lerp(a.power_limited_cmd, b.power_limited_cmd, alpha);
        r.torque_limited_front = lerp(a.torque_limited_front, b.torque_limited_front, alpha);
        r.torque_limited_rear = lerp(a.torque_limited_rear, b.torque_limited_rear, alpha);

        r.initial_phase_end_flag = 0.0;
        r.initial_phase_region = alpha < 0.5 ? a.initial_phase_region : b.initial_phase_region;

        r.initial_phase_end_t_s = a.initial_phase_end_t_s;
        r.initial_phase_end_s_m = a.initial_phase_end_s_m;
        r.initial_phase_end_vx_mps = a.initial_phase_end_vx_mps;
        r.initial_phase_end_ax_mps2 = a.initial_phase_end_ax_mps2;
        r.initial_phase_end_reason = a.initial_phase_end_reason;

        return r;
    }

    LaunchRuntimeCommand toCommand(const RuntimeRow& r, bool valid) const
    {
        LaunchRuntimeCommand out;

        out.valid = valid;

        out.t_phase_s = r.t_phase_s;
        out.t_global_s = r.t_global_s;
        out.s_global_m = r.s_global_m;
        out.vx_mps = r.vx_mps;
        out.ax_model_mps2 = r.ax_model_mps2;

        out.omega_front_radps = r.omega_front_radps;
        out.omega_rear_radps = r.omega_rear_radps;

        out.v_front_wheel_mps = r.v_front_wheel_mps;
        out.v_rear_wheel_mps = r.v_rear_wheel_mps;

        out.v_slip_front_mps = r.v_slip_front_mps;
        out.v_slip_rear_mps = r.v_slip_rear_mps;

        out.kappa_front = r.kappa_front;
        out.kappa_rear = r.kappa_rear;

        out.slack_front_mps = r.slack_front_mps;
        out.slack_rear_mps = r.slack_rear_mps;

        out.M_total_cmd_Nm = r.M_total_cmd_Nm;
        out.M_front_total_cmd_Nm = r.M_front_total_cmd_Nm;
        out.M_rear_total_cmd_Nm = r.M_rear_total_cmd_Nm;
        out.M_front_per_wheel_cmd_Nm = r.M_front_per_wheel_cmd_Nm;
        out.M_rear_per_wheel_cmd_Nm = r.M_rear_per_wheel_cmd_Nm;

        out.M_total_act_Nm = r.M_total_act_Nm;
        out.M_front_total_act_Nm = r.M_front_total_act_Nm;
        out.M_rear_total_act_Nm = r.M_rear_total_act_Nm;

        out.Fx_front_total_N = r.Fx_front_total_N;
        out.Fx_rear_total_N = r.Fx_rear_total_N;
        out.Fx_total_N = r.Fx_total_N;
        out.F_resistance_N = r.F_resistance_N;

        out.Fz_front_total_N = r.Fz_front_total_N;
        out.Fz_rear_total_N = r.Fz_rear_total_N;
        out.Fz_FL_N = r.Fz_FL_N;
        out.Fz_FR_N = r.Fz_FR_N;
        out.Fz_RL_N = r.Fz_RL_N;
        out.Fz_RR_N = r.Fz_RR_N;

        out.P_front_cmd_W = r.P_front_cmd_W;
        out.P_rear_cmd_W = r.P_rear_cmd_W;
        out.P_total_cmd_W = r.P_total_cmd_W;
        out.P_total_cmd_kW = r.P_total_cmd_kW;

        out.P_front_act_W = r.P_front_act_W;
        out.P_rear_act_W = r.P_rear_act_W;
        out.P_total_act_W = r.P_total_act_W;
        out.P_total_act_kW = r.P_total_act_kW;

        out.P_total_drive_limit_W = r.P_total_drive_limit_W;
        out.M_total_limit_Nm = r.M_total_limit_Nm;
        out.M_axle_limit_Nm = r.M_axle_limit_Nm;

        out.power_limited_cmd = r.power_limited_cmd > 0.5;
        out.torque_limited_front = r.torque_limited_front > 0.5;
        out.torque_limited_rear = r.torque_limited_rear > 0.5;

        out.initial_phase_end_flag = r.initial_phase_end_flag > 0.5;
        out.initial_phase_region = r.initial_phase_region;

        out.initial_phase_end = initial_phase_end_;

        return out;
    }
};

} // namespace acc_runtime_maps