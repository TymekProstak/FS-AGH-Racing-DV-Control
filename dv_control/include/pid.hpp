#pragma once

#include <algorithm>
#include <cmath>

namespace dv_control
{

struct PIDParams {
    double Kp{0.0};
    double Ki{0.0};
    double Kd{0.0};

    double saturation_upper{0.0};
    double saturation_lower{0.0};

    double anti_windup_gain{0.0};
    double leak_time_scale{1.0};

    // ============================================================
    // Optional output rate limiter
    // ============================================================
    //
    // use_output_rate_limit == true:
    //      output PID-a nie może zmienić się szybciej niż:
    //          +output_rate_up   na sekundę
    //          -output_rate_down na sekundę
    //
    // Dla traction control jednostką outputu jest [Nm],
    // więc output_rate_up/down mają jednostkę [Nm/s].
    //
    // Dla innych PID-ów jednostka jest taka jak jednostka ich outputu.
    //
    bool use_output_rate_limit{false};
    double output_rate_up{0.0};
    double output_rate_down{0.0};
};

class PIDController {
public:
    PIDController() = default;
    explicit PIDController(const PIDParams& params) : params_(params) {}

    void set_params(const PIDParams& params) { params_ = params; }

    void reset()
    {
        integrator_ = 0.0;
        prev_error_ = 0.0;
        output_     = 0.0;
        p_term_     = 0.0;
        i_term_     = 0.0;
        d_term_     = 0.0;
        active      = false;
    }

    void update(double error, double dt, bool on_off = true, bool leak_even_when_on = false)
    {
        active = on_off;

        if (!std::isfinite(dt) || dt <= 1e-9) {
            p_term_ = i_term_ = d_term_ = 0.0;
            output_ = 0.0;
            return;
        }

        if (!std::isfinite(error)) {
            reset();
            active = on_off;
            return;
        }

        auto leak_integrator = [&]() {
            const double tau = (params_.leak_time_scale > 1e-9)
                                   ? params_.leak_time_scale
                                   : 1e-9;

            const double alpha = std::exp(-dt / tau);
            integrator_ *= alpha;
        };

        if (!on_off) {
            leak_integrator();

            p_term_ = 0.0;
            i_term_ = 0.0;
            d_term_ = 0.0;

            // Jeżeli PID jest wyłączony, output też schodzi do zera przez rate limiter,
            // jeśli limiter jest aktywny. Dzięki temu korekta TC nie znika skokowo.
            const double target_output = 0.0;
            output_ = apply_output_rate_limit_(target_output, dt);

            return;
        }

        if (leak_even_when_on) {
            if (std::abs(error) < 1e-3) {
                leak_integrator();
            }
        }

        p_term_ = params_.Kp * error;

        if (std::abs(params_.Ki) > 0.0) {
            integrator_ += error * dt;

            if (!std::isfinite(integrator_)) {
                reset();
                active = true;
                return;
            }

            i_term_ = params_.Ki * integrator_;
        } else {
            i_term_ = 0.0;
        }

        if (std::abs(params_.Kd) > 0.0) {
            const double derr = (error - prev_error_) / dt;
            d_term_ = params_.Kd * derr;
        } else {
            d_term_ = 0.0;
        }

        const double u_unsat = p_term_ + i_term_ + d_term_;

        if (!std::isfinite(u_unsat)) {
            reset();
            active = true;
            return;
        }

        const double u_sat = std::clamp(
            u_unsat,
            params_.saturation_lower,
            params_.saturation_upper
        );

        if (!std::isfinite(u_sat)) {
            reset();
            active = true;
            return;
        }

        if (std::abs(params_.Ki) > 0.0 && std::abs(params_.anti_windup_gain) > 0.0) {
            const double u_error = output_ - u_unsat;

            if (std::isfinite(u_error)) {
                integrator_ += (params_.anti_windup_gain / params_.Ki) * u_error * dt;

                if (!std::isfinite(integrator_)) {
                    reset();
                    active = true;
                    return;
                }

                i_term_ = params_.Ki * integrator_;
            }
        }

        output_ = apply_output_rate_limit_(u_sat, dt);

        

        prev_error_ = error;
    }

    double get_output() const { return output_; }
    double get_P_term() const { return p_term_; }
    double get_I_term_integrator() const { return i_term_; }
    double get_D_term() const { return d_term_; }

    bool is_active() const { return active; }

private:
    double apply_output_rate_limit_(double target_output, double dt)
    {
        if (!params_.use_output_rate_limit) {
            return target_output;
        }

        if (!std::isfinite(target_output)) {
            target_output = output_;
        }

        const double rate_up =
            std::max(0.0, params_.output_rate_up);

        const double rate_down =
            std::max(0.0, params_.output_rate_down);

        const double max_up_step =
            rate_up * dt;

        const double max_down_step =
            rate_down * dt;

        if (target_output > output_) {
            return std::min(target_output, output_ + max_up_step);
        }

        return std::max(target_output, output_ - max_down_step);
    }

private:
    PIDParams params_{};

    double integrator_{0.0};
    double prev_error_{0.0};
    double output_{0.0};

    double p_term_{0.0};
    double i_term_{0.0};
    double d_term_{0.0};

    bool active{false};
};

} // namespace dv_control