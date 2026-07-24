#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch-only AWD map generator for ACC launch control.

Conventions used by this generator:

    - all OCP decisions are wheel-side torques in Nm
    - drivetrain.M_wheel_drive_max_Nm is TOTAL car wheel torque limit [Nm]
    - front axle torque limit = max_motor_torque / 2
    - rear axle torque limit  = max_motor_torque / 2
    - drivetrain.P_wheel_drive_max_W is TOTAL car power limit [W]
    - output CSV stores front/rear/total torque in Nm
    - no braking profile is generated here

Runtime idea:
    map is only for initial launch.  The CSV also marks the detected end of
    the initial launch phase, where acceleration starts becoming plateau-like
    / power-limited.
"""

import argparse
import csv
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

import casadi as ca

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# =============================================================================
#                         DEFAULT PATHS
# =============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# offline_map_generator.py is expected here:
#   dv_acc_launch_control/include/longitudal_control/offline_map_generator.py
#
# Native config path:
#   dv_acc_launch_control/config/params.json
PACKAGE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
DEFAULT_CONFIG_PATH = os.path.join(PACKAGE_DIR, "config", "params.json")


# =============================================================================
#                               CONFIG HELPERS
# =============================================================================

def load_json_config(path: str) -> Dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def require_key(d: Dict, key: str, path: str):
    if key not in d:
        raise KeyError(f"Missing config key: {path}.{key}")
    return d[key]


def get_cfg(cfg: Dict) -> Dict:
    general = require_key(cfg, "general", "root")
    model = require_key(cfg, "model", "root")
    body = require_key(model, "body", "model")
    tire = require_key(model, "tire", "model")
    drivetrain = require_key(model, "drivetrain", "model")
    mass_transfer = require_key(model, "mass_transfer", "model")
    map_generator = require_key(cfg, "map_generator", "root")
    launch_map = require_key(cfg, "launch_map", "root")

    return {
        "general": general,
        "body": body,
        "tire": tire,
        "drivetrain": drivetrain,
        "mass_transfer": mass_transfer,
        "map_generator": map_generator,
        "launch_map": launch_map,
    }


def package_relative_output_root(config_path: str, maps_root: str) -> str:
    maps_root = os.path.expanduser(os.path.expandvars(maps_root))

    if os.path.isabs(maps_root):
        return maps_root

    # Typical ROS layout:
    #   package/config/params.json
    #   package/include/longitudal_control/launch_brake_maps
    config_dir = os.path.dirname(os.path.abspath(config_path))
    package_dir = os.path.abspath(os.path.join(config_dir, ".."))
    return os.path.abspath(os.path.join(package_dir, maps_root))


def ensure_dirs(root: str) -> Dict[str, str]:
    paths = {
        "root": root,
        "launch_runtime": os.path.join(root, "launch_runtime_profiles"),
        "plots": os.path.join(root, "plots"),
        "info": os.path.join(root, "info"),
    }

    for p in paths.values():
        os.makedirs(p, exist_ok=True)

    return paths


def make_float_tag(value: float, prefix: str) -> str:
    return f"{prefix}_{value:.2f}".replace(".", "p").replace("-", "m")


def make_mu_tag(mu: float) -> str:
    return make_float_tag(mu, "mu")


def make_s_tag(S: float) -> str:
    return make_float_tag(S, "S")


# =============================================================================
#                               NUMERIC HELPERS
# =============================================================================

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def ceil_to_grid(x: float, grid: float) -> float:
    return math.ceil(x / grid) * grid


def slip_denominator_numeric(vx: float, tire: Dict) -> float:
    v_th = max(float(tire["v_threshold"]), 1.0e-9)
    v_abs = abs(vx)
    denom_low = 0.5 * (v_th + vx * vx / v_th)
    denom_high = v_abs
    if v_abs > v_th:
        return max(denom_high, 1.0e-9)
    return max(denom_low, 1.0e-9)


def regularized_kappa_numeric(vx: float, omega: float, tire: Dict) -> float:
    R = float(tire["R_tire"])
    return (R * omega - vx) / slip_denominator_numeric(vx, tire)


def resistance_force_numeric(vx: float, body: Dict) -> float:
    v = abs(vx)
    m = float(body["m"])
    g = float(body["g"])
    Crr = float(body["rolling_resistance_coeff"])
    Cd = float(body["Cd"])
    F_const = float(body["resistance_constant_N"])
    F_lin = float(body["resistance_linear_N_per_mps"])

    # Same runtime-style resistance model:
    #   rolling + lumped quadratic drag + empirical constant + linear part
    return Crr * m * g + Cd * v * v + F_const + F_lin * v


def aero_downforce_front_rear_numeric(vx: float, body: Dict) -> Tuple[float, float]:
    v = abs(vx)
    return float(body["Cl1"]) * v * v, float(body["Cl2"]) * v * v


def normal_loads_front_rear_numeric(vx: float, ax: float, cfg: Dict) -> Tuple[float, float]:
    body = cfg["body"]
    mt = cfg["mass_transfer"]

    m = float(body["m"])
    g = float(body["g"])
    lf = float(body["l_f"])
    lr = float(body["l_r"])
    h = float(body["h_cg"])
    L = max(lf + lr, 1.0e-9)

    Fz_front_aero, Fz_rear_aero = aero_downforce_front_rear_numeric(vx, body)

    Fz_front = m * g * lr / L + Fz_front_aero
    Fz_rear = m * g * lf / L + Fz_rear_aero

    if mt.get("enabled", True):
        anti_squat = clamp(float(mt.get("anti_squat", 0.0)), 0.0, 1.0)
        anti_dive = clamp(float(mt.get("anti_dive", 0.0)), 0.0, 1.0)
        direct_fraction = anti_squat if ax >= 0.0 else anti_dive
        dFz = m * ax * h / L * (1.0 - direct_fraction)
        Fz_front -= dFz
        Fz_rear += dFz

    Fz_min_axle = 2.0 * float(mt["Fz_min_N"])
    Fz_max_axle = 2.0 * float(mt["Fz_max_N"])

    Fz_front = clamp(Fz_front, Fz_min_axle, Fz_max_axle)
    Fz_rear = clamp(Fz_rear, Fz_min_axle, Fz_max_axle)

    return Fz_front, Fz_rear


def optimistic_accel(vx: float, cfg: Dict) -> float:
    body = cfg["body"]
    tire = cfg["tire"]
    drv = cfg["drivetrain"]

    m = float(body["m"])
    R = float(tire["R_tire"])
    mu = float(cfg["general"].get("mu_x", 0.8))
    max_total_torque = float(drv["M_wheel_drive_max_Nm"])
    Pmax = float(drv["P_wheel_drive_max_W"])

    # Simple fixed point on load transfer and traction limit.
    ax = 4.0
    for _ in range(8):
        Fz_f, Fz_r = normal_loads_front_rear_numeric(vx, ax, cfg)
        F_trac = mu * (Fz_f + Fz_r)
        F_torque = max_total_torque / max(R, 1.0e-9)
        F_power = Pmax / max(vx, 1.0)
        F_res = resistance_force_numeric(vx, body)
        ax_new = (min(F_trac, F_torque, F_power) - F_res) / max(m, 1.0e-9)
        ax = 0.5 * ax + 0.5 * ax_new

    return max(ax, 0.3)


def estimate_launch_horizon_s(S: float, cfg: Dict) -> float:
    dt = 0.002
    t = 0.0
    s = 0.0
    vx = 0.0

    guard = 60.0
    while s < S and t < guard:
        ax = optimistic_accel(vx, cfg)
        vx = max(0.0, vx + ax * dt)
        s += vx * dt
        t += dt

    if s < S:
        raise RuntimeError("Could not estimate launch horizon before guard time")

    dt_control = float(cfg["map_generator"]["dt_control"])
    return ceil_to_grid(1.25 * t, dt_control)


# =============================================================================
#                               CASADI MODEL
# =============================================================================

def smooth_abs(x):
    return ca.sqrt(x * x + 1.0e-12)


def slip_denominator_expr(vx, tire: Dict):
    v_th = float(tire["v_threshold"])
    v_abs = smooth_abs(vx)
    denom_low = 0.5 * (v_th + vx * vx / v_th)
    denom_high = ca.sqrt(vx * vx + 1.0e-6)
    return ca.if_else(v_abs > v_th, denom_high, denom_low)


def resistance_force_expr(vx, body: Dict):
    v = smooth_abs(vx)
    m = float(body["m"])
    g = float(body["g"])
    Crr = float(body["rolling_resistance_coeff"])
    Cd = float(body["Cd"])
    F_const = float(body["resistance_constant_N"])
    F_lin = float(body["resistance_linear_N_per_mps"])
    return Crr * m * g + Cd * v * v + F_const + F_lin * v


def normal_loads_front_rear_expr(vx, ax, cfg: Dict):
    body = cfg["body"]
    mt = cfg["mass_transfer"]

    m = float(body["m"])
    g = float(body["g"])
    lf = float(body["l_f"])
    lr = float(body["l_r"])
    h = float(body["h_cg"])
    L = lf + lr
    v_abs = smooth_abs(vx)

    Fz_front = m * g * lr / L + float(body["Cl1"]) * v_abs * v_abs
    Fz_rear = m * g * lf / L + float(body["Cl2"]) * v_abs * v_abs

    if mt.get("enabled", True):
        anti_squat = clamp(float(mt.get("anti_squat", 0.0)), 0.0, 1.0)
        anti_dive = clamp(float(mt.get("anti_dive", 0.0)), 0.0, 1.0)
        direct_fraction = ca.if_else(ax >= 0.0, anti_squat, anti_dive)
        dFz = m * ax * h / L * (1.0 - direct_fraction)
        Fz_front = Fz_front - dFz
        Fz_rear = Fz_rear + dFz

    Fz_min_axle = 2.0 * float(mt["Fz_min_N"])
    Fz_max_axle = 2.0 * float(mt["Fz_max_N"])

    Fz_front = ca.fmin(ca.fmax(Fz_front, Fz_min_axle), Fz_max_axle)
    Fz_rear = ca.fmin(ca.fmax(Fz_rear, Fz_min_axle), Fz_max_axle)
    return Fz_front, Fz_rear


def dynamics(x, u, cfg: Dict):
    # x = [s, vx, omega_f, omega_r, M_f_act, M_r_act, Fx_f_rel, Fx_r_rel]
    s = x[0]
    vx = x[1]
    omega_f = x[2]
    omega_r = x[3]
    M_f_act = x[4]       # front axle total torque [Nm]
    M_r_act = x[5]       # rear axle total torque [Nm]
    Fx_f_rel = x[6]      # front axle total force [N]
    Fx_r_rel = x[7]      # rear axle total force [N]

    M_f_cmd = u[0]
    M_r_cmd = u[1]

    body = cfg["body"]
    tire = cfg["tire"]
    drv = cfg["drivetrain"]

    m = float(body["m"])
    R = float(tire["R_tire"])
    Iw = float(tire["I_wheel"])
    Ck = float(tire["Ckappa_tanh"])
    mu = float(cfg["general"].get("mu_x", 0.8))

    F_res = resistance_force_expr(vx, body)
    Fx_total = Fx_f_rel + Fx_r_rel
    ax = (Fx_total - F_res) / m

    Fz_front, Fz_rear = normal_loads_front_rear_expr(vx, ax, cfg)
    Fz_f_wheel = 0.5 * Fz_front
    Fz_r_wheel = 0.5 * Fz_rear

    kappa_f = (R * omega_f - vx) / (slip_denominator_expr(vx, tire) + 1.0e-9)
    kappa_r = (R * omega_r - vx) / (slip_denominator_expr(vx, tire) + 1.0e-9)

    Fx_f_ss_per_wheel = mu * Fz_f_wheel * ca.tanh(Ck * kappa_f)
    Fx_r_ss_per_wheel = mu * Fz_r_wheel * ca.tanh(Ck * kappa_r)

    Fx_f_ss = 2.0 * Fx_f_ss_per_wheel
    Fx_r_ss = 2.0 * Fx_r_ss_per_wheel

    v_relax = ca.sqrt(vx * vx + float(tire["v_threshold"]) ** 2)
    tau_x = float(tire["relaxation_length_long"]) / (v_relax + 1.0e-9)

    dFx_f_rel = (Fx_f_ss - Fx_f_rel) / (tau_x + 1.0e-9)
    dFx_r_rel = (Fx_r_ss - Fx_r_rel) / (tau_x + 1.0e-9)

    # axle torque -> per wheel torque for rotational dynamics
    domega_f = (0.5 * M_f_act - R * 0.5 * Fx_f_rel) / (Iw + 1.0e-9)
    domega_r = (0.5 * M_r_act - R * 0.5 * Fx_r_rel) / (Iw + 1.0e-9)

    tau_M = float(drv["time_constant"])
    dM_f_act = (M_f_cmd - M_f_act) / (tau_M + 1.0e-9)
    dM_r_act = (M_r_cmd - M_r_act) / (tau_M + 1.0e-9)

    return ca.vertcat(
        vx,
        ax,
        domega_f,
        domega_r,
        dM_f_act,
        dM_r_act,
        dFx_f_rel,
        dFx_r_rel,
    )


def integrate_one_interval(x, u, cfg: Dict):
    dt_control = float(cfg["map_generator"]["dt_control"])
    dt_dyn = float(cfg["map_generator"]["dt_dynamics"])
    substeps = int(round(dt_control / dt_dyn))

    if abs(substeps * dt_dyn - dt_control) > 1.0e-9:
        raise ValueError("map_generator.dt_control / dt_dynamics must be integer")

    x_next = x
    for _ in range(substeps):
        x_next = x_next + dt_dyn * dynamics(x_next, u, cfg)

    return x_next


# =============================================================================
#                               OCP SOLVER
# =============================================================================

def make_solver_opts(cfg: Dict) -> Dict:
    s = cfg["map_generator"].get("solver", {})
    return {
        "ipopt.max_iter": int(s.get("max_iter", 700)),
        "ipopt.tol": float(s.get("tol", 1.0e-5)),
        "ipopt.acceptable_tol": float(s.get("acceptable_tol", 1.0e-3)),
        "ipopt.acceptable_iter": int(s.get("acceptable_iter", 10)),
        "ipopt.print_level": int(s.get("ipopt_print_level", 3)),
        "print_time": bool(s.get("print_time", False)),
    }


def build_and_solve_launch_ocp(cfg: Dict, mu_x: float, S_target_m: float):
    cfg = json.loads(json.dumps(cfg))
    cfg["general"]["mu_x"] = float(mu_x)

    mg = cfg["map_generator"]
    tire = cfg["tire"]
    drv = cfg["drivetrain"]

    dt = float(mg["dt_control"])
    T = estimate_launch_horizon_s(S_target_m, cfg)
    N = max(2, int(round(T / dt)))

    nx = 8
    nu = 2

    opti = ca.Opti()
    X = opti.variable(nx, N + 1)
    U = opti.variable(nu, N)
    slack_f = opti.variable(1, N)
    slack_r = opti.variable(1, N)

    R = float(tire["R_tire"])
    max_total_torque = float(drv["M_wheel_drive_max_Nm"])
    max_axle_torque = 0.5 * max_total_torque
    max_power_W = float(drv["P_wheel_drive_max_W"])

    x0 = ca.vertcat(
        0.0,     # s
        0.0,     # vx
        0.0,     # omega_f
        0.0,     # omega_r
        0.0,     # M_f_act
        0.0,     # M_r_act
        0.0,     # Fx_f_rel
        0.0,     # Fx_r_rel
    )

    opti.subject_to(X[:, 0] == x0)
    opti.subject_to(slack_f >= 0.0)
    opti.subject_to(slack_r >= 0.0)

    opti.subject_to(opti.bounded(0.0, U[0, :], max_axle_torque))
    opti.subject_to(opti.bounded(0.0, U[1, :], max_axle_torque))

    J_slack = 0.0
    J_dM = 0.0

    kappa_lim = float(tire["kappa_limit"])

    # Existing config value is dimensionless per second from the old normalized map.
    # Internally it is converted to Nm/s so decision variables are still pure Nm.
    M_dot_up_Nmps = float(mg.get("M_cmd_dot_up_max", 6.0)) * max_axle_torque
    M_dot_down_Nmps = float(mg.get("M_cmd_dot_down_max", 6.0)) * max_axle_torque

    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]
        x_next = integrate_one_interval(xk, uk, cfg)
        opti.subject_to(X[:, k + 1] == x_next)

        vx = X[1, k]
        omega_f = X[2, k]
        omega_r = X[3, k]

        opti.subject_to(vx >= -1.0e-6)
        opti.subject_to(omega_f >= -1.0e-6)
        opti.subject_to(omega_r >= -1.0e-6)

        # total car power limit; front/rear are axle-total torques
        P_total = U[0, k] * omega_f + U[1, k] * omega_r
        opti.subject_to(P_total <= max_power_W)

        denom = slip_denominator_expr(vx, tire)
        v_slip_f = R * omega_f - vx
        v_slip_r = R * omega_r - vx
        v_slip_lim = kappa_lim * denom

        opti.subject_to(v_slip_f <= v_slip_lim + slack_f[0, k])
        opti.subject_to(v_slip_r <= v_slip_lim + slack_r[0, k])
        opti.subject_to(v_slip_f >= -v_slip_lim - slack_f[0, k])
        opti.subject_to(v_slip_r >= -v_slip_lim - slack_r[0, k])

        if k == 0:
            dMf = U[0, k] - 0.0
            dMr = U[1, k] - 0.0
        else:
            dMf = U[0, k] - U[0, k - 1]
            dMr = U[1, k] - U[1, k - 1]

        opti.subject_to(dMf <= M_dot_up_Nmps * dt)
        opti.subject_to(dMr <= M_dot_up_Nmps * dt)
        opti.subject_to(dMf >= -M_dot_down_Nmps * dt)
        opti.subject_to(dMr >= -M_dot_down_Nmps * dt)

        J_slack += slack_f[0, k] * slack_f[0, k] + slack_r[0, k] * slack_r[0, k]
        J_dM += dMf * dMf + dMr * dMr

    weights = mg.get("weights", {})
    w_distance = float(weights.get("distance", 100.0))
    w_slip = float(weights.get("slip_slack", 5000.0))
    w_rate = float(weights.get("torque_rate", 5.0)) / max(max_axle_torque * max_axle_torque, 1.0)

    s_final = X[0, N]
    J = -w_distance * s_final + w_slip * J_slack + w_rate * J_dM
    opti.minimize(J)

    # Initial guess: balanced AWD and optimistic kinematics.
    vx_guess = 0.0
    s_guess = 0.0
    for k in range(N + 1):
        t = k * dt
        axg = optimistic_accel(vx_guess, cfg)
        if k > 0:
            vx_guess = max(0.0, vx_guess + axg * dt)
            s_guess += vx_guess * dt
        omega_guess = vx_guess / R
        opti.set_initial(X[0, k], s_guess)
        opti.set_initial(X[1, k], vx_guess)
        opti.set_initial(X[2, k], omega_guess)
        opti.set_initial(X[3, k], omega_guess)
        opti.set_initial(X[4, k], 0.45 * max_axle_torque)
        opti.set_initial(X[5, k], 0.45 * max_axle_torque)
        opti.set_initial(X[6, k], 0.0)
        opti.set_initial(X[7, k], 0.0)

    Mf_guess = 0.0
    Mr_guess = 0.0
    Mf_target = 0.45 * max_axle_torque
    Mr_target = 0.45 * max_axle_torque

    for k in range(N):
        # Initial guess respects the same torque-rate constraints as the NLP.
        Mf_guess = min(Mf_target, Mf_guess + M_dot_up_Nmps * dt)
        Mr_guess = min(Mr_target, Mr_guess + M_dot_up_Nmps * dt)

        opti.set_initial(U[0, k], Mf_guess)
        opti.set_initial(U[1, k], Mr_guess)
        opti.set_initial(slack_f[0, k], 0.0)
        opti.set_initial(slack_r[0, k], 0.0)

    opti.solver("ipopt", make_solver_opts(cfg))
    sol = opti.solve()

    return sol.value(X), sol.value(U), sol.value(slack_f), sol.value(slack_r), N, dt, cfg


# =============================================================================
#                               POSTPROCESSING
# =============================================================================

def value_1xn_or_1d(arr, k: int) -> float:
    """
    CasADi/NumPy may return Opti variable values with shape (1, N) or flattened
    shape (N,). Both represent the same slack sequence.
    """
    try:
        return float(arr[0, k])
    except (IndexError, TypeError):
        return float(arr[k])


def compute_row(k: int,
                Xv,
                Uv,
                slack_f,
                slack_r,
                dt: float,
                cfg: Dict,
                mu_x: float,
                S_target_m: float) -> Dict[str, float]:
    body = cfg["body"]
    tire = cfg["tire"]
    drv = cfg["drivetrain"]

    R = float(tire["R_tire"])

    s = float(Xv[0, k])
    vx = float(Xv[1, k])
    omega_f = float(Xv[2, k])
    omega_r = float(Xv[3, k])
    M_front_act = float(Xv[4, k])
    M_rear_act = float(Xv[5, k])
    Fx_front = float(Xv[6, k])
    Fx_rear = float(Xv[7, k])

    if k < Uv.shape[1]:
        M_front_cmd = float(Uv[0, k])
        M_rear_cmd = float(Uv[1, k])
        slack_f_k = value_1xn_or_1d(slack_f, k)
        slack_r_k = value_1xn_or_1d(slack_r, k)
    else:
        M_front_cmd = float(Uv[0, -1])
        M_rear_cmd = float(Uv[1, -1])
        slack_f_k = value_1xn_or_1d(slack_f, -1)
        slack_r_k = value_1xn_or_1d(slack_r, -1)

    F_res = resistance_force_numeric(vx, body)
    ax = (Fx_front + Fx_rear - F_res) / float(body["m"])
    Fz_front, Fz_rear = normal_loads_front_rear_numeric(vx, ax, cfg)

    kappa_f = regularized_kappa_numeric(vx, omega_f, tire)
    kappa_r = regularized_kappa_numeric(vx, omega_r, tire)

    v_slip_f = R * omega_f - vx
    v_slip_r = R * omega_r - vx

    P_front_cmd = M_front_cmd * omega_f
    P_rear_cmd = M_rear_cmd * omega_r
    P_total_cmd = P_front_cmd + P_rear_cmd

    P_front_act = M_front_act * omega_f
    P_rear_act = M_rear_act * omega_r
    P_total_act = P_front_act + P_rear_act

    M_total_cmd = M_front_cmd + M_rear_cmd
    M_total_act = M_front_act + M_rear_act

    max_total_torque = float(drv["M_wheel_drive_max_Nm"])
    max_axle_torque = 0.5 * max_total_torque
    power_limit = float(drv["P_wheel_drive_max_W"])

    power_limited_cmd = int(P_total_cmd >= 0.98 * power_limit)
    torque_limited_front = int(M_front_cmd >= 0.98 * max_axle_torque)
    torque_limited_rear = int(M_rear_cmd >= 0.98 * max_axle_torque)

    return {
        "phase": "launch",
        "sample_index": k,
        "t_phase_s": k * dt,
        "t_global_s": k * dt,
        "s_global_m": s,
        "S_total_m": S_target_m,
        "mu_x": mu_x,
        "vx_mps": vx,
        "ax_model_mps2": ax,
        "omega_front_radps": omega_f,
        "omega_rear_radps": omega_r,
        "v_front_wheel_mps": R * omega_f,
        "v_rear_wheel_mps": R * omega_r,
        "v_slip_front_mps": v_slip_f,
        "v_slip_rear_mps": v_slip_r,
        "kappa_front": kappa_f,
        "kappa_rear": kappa_r,
        "slack_front_mps": slack_f_k,
        "slack_rear_mps": slack_r_k,
        "M_total_cmd_Nm": M_total_cmd,
        "M_front_total_cmd_Nm": M_front_cmd,
        "M_rear_total_cmd_Nm": M_rear_cmd,
        "M_front_per_wheel_cmd_Nm": 0.5 * M_front_cmd,
        "M_rear_per_wheel_cmd_Nm": 0.5 * M_rear_cmd,
        "M_total_act_Nm": M_total_act,
        "M_front_total_act_Nm": M_front_act,
        "M_rear_total_act_Nm": M_rear_act,
        "Fx_front_total_N": Fx_front,
        "Fx_rear_total_N": Fx_rear,
        "Fx_total_N": Fx_front + Fx_rear,
        "F_resistance_N": F_res,
        "Fz_front_total_N": Fz_front,
        "Fz_rear_total_N": Fz_rear,
        "Fz_FL_N": 0.5 * Fz_front,
        "Fz_FR_N": 0.5 * Fz_front,
        "Fz_RL_N": 0.5 * Fz_rear,
        "Fz_RR_N": 0.5 * Fz_rear,
        "P_front_cmd_W": P_front_cmd,
        "P_rear_cmd_W": P_rear_cmd,
        "P_total_cmd_W": P_total_cmd,
        "P_total_cmd_kW": P_total_cmd / 1000.0,
        "P_front_act_W": P_front_act,
        "P_rear_act_W": P_rear_act,
        "P_total_act_W": P_total_act,
        "P_total_act_kW": P_total_act / 1000.0,
        "P_total_drive_limit_W": power_limit,
        "M_total_limit_Nm": max_total_torque,
        "M_axle_limit_Nm": max_axle_torque,
        "power_limited_cmd": power_limited_cmd,
        "torque_limited_front": torque_limited_front,
        "torque_limited_rear": torque_limited_rear,
    }


def trim_rows_to_distance(rows: List[Dict], S: float) -> List[Dict]:
    if not rows:
        return rows

    out = []
    for i, row in enumerate(rows):
        if row["s_global_m"] <= S:
            out.append(row)
            continue

        if i == 0 or not out:
            break

        a = rows[i - 1]
        b = row
        ds = b["s_global_m"] - a["s_global_m"]
        alpha = 0.0 if abs(ds) <= 1.0e-12 else clamp((S - a["s_global_m"]) / ds, 0.0, 1.0)
        interp = {}
        for key in a.keys():
            av = a[key]
            bv = b[key]
            if isinstance(av, (int, float)) and isinstance(bv, (int, float)):
                interp[key] = av + alpha * (bv - av)
            else:
                interp[key] = av
        interp["s_global_m"] = S
        out.append(interp)
        break

    return out



def moving_average_same_length(values: List[float], window: int) -> List[float]:
    if not values:
        return []

    window = max(1, int(window))
    if window <= 1:
        return list(values)

    half = window // 2
    out = []

    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        out.append(sum(values[lo:hi]) / max(1, hi - lo))

    return out


def first_index_at_or_after_time(rows: List[Dict], t_s: float) -> int:
    for i, r in enumerate(rows):
        if float(r["t_phase_s"]) >= t_s:
            return i
    return max(0, len(rows) - 1)


def detect_initial_launch_end(rows: List[Dict]) -> Dict[str, float]:
    if not rows:
        return {
            "valid": 0,
            "t_s": 0.0,
            "s_m": 0.0,
            "vx_mps": 0.0,
            "ax_mps2": 0.0,
            "reason": "empty",
            "ax_peak_mps2": 0.0,
        }

    if len(rows) < 5:
        idx = len(rows) - 1
        r = rows[idx]
        ax_values = [float(x["ax_model_mps2"]) for x in rows]
        return {
            "valid": 1,
            "index": idx,
            "t_s": float(r["t_phase_s"]),
            "s_m": float(r["s_global_m"]),
            "vx_mps": float(r["vx_mps"]),
            "ax_mps2": float(r["ax_model_mps2"]),
            "reason": "too_short",
            "ax_peak_mps2": max(ax_values),
        }

    t_values = [float(r["t_phase_s"]) for r in rows]
    ax_values = [float(r["ax_model_mps2"]) for r in rows]

    dt = max(
        1.0e-9,
        t_values[1] - t_values[0]
    )

    ax_peak = max(ax_values)

    # Python note:
    # This detector is intentionally NOT based on power limit and NOT based
    # on the global ax peak.
    # The runtime launch segment should be the first steep rise from the OCP
    # map only. In the example plot this is the part ending near 0.5 s, where
    # ax stops rising rapidly and starts becoming much flatter.
    # Smoothing over about 0.10 s removes single-sample OCP noise.
    smooth_window = max(
        3,
        int(round(0.10 / dt))
    )

    ax_smooth = moving_average_same_length(
        ax_values,
        smooth_window
    )

    dax_dt = [0.0]
    for i in range(1, len(ax_smooth)):
        dax_dt.append(
            (ax_smooth[i] - ax_smooth[i - 1]) / dt
        )

    # Detect only the initial ramp, not late power-limited behaviour.
    search_start_idx = first_index_at_or_after_time(rows, 0.12)
    search_end_idx = first_index_at_or_after_time(
        rows,
        min(1.20, max(t_values[-1], 0.12))
    )

    if search_end_idx <= search_start_idx:
        search_start_idx = 1
        search_end_idx = len(rows) - 1

    peak_slope_idx = search_start_idx
    peak_slope = dax_dt[search_start_idx]

    for i in range(search_start_idx, search_end_idx + 1):
        if dax_dt[i] > peak_slope:
            peak_slope = dax_dt[i]
            peak_slope_idx = i

    if peak_slope <= 1.0e-9:
        idx = len(rows) - 1
        reason = "no_positive_ax_slope"
    else:
        slope_threshold = 0.12 * peak_slope

        ax_ref_first_second = max(
            ax_smooth[0:search_end_idx + 1]
        )

        min_ax_for_end = 0.85 * ax_ref_first_second

        hold_count = max(
            3,
            int(round(0.12 / dt))
        )

        idx = None
        reason = ""

        for i in range(max(peak_slope_idx + 1, search_start_idx), len(rows) - hold_count):
            if t_values[i] < 0.25:
                continue

            if ax_smooth[i] < min_ax_for_end:
                continue

            slope_window = dax_dt[i:i + hold_count]

            if all(x <= slope_threshold for x in slope_window):
                idx = i
                reason = "initial_ax_ramp_knee"
                break

        if idx is None:
            # Fallback: first point after 0.25 s closest to 90% of the first-second ax.
            target_ax = 0.90 * ax_ref_first_second
            idx = first_index_at_or_after_time(rows, 0.25)

            best_error = abs(ax_smooth[idx] - target_ax)

            for i in range(idx, search_end_idx + 1):
                err = abs(ax_smooth[i] - target_ax)
                if err < best_error:
                    best_error = err
                    idx = i

            reason = "initial_ax_90pct_first_second"

    r = rows[idx]

    return {
        "valid": 1,
        "index": idx,
        "t_s": float(r["t_phase_s"]),
        "s_m": float(r["s_global_m"]),
        "vx_mps": float(r["vx_mps"]),
        "ax_mps2": float(r["ax_model_mps2"]),
        "reason": reason,
        "ax_peak_mps2": ax_peak,
    }


def annotate_rows_with_initial_end(rows: List[Dict], info: Dict[str, float]) -> None:
    idx = int(info.get("index", -1))
    for i, r in enumerate(rows):
        r["initial_phase_end_flag"] = int(i == idx)
        r["initial_phase_region"] = "initial" if i <= idx else "plateau"
        r["initial_phase_end_t_s"] = info["t_s"]
        r["initial_phase_end_s_m"] = info["s_m"]
        r["initial_phase_end_vx_mps"] = info["vx_mps"]
        r["initial_phase_end_ax_mps2"] = info["ax_mps2"]
        r["initial_phase_end_reason"] = info["reason"]


def save_csv(rows: List[Dict], path: str) -> None:
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_info_txt(path: str, rows: List[Dict], info: Dict[str, float]) -> None:
    if not rows:
        return

    last = rows[-1]
    with open(path, "w", encoding="utf-8") as f:
        f.write("launch_map_info\n")
        f.write(f"S_total_m: {last['S_total_m']:.10g}\n")
        f.write(f"mu_x: {last['mu_x']:.10g}\n")
        f.write(f"launch_final_t_s: {last['t_phase_s']:.10g}\n")
        f.write(f"launch_final_s_m: {last['s_global_m']:.10g}\n")
        f.write(f"launch_final_vx_mps: {last['vx_mps']:.10g}\n")
        f.write(f"initial_phase_end_t_s: {info['t_s']:.10g}\n")
        f.write(f"initial_phase_end_s_m: {info['s_m']:.10g}\n")
        f.write(f"initial_phase_end_vx_mps: {info['vx_mps']:.10g}\n")
        f.write(f"initial_phase_end_ax_mps2: {info['ax_mps2']:.10g}\n")
        f.write(f"initial_phase_end_reason: {info['reason']}\n")
        f.write(f"ax_peak_mps2: {info['ax_peak_mps2']:.10g}\n")


# =============================================================================
#                               PLOTS
# =============================================================================

def _series(rows: List[Dict], x: str, y: str):
    xs, ys = [], []
    for r in rows:
        try:
            xs.append(float(r[x]))
            ys.append(float(r[y]))
        except Exception:
            pass
    return xs, ys



def interpolate_series_value_at_x(rows: List[Dict], x_key: str, y_key: str, x0: float):
    pairs = []

    for r in rows:
        try:
            pairs.append((float(r[x_key]), float(r[y_key])))
        except Exception:
            pass

    if not pairs:
        return None

    pairs.sort(key=lambda p: p[0])

    if x0 <= pairs[0][0]:
        return pairs[0][1]

    if x0 >= pairs[-1][0]:
        return pairs[-1][1]

    for i in range(1, len(pairs)):
        x_prev, y_prev = pairs[i - 1]
        x_next, y_next = pairs[i]

        if x_prev <= x0 <= x_next:
            dx = x_next - x_prev

            if abs(dx) <= 1.0e-12:
                return y_next

            alpha = (x0 - x_prev) / dx

            return y_prev + alpha * (y_next - y_prev)

    return pairs[-1][1]


def initial_end_x_for_axis(x_key: str, end_info: Dict[str, float]) -> float:
    if x_key == "t_phase_s":
        return float(end_info["t_s"])

    if x_key == "s_global_m":
        return float(end_info["s_m"])

    if x_key == "vx_mps":
        return float(end_info["vx_mps"])

    return float(end_info["t_s"])



def save_plot(rows: List[Dict], x_key: str, ys: List[Tuple[str, str]], title: str,
              xlabel: str, ylabel: str, path: str, end_info: Dict[str, float]) -> None:
    plt.figure()
    any_plot = False

    for y_key, label in ys:
        xs, vals = _series(rows, x_key, y_key)

        if xs:
            plt.plot(xs, vals, label=label)
            any_plot = True

    if not any_plot:
        plt.close()
        return

    x_end = initial_end_x_for_axis(
        x_key,
        end_info
    )

    plt.axvline(
        x_end,
        linestyle="--",
        label="initial phase end"
    )

    first_marker = True

    for y_key, _label in ys:
        y_end = interpolate_series_value_at_x(
            rows,
            x_key,
            y_key,
            x_end
        )

        if y_end is None:
            continue

        plt.scatter(
            [x_end],
            [y_end],
            s=70,
            zorder=5,
            label="initial phase end point" if first_marker else "_nolegend_"
        )

        first_marker = False

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()




def save_initial_detection_plot(rows: List[Dict], path: str, end_info: Dict[str, float]) -> None:
    if not rows or len(rows) < 3:
        return

    t_values = [float(r["t_phase_s"]) for r in rows]
    ax_values = [float(r["ax_model_mps2"]) for r in rows]

    dt = max(1.0e-9, t_values[1] - t_values[0])
    smooth_window = max(3, int(round(0.10 / dt)))
    ax_smooth = moving_average_same_length(ax_values, smooth_window)

    dax_dt = [0.0]
    for i in range(1, len(ax_smooth)):
        dax_dt.append((ax_smooth[i] - ax_smooth[i - 1]) / dt)

    fig, ax1 = plt.subplots(figsize=(10, 6))

    ax1.plot(t_values, ax_values, label="ax raw")
    ax1.plot(t_values, ax_smooth, label="ax smoothed")
    ax1.axvline(end_info["t_s"], linestyle="--", label="initial phase end")

    ax_end = interpolate_series_value_at_x(
        rows,
        "t_phase_s",
        "ax_model_mps2",
        end_info["t_s"]
    )

    if ax_end is not None:
        ax1.scatter(
            [end_info["t_s"]],
            [ax_end],
            s=80,
            zorder=6,
            label="initial phase end point"
        )
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("ax [m/s²]")
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.plot(t_values, dax_dt, linestyle=":", label="dax/dt")
    ax2.set_ylabel("dax/dt [m/s³]")

    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc="best")

    fig.suptitle(
        "Initial launch end detection "
        f"(t={end_info['t_s']:.3f}s, reason={end_info['reason']})"
    )

    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)



def save_overview_plot(rows: List[Dict], path: str, end_info: Dict[str, float]) -> None:
    """
    One quick-check plot generated every time a launch map is solved.

    It is meant for fast visual verification that the generator really produced
    a sensible profile:
        - total torque vs speed,
        - speed vs distance,
        - acceleration vs distance,
        - power vs speed.
    """
    if not rows:
        return

    vx, torque = _series(rows, "vx_mps", "M_total_cmd_Nm")
    s, speed = _series(rows, "s_global_m", "vx_mps")
    s_ax, ax = _series(rows, "s_global_m", "ax_model_mps2")
    vx_p, power = _series(rows, "vx_mps", "P_total_cmd_kW")

    if not vx or not s:
        return

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    axes[0, 0].plot(vx, torque)
    axes[0, 0].axvline(end_info["vx_mps"], linestyle="--")
    torque_end = interpolate_series_value_at_x(rows, "vx_mps", "M_total_cmd_Nm", end_info["vx_mps"])
    if torque_end is not None:
        axes[0, 0].scatter([end_info["vx_mps"]], [torque_end], s=70, zorder=5)
    axes[0, 0].set_title("Total torque vs speed")
    axes[0, 0].set_xlabel("vx [m/s]")
    axes[0, 0].set_ylabel("torque [Nm]")
    axes[0, 0].grid(True)

    axes[0, 1].plot(s, speed)
    axes[0, 1].axvline(end_info["s_m"], linestyle="--")
    speed_end = interpolate_series_value_at_x(rows, "s_global_m", "vx_mps", end_info["s_m"])
    if speed_end is not None:
        axes[0, 1].scatter([end_info["s_m"]], [speed_end], s=70, zorder=5)
    axes[0, 1].set_title("Speed vs distance")
    axes[0, 1].set_xlabel("s [m]")
    axes[0, 1].set_ylabel("vx [m/s]")
    axes[0, 1].grid(True)

    axes[1, 0].plot(s_ax, ax)
    axes[1, 0].axvline(end_info["s_m"], linestyle="--")
    ax_end = interpolate_series_value_at_x(rows, "s_global_m", "ax_model_mps2", end_info["s_m"])
    if ax_end is not None:
        axes[1, 0].scatter([end_info["s_m"]], [ax_end], s=70, zorder=5)
    axes[1, 0].set_title("Acceleration vs distance")
    axes[1, 0].set_xlabel("s [m]")
    axes[1, 0].set_ylabel("ax [m/s²]")
    axes[1, 0].grid(True)

    axes[1, 1].plot(vx_p, power)
    axes[1, 1].axvline(end_info["vx_mps"], linestyle="--")
    power_end = interpolate_series_value_at_x(rows, "vx_mps", "P_total_cmd_kW", end_info["vx_mps"])
    if power_end is not None:
        axes[1, 1].scatter([end_info["vx_mps"]], [power_end], s=70, zorder=5)
    axes[1, 1].set_title("Power vs speed")
    axes[1, 1].set_xlabel("vx [m/s]")
    axes[1, 1].set_ylabel("power [kW]")
    axes[1, 1].grid(True)

    fig.suptitle(
        "Launch map overview "
        f"(initial end: vx={end_info['vx_mps']:.2f} m/s, "
        f"s={end_info['s_m']:.2f} m, reason={end_info['reason']})"
    )

    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)



def save_all_plots(rows: List[Dict], prefix: str, end_info: Dict[str, float]) -> None:
    save_overview_plot(rows, prefix + "_overview.png", end_info)
    save_initial_detection_plot(rows, prefix + "_initial_detection.png", end_info)

    save_plot(
        rows, "t_phase_s",
        [("M_total_cmd_Nm", "total"), ("M_front_total_cmd_Nm", "front"), ("M_rear_total_cmd_Nm", "rear")],
        "Launch torque vs time", "time [s]", "torque [Nm]", prefix + "_torque_vs_t.png", end_info)

    save_plot(
        rows, "vx_mps",
        [("M_total_cmd_Nm", "total"), ("M_front_total_cmd_Nm", "front"), ("M_rear_total_cmd_Nm", "rear")],
        "Launch torque vs speed", "vx [m/s]", "torque [Nm]", prefix + "_torque_vs_vx.png", end_info)

    save_plot(
        rows, "s_global_m",
        [("ax_model_mps2", "ax")],
        "Launch acceleration vs distance", "s [m]", "ax [m/s²]", prefix + "_ax_vs_s.png", end_info)

    save_plot(
        rows, "t_phase_s",
        [("ax_model_mps2", "ax")],
        "Launch acceleration vs time", "time [s]", "ax [m/s²]", prefix + "_ax_vs_t.png", end_info)

    save_plot(
        rows, "t_phase_s",
        [("P_total_cmd_kW", "P cmd"), ("P_total_act_kW", "P act")],
        "Launch power vs time", "time [s]", "power [kW]", prefix + "_power_vs_t.png", end_info)

    save_plot(
        rows, "t_phase_s",
        [("kappa_front", "front"), ("kappa_rear", "rear")],
        "Reference slip vs time", "time [s]", "kappa [-]", prefix + "_slip_vs_t.png", end_info)

    save_plot(
        rows, "t_phase_s",
        [("Fz_front_total_N", "front"), ("Fz_rear_total_N", "rear")],
        "Normal load vs time", "time [s]", "Fz [N]", prefix + "_fz_vs_t.png", end_info)

    save_plot(
        rows, "s_global_m",
        [("vx_mps", "vx")],
        "Speed vs distance", "s [m]", "vx [m/s]", prefix + "_speed_vs_s.png", end_info)


# =============================================================================
#                               MAIN
# =============================================================================

def solve_case(cfg_root: Dict, mu: float, S: float, out_dirs: Dict[str, str]) -> None:
    Xv, Uv, slack_f, slack_r, N, dt, cfg = build_and_solve_launch_ocp(cfg_root, mu, S)

    rows = [compute_row(k, Xv, Uv, slack_f, slack_r, dt, cfg, mu, S) for k in range(N + 1)]
    rows = trim_rows_to_distance(rows, S)

    end_info = detect_initial_launch_end(rows)
    annotate_rows_with_initial_end(rows, end_info)

    tag = f"{make_mu_tag(mu)}_{make_s_tag(S)}"
    csv_path = os.path.join(out_dirs["launch_runtime"], f"launch_{tag}.csv")
    info_path = os.path.join(out_dirs["info"], f"launch_{tag}_info.txt")
    plot_prefix = os.path.join(out_dirs["plots"], f"launch_{tag}")

    save_csv(rows, csv_path)
    save_info_txt(info_path, rows, end_info)
    save_all_plots(rows, plot_prefix, end_info)

    print(f"[launch-map] saved CSV: {csv_path}")
    print(f"[launch-map] saved info: {info_path}")
    print(f"[launch-map] saved overview plot: {plot_prefix}_overview.png")
    print(f"[launch-map] saved detection plot: {plot_prefix}_initial_detection.png")
    print(
        "[launch-map] initial phase end: "
        f"t={end_info['t_s']:.3f}s, "
        f"s={end_info['s_m']:.3f}m, "
        f"vx={end_info['vx_mps']:.3f}m/s, "
        f"reason={end_info['reason']}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="",
        help=(
            "Path to params.json. "
            "If omitted, uses ../../config/params.json relative to this script."
        ),
    )
    parser.add_argument("--output-root", default="", help="Override launch_map.maps_root")
    parser.add_argument("--mu", type=float, default=None, help="Generate only one mu")
    parser.add_argument("--S", type=float, default=None, help="Generate only one distance")
    args = parser.parse_args()

    config_path = os.path.abspath(args.config) if args.config else DEFAULT_CONFIG_PATH

    if not os.path.isfile(config_path):
        raise FileNotFoundError(
            "Config not found. Expected native config at: "
            f"{config_path}. You can override it with --config /path/to/params.json"
        )

    raw_cfg = load_json_config(config_path)
    cfg = get_cfg(raw_cfg)

    output_root = args.output_root
    if not output_root:
        output_root = package_relative_output_root(
            config_path,
            str(cfg["launch_map"]["maps_root"])
        )

    out_dirs = ensure_dirs(output_root)

    if args.mu is not None:
        mu_cases = [float(args.mu)]
    else:
        # Offline generator uses the explicit offline case list.
        # general.mu_x belongs to runtime/controller config and does not decide
        # what maps are generated.
        mu_cases = [
            float(x)
            for x in cfg["map_generator"]["mu_x_cases"]
        ]

        if not mu_cases:
            raise ValueError("map_generator.mu_x_cases must contain at least one value")

    S_cases = (
        [float(args.S)]
        if args.S is not None
        else [float(cfg["map_generator"]["S"])]
    )

    print(f"[launch-map] config: {config_path}")
    print(f"[launch-map] output_root: {output_root}")

    for mu in mu_cases:
        for S in S_cases:
            solve_case(cfg, mu, S, out_dirs)


if __name__ == "__main__":
    main()