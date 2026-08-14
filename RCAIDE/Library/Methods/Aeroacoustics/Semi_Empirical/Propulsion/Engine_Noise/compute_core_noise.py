# RCAIDE/Library/Methods/Aeroacoustics/Semi_Empirical/Propulsion/Engine_Noise/compute_core_noise.py
#
# Created:  Jun 2026, M. Clarke, P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.interpolate_core_noise import create_interpolator
from RCAIDE.Library.Methods.Aeroacoustics.Common import SPL_arithmetic
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Turbofan Engine Core Noise
# ----------------------------------------------------------------------------------------------------------------------
interp_C1 = create_interpolator("table_c1")
interp_C2 = create_interpolator("table_c2")
interp_C3 = create_interpolator("table_c3")

def compute_core_noise(R_val, theta_engine, turbofan, pr, cpt, segment, frequencies):
    """
    Computes turbofan core (combustor) noise for one or more ground receptors.

    Inputs are vectorized across receptors (R_val, theta_engine) and frequency: R_val/theta_engine
    broadcast as column vectors (n_receptor, 1), frequency as a row vector (1, n_freq).

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
    cpt : int or array_like
        Control point index (or one index per receptor, matching R_val/theta_engine when
        batching multiple control points in one call) into segment.state.conditions

    References
    ----------
    Enhanced Core Noise Modeling for Turbofan Engines (NASA)
    """
    distance_microphone = np.atleast_1d(R_val).reshape(-1, 1) / Units.feet
    theta_exhaust = np.pi - np.atleast_1d(theta_engine).reshape(-1, 1)
    theta_c = np.degrees(theta_exhaust)
    freq = np.atleast_1d(frequencies).reshape(1, -1)

    # cpt may be a scalar or an array of per-receptor control-point indices; indexing with
    # np.atleast_1d(cpt) (rather than [cpt, 0]) keeps the trailing column dimension so these
    # broadcast correctly against the (n_receptor, 1) arrays below either way.
    cpt_idx = np.atleast_1d(cpt)
    converters = segment.state.conditions.energy.converters
    core_nozzle_out = converters[turbofan.core_nozzle.tag].outputs

    Velocity_primary    = core_nozzle_out.velocity[cpt_idx]
    Temperature_primary = core_nozzle_out.stagnation_temperature[cpt_idx]
    Pressure_primary    = core_nozzle_out.stagnation_pressure[cpt_idx]
    Diameter_primary    = turbofan.core_nozzle.diameter
    Num_nozzle          = turbofan.combustor.number_of_fuel_nozzle

    sound_ambient = segment.state.conditions.freestream.speed_of_sound[cpt_idx]
    pressure_amb  = segment.state.conditions.freestream.pressure[cpt_idx]
    temp_amb      = segment.state.conditions.freestream.temperature[cpt_idx]
    combustor_T_out = segment.state.conditions.energy.converters['combustor'].outputs.static_temperature[cpt_idx]
    combustor_T_in  = segment.state.conditions.energy.converters['combustor'].inputs.static_temperature[cpt_idx]
    R_gas, gamma_primary = 287.1, 1.37
    Cpp = R_gas / (1 - 1/gamma_primary)
    density_primary = Pressure_primary / (R_gas * Temperature_primary - (0.5 * R_gas * Velocity_primary**2 / Cpp))

    model_inputs = Data(
        W1 = ((np.pi * (Diameter_primary/2)**2 * Velocity_primary * density_primary) / Units.lbm),
        T_C_o = combustor_T_out * 1.8,
        T_C_i = combustor_T_in * 1.8,
        P_amb = pressure_amb / Units.psi,
        T_amb = temp_amb * 1.8,
        n_f = Num_nozzle,
        R = distance_microphone,
        D_h_1 = turbofan.core_nozzle.diameter / Units.feet,
        c_amb = sound_ambient / Units.feet,
        D_C = turbofan.combustor.diameter / Units.feet,
        c_C_o = (331.3 * (1 + ((combustor_T_out - 273)/273))**0.5) / Units.feet,
        f = freq,
        theta_c = theta_c,
        pressure_ratio = pr
    )

    core_param_log = calc_core_param(model_inputs.W1, model_inputs.T_C_o, model_inputs.T_C_i, model_inputs.pressure_ratio, model_inputs.T_amb)

    uol_c1 = calc_uol_c1(model_inputs.R, model_inputs.n_f, core_param_log)
    uol_c2 = calc_uol_c2(model_inputs.R, model_inputs.n_f, core_param_log)
    uol_c3 = calc_uol_c3(model_inputs.R, core_param_log)

    s_c1 = calc_strouhal_c1(model_inputs.f, model_inputs.D_h_1, model_inputs.c_amb)
    s_c2_c3 = calc_strouhal_c2_c3(model_inputs.f, model_inputs.D_C, model_inputs.c_C_o)

    norm_spl_c1 = get_normalized_spl(interp_C1, s_c1, model_inputs.theta_c)
    norm_spl_c2 = get_normalized_spl(interp_C2, s_c2_c3, model_inputs.theta_c)
    norm_spl_c3 = get_normalized_spl(interp_C3, s_c2_c3, model_inputs.theta_c)

    c1_arr = norm_spl_c1 + uol_c1
    c2_arr = norm_spl_c2 + uol_c2
    c3_arr = norm_spl_c3 + uol_c3

    SPL_total = 10 * np.log10(10**(c1_arr/10) + 10**(c2_arr/10) + 10**(c3_arr/10))

    core_noise = Data()
    core_noise.SPL_1_3_spectrum = np.expand_dims(SPL_total, axis=0)
    core_noise.SPL              = np.expand_dims(SPL_arithmetic(SPL_total, sum_axis=1), axis=0)
    core_noise.SPL_dBA          = np.expand_dims(SPL_arithmetic(np.atleast_2d(A_weighting_metric(SPL_total, freq.flatten())), sum_axis=1), axis=0)

    return core_noise

def get_normalized_spl(interpolator_obj, strouhal_num, theta_c):
    # Clip Strouhal numbers to prevent wild extrapolations outside empirical bounds
    clamped_strouhal = np.clip(strouhal_num, 1e-2, 1e2)
    st_bcast = np.broadcast_to(clamped_strouhal, (theta_c.shape[0], clamped_strouhal.shape[1]))

    mask = st_bcast > 0
    log_S = np.full_like(st_bcast, -10.0, dtype=float)
    log_S[mask] = np.log10(st_bcast[mask])

    Theta = np.broadcast_to(theta_c, st_bcast.shape)

    try:
        res = interpolator_obj.ev(log_S.ravel(), Theta.ravel()).reshape(st_bcast.shape)
    except AttributeError:
        res = interpolator_obj(np.column_stack((log_S.ravel(), Theta.ravel()))).reshape(st_bcast.shape)

    res[~mask] = 0.0
    return res


def calc_core_param(W1, T_C_o, T_C_i, pressure_ratio, T_amb):
    return np.log10(W1 * (((T_C_o - T_C_i) * pressure_ratio * (T_amb / T_C_i)) ** 2))

def calc_uol_c1(R, n_f, core_param_log):
    return 78.0 - (20.0 * np.log10(R)) + (7.0 * core_param_log) - (14.0 * np.log10(n_f))

def calc_uol_c2(R, n_f, core_param_log):
    return 60.3 - (20.0 * np.log10(R)) + (10.0 * core_param_log) - (18.0 * np.log10(n_f))

def calc_uol_c3(R, core_param_log):
    return 42.5 - (20.0 * np.log10(R)) + (9.0 * core_param_log)

def calc_strouhal_c1(f, D_h_1, c_amb):
    return (f * D_h_1) / c_amb

def calc_strouhal_c2_c3(f, D_C, c_C_o):
    return (f * D_C) / c_C_o
