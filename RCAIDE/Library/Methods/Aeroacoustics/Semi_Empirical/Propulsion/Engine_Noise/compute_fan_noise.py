# RCAIDE/Library/Methods/Aeroacoustics/Semi_Empirical/Propulsion/Engine_Noise/compute_fan_noise.py
#
# Created:  Jul 2026, P. Siripun, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.interpolate_fan_noise import get_spl_fan, create_interpolator_fan
from RCAIDE.Library.Methods.Aeroacoustics.Common import SPL_arithmetic
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Safe Interpolator Wrapper (protects against strict scalar-only RCAIDE interpolators)
# ----------------------------------------------------------------------------------------------------------------------
def _safe_interp_1d(interp_func, interp_obj, name, x):
    """Safely evaluates a 1D interpolator over an N-D array."""
    try:
        res = interp_func(interp_obj, name, x)
        if isinstance(res, np.ndarray) and res.shape == x.shape:
            return res
    except:
        pass
    # Fallback if the specific RCAIDE interpolator rejects arrays
    res = np.zeros_like(x, dtype=float)
    for idx, val in np.ndenumerate(x):
        res[idx] = interp_func(interp_obj, name, val)
    return res

# ----------------------------------------------------------------------------------------------------------------------
#  Turbofan Fan Noise
# ----------------------------------------------------------------------------------------------------------------------
interpolator_fan = create_interpolator_fan()

def compute_fan_noise(R_val, theta_engine, turbofan, m, cpt, segment, frequencies):
    """
    Computes turbofan fan (combination tone) noise for one or more ground receptors.

    Inputs are vectorized across receptors (R_val, theta_engine) and frequency: R_val/theta_engine
    broadcast as column vectors (n_receptor, 1), frequency as a row vector (1, n_freq).

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
    cpt : int or array_like
        Control point index (or one index per receptor, matching R_val/theta_engine when
        batching multiple control points in one call) into segment.state.conditions

    Notes
    -----
    Fan diameter and blade count have no dedicated fan-level design attribute in the current
    turbofan cycle model; turbofan.diameter (engine/nacelle diameter) is used as a stand-in for
    fan face diameter, and fan.number_of_blades must be set on the vehicle's Fan component.
    Fan angular velocity falls back to fan.design_angular_velocity when the operating value
    hasn't been solved for (compute_fan_performance does not currently update it).

    References
    ----------
    Enhanced Fan Noise Modeling for Turbofan Engines (NASA)
    """
    distance_microphone = np.atleast_1d(R_val).reshape(-1, 1)
    theta_s = np.degrees(np.atleast_1d(theta_engine).reshape(-1, 1))
    freq = np.atleast_1d(frequencies).reshape(1, -1)

    # cpt may be a scalar or an array of per-receptor control-point indices; indexing with
    # np.atleast_1d(cpt) (rather than [cpt, 0]) keeps the trailing column dimension so these
    # broadcast correctly against the (n_receptor, 1) arrays below either way.
    cpt_idx    = np.atleast_1d(cpt)
    converters = segment.state.conditions.energy.converters
    fan_in     = converters[turbofan.fan.tag].inputs
    fan_out    = converters[turbofan.fan.tag].outputs
    fan_nozzle_out = converters[turbofan.fan_nozzle.tag].outputs

    N1                        = turbofan.fan.angular_velocity if turbofan.fan.angular_velocity > 0 else turbofan.fan.design_angular_velocity
    Velocity_secondary        = fan_nozzle_out.velocity[cpt_idx]
    Temperature_secondary     = fan_nozzle_out.stagnation_temperature[cpt_idx]
    Temperature_static_output = fan_out.static_temperature[cpt_idx]
    Temperature_static_input  = fan_in.static_temperature[cpt_idx]
    Pressure_secondary        = fan_nozzle_out.stagnation_pressure[cpt_idx]
    Diameter_secondary        = turbofan.diameter
    Num_blades                = turbofan.fan.number_of_blades

    Velocity_aircraft = segment.state.conditions.freestream.velocity[cpt_idx]
    sound_ambient     = segment.state.conditions.freestream.speed_of_sound[cpt_idx]
    R_gas, gamma = 287.1, 1.4
    Cp = R_gas / (1 - 1/gamma)

    density_secondary = Pressure_secondary / (R_gas * (Temperature_secondary - (0.5 * Velocity_secondary**2 / Cp)))
    delt_T = Temperature_static_output - Temperature_static_input
    M_TR = ((Velocity_aircraft**2 + ((np.pi*Diameter_secondary*N1)/60)**2)**0.5) / sound_ambient

    if m is None:
        # Annular secondary (bypass) flow area, subtracting the core nozzle area
        Diameter_primary = turbofan.core_nozzle.diameter
        Area_secondary = (np.pi / 4.0) * (Diameter_secondary**2 - Diameter_primary**2)
        m = (Area_secondary * Velocity_secondary * density_secondary) / Units.lbs

    fan_inputs = Data(
        m = m,
        delta_T = delt_T * 1.8,
        M_TR = M_TR,
        RSS = 150.0,
        f_b = (N1 * Num_blades * Diameter_secondary) / Units.minute,
        M_Tip = (np.pi * Diameter_secondary * N1 * (1/60)) / (sound_ambient),
        V_Number = 54,
        B_Number = Num_blades,
        inlet_distortion = False
    )

    def calc_base_level(inputs):
        # np.log10 (not math.log10): m/delta_T are per-receptor arrays once multiple control
        # points are batched into one call
        return 10 * np.log10(inputs.m / 1.0) + 40 * np.log10(inputs.delta_T / 1.0)

    def calc_combination_tones(inputs, f, theta):
        """Combination tone noise, valid for supersonic fan tip speeds."""
        F3 = _safe_interp_1d(get_spl_fan, interpolator_fan, "Combination Tones", theta)
        F2 = 0
        M_TR, f_b = inputs.M_TR, inputs.f_b
        base_spl = calc_base_level(inputs)

        F1_half = np.where(M_TR < 1.146, -18 + 46.5*(M_TR - 1)/0.146, 28.5 - 12*(M_TR - 1.146)/0.854)
        ratio = np.maximum(f / (0.5 * f_b), 1e-12)
        F4_half = np.where(f < 0.5 * f_b, 20 * np.log10(ratio), -20 * np.log10(ratio))
        spl_half = base_spl + F1_half + F2 + F3 + F4_half

        F1_quarter = np.where(M_TR < 1.322, -15 + 47.5*(M_TR - 1)/0.322, 32.5 - 9*(M_TR - 1.322)/0.678)
        F4_quarter = np.where(f < 0.25 * f_b, 30 * np.log10(f / (0.25 * f_b)), -30 * np.log10(f / (0.25 * f_b)))
        spl_quarter = base_spl + F1_quarter + F2 + F3 + F4_quarter

        F1_eighth = np.where(M_TR < 1.61, -12 + 41.2*(M_TR - 1)/0.61, 29.2 - 4.7*(M_TR - 1.61)/0.39)
        F4_eighth = np.where(f < 0.125 * f_b, 30 * np.log10(f / (0.125 * f_b)), -20 * np.log10(f / (0.125 * f_b)))
        spl_eighth = base_spl + F1_eighth + F2 + F3 + F4_eighth

        return 10 * np.log10(10**(spl_half/10) + 10**(spl_quarter/10) + 10**(spl_eighth/10))

    spl = calc_combination_tones(fan_inputs, freq, theta_s)
    distance_attenuated_spl = spl + 20 * np.log10(0.25 / distance_microphone)

    # Pack into (1, n_receptor, n_freq) expected shapes
    fan_noise = Data()
    fan_noise.SPL_1_3_spectrum = np.expand_dims(distance_attenuated_spl, axis=0)
    fan_noise.SPL              = np.expand_dims(SPL_arithmetic(distance_attenuated_spl, sum_axis=1), axis=0)
    fan_noise.SPL_dBA          = np.expand_dims(SPL_arithmetic(np.atleast_2d(A_weighting_metric(distance_attenuated_spl, freq.flatten())), sum_axis=1), axis=0)

    return fan_noise
