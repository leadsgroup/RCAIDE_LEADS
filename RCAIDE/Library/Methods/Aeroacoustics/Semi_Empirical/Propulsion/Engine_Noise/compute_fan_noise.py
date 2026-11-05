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
    Fan angular velocity [rad/s] and the fan total temperature rise are the per-control-point
    values solved by the turbofan cycle (conditions.energy.converters[fan.tag]); see
    compute_fan_angular_velocity.

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

    omega_fan                 = converters[turbofan.fan.tag].omega[cpt_idx]
    Temperature_total_output  = fan_out.stagnation_temperature[cpt_idx]
    Temperature_total_input   = fan_in.stagnation_temperature[cpt_idx]
    Diameter_secondary        = turbofan.diameter
    Num_blades                = turbofan.fan.number_of_blades
    if Num_blades <= 0:
        raise ValueError(f"Fan '{turbofan.fan.tag}' of propulsor '{turbofan.tag}' has no number_of_blades set; "
                         "fan noise requires the fan blade count.")

    Velocity_aircraft = segment.state.conditions.freestream.velocity[cpt_idx]
    sound_ambient     = segment.state.conditions.freestream.speed_of_sound[cpt_idx]

    # total temperature rise across the fan (Ref. NASA CR-2014-218421)
    delt_T = Temperature_total_output - Temperature_total_input
    U_tip = omega_fan * Diameter_secondary / 2.0
    M_TR = ((Velocity_aircraft**2 + U_tip**2)**0.5) / sound_ambient

    if m is None:
        # flow through the fan from the turbofan cycle, core plus bypass [lb/s]
        cycle = segment.state.conditions.energy.propulsors[turbofan.tag]
        bypass_ratio = np.broadcast_to(cycle.bypass_ratio, cycle.core_mass_flow_rate.shape)[cpt_idx]
        m = cycle.core_mass_flow_rate[cpt_idx] * (1 + bypass_ratio) / Units.lbs

    fan_inputs = Data(
        m = m,
        delta_T = delt_T * 1.8,
        M_TR = M_TR,
        RSS = 150.0,
        f_b = Num_blades * omega_fan / (2.0 * np.pi),
        M_Tip = U_tip / sound_ambient,
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

    def calc_spacing_term(inputs, multiplier):
        """F2 rotor-stator spacing term, with no inlet distortion."""
        return multiplier * np.log10(inputs.RSS / 300.0)

    def calc_broadband(inputs, f, theta, name, F1, peak_ratio):
        """Inlet or aft broadband noise, Gaussian in log frequency around peak_ratio * f_b."""
        F3 = _safe_interp_1d(get_spl_fan, interpolator_fan, name, theta)
        F5 = -0.5 * (np.log(f / (peak_ratio * inputs.f_b)) / np.log(2.2))**2
        F4 = 10 * np.log10(np.exp(F5))
        return calc_base_level(inputs) + F1 + calc_spacing_term(inputs, -5.0) + F3 + F4

    def calc_tones(inputs, f, theta, name, F1, number_of_harmonics=5):
        """Inlet or aft tones. Each harmonic of the blade passing frequency goes into its 1/3 octave band."""
        F3      = _safe_interp_1d(get_spl_fan, interpolator_fan, name, theta)
        level   = calc_base_level(inputs) + F1 + calc_spacing_term(inputs, -10.0) + F3
        cut_on  = (inputs.M_Tip / np.abs(1 - inputs.V_Number / inputs.B_Number) > 1.05) | (inputs.M_Tip > 1.05)
        energy  = np.zeros(np.broadcast(level, f).shape)
        bands   = f.ravel()
        for k in range(1, number_of_harmonics + 1):
            F4     = np.where(cut_on, 0.0, -8.0) if k == 1 else 3.0 - 3.0 * k
            nearest = np.argmin(np.abs(np.log(bands) - np.log(k * inputs.f_b)), axis=1)
            energy[np.arange(energy.shape[0]), nearest] += (10**((level + F4) / 10))[:, 0]
        return 10 * np.log10(np.maximum(energy, 1e-12))

    # all fan noise components of the modified Heidmann method (NASA CR-2014-218421). Combination tones
    # exist only at supersonic fan tip speed.
    M_TR         = fan_inputs.M_TR
    inlet_F1     = np.where(M_TR < 0.72, 34.0, 34.0 - 43.0 * (M_TR - 0.72))
    combination  = np.where(M_TR > 1.0, calc_combination_tones(fan_inputs, freq, theta_s), -100.0)
    components   = np.array([calc_broadband(fan_inputs, freq, theta_s, "Inlet Broadband", inlet_F1, 4.0),
                             calc_broadband(fan_inputs, freq, theta_s, "Aft Broadband", 34.0 - 17.0 * (M_TR - 0.65), 2.5),
                             calc_tones(fan_inputs, freq, theta_s, "Inlet Tones", 42.0 - 20.0 * M_TR),
                             calc_tones(fan_inputs, freq, theta_s, "Aft Tones", 46.0 - 20.0 * M_TR),
                             combination])
    spl = 10 * np.log10(np.sum(10**(components / 10), axis=0))
    distance_attenuated_spl = spl + 20 * np.log10(1.0 / distance_microphone)   # the method gives levels at 1 m

    # Pack into (1, n_receptor, n_freq) expected shapes
    fan_noise = Data()
    fan_noise.SPL_1_3_spectrum = np.expand_dims(distance_attenuated_spl, axis=0)
    fan_noise.SPL              = np.expand_dims(SPL_arithmetic(distance_attenuated_spl, sum_axis=1), axis=0)
    fan_noise.SPL_dBA          = np.expand_dims(SPL_arithmetic(np.atleast_2d(A_weighting_metric(distance_attenuated_spl, freq.flatten())), sum_axis=1), axis=0)

    return fan_noise
