# RCAIDE/Methods/Powertrain/Modulators/inverter/compute_inverter_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_inverter_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_inverter_performance(inverter, state):

    inverter_conditions = state.segment.conditions.energy.modulators[inverter.tag]

    Vdc        = float(inverter.inputs.dc_voltage)         # [V]
    eta_inv    = float(inverter.inputs.efficiency)         # [-] 0<eta<=1
    f_out      = float(inverter.inputs.frequency)          # [Hz]
    Vph_sp     = float(inverter.inputs.per_phase_setpoint) # [V_rms] per-phase setpoint
    Zp_in      = inverter.inputs.per_phase_load_impedance  # [ohm] per-phase load impedance (can be complex)`

    Z_phase = Zp_in if isinstance(Zp_in, complex) else complex(float(Zp_in), 0.0)

    # ---- Modulation & synthesized voltages (fundamental only) ----
    m_target = (2 * np.sqrt(2) * Vph_sp) / Vdc
    m = np.clip(m_target, 0.0, 1.0)
    modulation_limited = (m_target > 1.0)

    fs = 200_000
    t  = np.arange(0, 3 / f_out, 1 / fs)
    w  = 2 * np.pi * f_out

    Va = (m * Vdc / 2) * np.sin(w * t)
    Vb = (m * Vdc / 2) * np.sin(w * t - 2 * np.pi / 3)
    Vc = (m * Vdc / 2) * np.sin(w * t + 2 * np.pi / 3)

    Vph_rms = np.sqrt(np.mean(Va**2))
    Vll_rms = np.sqrt(np.mean((Va - Vb)**2))  

    # ---- Load currents & power ----
    Iph_rms = Vph_rms / abs(Z_phase)             # [A_rms] magnitude
    Y = 1 / Z_phase
    G, B = np.real(Y), np.imag(Y)                # conductance & susceptance

    P_phase = Vph_rms**2 * G                     # [W] per-phase real power
    Q_phase = -Vph_rms**2 * B                    # [var] sign: +inductive (lag), -capacitive (lead)

    P_out = 3 * P_phase                          # [W]
    Q_out = 3 * Q_phase                          # [var]
    S_phase = Vph_rms * Iph_rms                  # [VA]
    S_out   = 3 * S_phase                        # [VA]
    pf = 0.0 if S_phase == 0 else P_phase / S_phase

    # ---- DC side ----
    P_in = P_out / eta_inv                       # [W]
    Idc  = P_in / Vdc                            # [A]

    # ---- Output ----
    inverter_conditions.outputs.Vph_rms               = Vph_rms
    inverter_conditions.outputs.Vll_rms               = Vll_rms
    inverter_conditions.outputs.Iph_rms               = Iph_rms
    inverter_conditions.outputs.P_out                 = P_out
    inverter_conditions.outputs.Q_out                 = Q_out
    inverter_conditions.outputs.S_out                 = S_out
    inverter_conditions.outputs.pf                    = pf
    inverter_conditions.outputs.P_in                  = P_in
    inverter_conditions.outputs.Idc                   = Idc
    inverter_conditions.outputs.m                     = m
    inverter_conditions.outputs.m_target              = m_target
    inverter_conditions.outputs.modulation_limited    = modulation_limited
    inverter_conditions.outputs.f_out                 = f_out

    stored_results_flag            = True
    stored_modulator_tag           = inverter.tag  

    inverter_conditions.power.propulsive               = 0.0 * state.ones_row(1)
    inverter_conditions.power.mechanical               = 0.0 * state.ones_row(1)
    inverter_conditions.power.electrical               = P_out
    inverter_conditions.power.chemical                 = 0.0 * state.ones_row(1)
    inverter_conditions.power.pneumatic                = 0.0 * state.ones_row(1)
    inverter_conditions.power.hydraulic                = 0.0 * state.ones_row(1)
    inverter_conditions.power.thermal                  = 0.0 * state.ones_row(1)

    return  inverter_conditions.power, stored_results_flag, stored_modulator_tag
