# RCAIDE/Library/Methods/Powertrain/Modulators/High_Voltage_Converter/compute_hvpc_performance.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_hvpc_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_hvpc_performance(high_voltage_converter, conditions, output_power, output_voltage):
    """
    Computes bus-side power and losses of a fixed-efficiency high-voltage power converter (spec 3.3 step 9, R17).

    Parameters
    ----------
    high_voltage_converter : RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter
        Uses efficiency eta_HV, input_voltage_window and bus_voltage
    conditions : RCAIDE.Framework.Mission.Common.Results
    output_power : numpy.ndarray
        Power delivered to the electrodes P_EHD [W]
    output_voltage : numpy.ndarray
        Electrode voltage V_a [V]

    Returns
    -------
    input_power : numpy.ndarray
        Bus demand P_bus = P_EHD / eta_HV [W]

    Notes
    -----
    Stores inputs.power, outputs.power, outputs.voltage, heat (P_bus - P_EHD) and input_voltage_flag
    (1 where bus_voltage is outside input_voltage_window) in conditions.energy.modulators[tag].

    References
    ----------
    [1] He, Woolston & Perreault, IEEE COMPEL 2017 (R17).
    [2] Shevgaonkar, MIT thesis, DSpace@MIT 1721.1/163001 (2025): 85% efficiency, 1.15 kW/kg.
    """
    hvpc        = high_voltage_converter
    results     = conditions.energy.modulators[hvpc.tag]
    input_power = output_power / hvpc.efficiency

    results.outputs.power   = output_power
    results.outputs.voltage = output_voltage
    results.inputs.power    = input_power
    results.heat            = input_power - output_power
    if hvpc.bus_voltage is not None:
        V_low, V_high = hvpc.input_voltage_window
        outside = (hvpc.bus_voltage < V_low) or (hvpc.bus_voltage > V_high)
        results.input_voltage_flag = float(outside) * np.ones_like(output_power)
    return input_power
