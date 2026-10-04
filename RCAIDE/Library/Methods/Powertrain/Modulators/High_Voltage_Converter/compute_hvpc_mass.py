# RCAIDE/Library/Methods/Powertrain/Modulators/High_Voltage_Converter/compute_hvpc_mass.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  compute_hvpc_mass
# ----------------------------------------------------------------------------------------------------------------------
def compute_hvpc_mass(high_voltage_converter):
    """
    Computes high-voltage power converter mass from rated power and specific power (spec 3.3, Mass; R17).

    Parameters
    ----------
    high_voltage_converter : RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter
        Uses rated_power [W] and specific_power [W/kg]

    Returns
    -------
    mass : float
        m_HVPC = P_rated / specific_power [kg]; also stored in mass_properties.mass

    References
    ----------
    [1] Shevgaonkar, MIT thesis (2025), 1.15 kW/kg; He, Woolston & Perreault, IEEE COMPEL 2017, 1.2 kW/kg.
    """
    hvpc = high_voltage_converter
    if hvpc.rated_power is None:
        raise ValueError("High_Voltage_Converter.rated_power must be set to compute mass.")
    hvpc.mass_properties.mass = hvpc.rated_power / hvpc.specific_power
    return hvpc.mass_properties.mass
