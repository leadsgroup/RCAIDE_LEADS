# RCAIDE/Library/Methods/Powertrain/Propulsors/EHD_Thruster/compute_ehd_thruster_mass.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core                                               import Data
from RCAIDE.Library.Methods.Powertrain.Converters.EHD_Electrode_Array    import compute_electrode_array_mass
from RCAIDE.Library.Methods.Powertrain.Modulators.High_Voltage_Converter import compute_hvpc_mass

# ----------------------------------------------------------------------------------------------------------------------
#  compute_ehd_thruster_mass
# ----------------------------------------------------------------------------------------------------------------------
def compute_ehd_thruster_mass(propulsor):
    """
    Computes the mass of an EHD thruster (spec 3.3, Mass).

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster

    Returns
    -------
    breakdown : RCAIDE.Framework.Core.Data
        wire, collector, high_voltage_converter, additional and total mass [kg]. The total is also
        stored in propulsor.mass_properties.mass.

    Notes
    -----
    m_HVPC = P_rated/specific_power; m_wire = N·rho_w·pi·a^2·b;
    m_collector = N·b·(rho_f·0.0685c^2 + sigma_foil·2.029c); spacers and wiring are the user-supplied
    additional_mass lump.
    """
    wire, collector = compute_electrode_array_mass(propulsor.electrode_array)
    hvpc            = compute_hvpc_mass(propulsor.high_voltage_converter)
    breakdown                        = Data()
    breakdown.wire                   = wire
    breakdown.collector              = collector
    breakdown.high_voltage_converter = hvpc
    breakdown.additional             = propulsor.additional_mass
    breakdown.total                  = wire + collector + hvpc + propulsor.additional_mass
    propulsor.mass_properties.mass   = breakdown.total
    return breakdown
