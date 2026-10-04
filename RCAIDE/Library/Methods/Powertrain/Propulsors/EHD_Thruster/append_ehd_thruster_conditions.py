# RCAIDE/Library/Methods/Powertrain/Propulsors/EHD_Thruster/append_ehd_thruster_conditions.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Mission.Common import Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_ehd_thruster_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_ehd_thruster_conditions(propulsor, segment, energy_conditions, noise_conditions=None):
    """
    Allocates EHD thruster results and those of its electrode array and HVPC (spec 3.2 item 2, 3.3 step 10).

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster
    segment : RCAIDE.Framework.Mission.Segments.Segment
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions
    noise_conditions : RCAIDE.Framework.Mission.Common.Conditions, optional

    Returns
    -------
    None
        Creates energy_conditions.propulsors[tag] with throttle, commanded_thrust_vector_angle,
        thrust (3), moment (3), power, net_thrust, electrode_power (P_EHD), bus_power (P_bus) and
        thrust_to_power_ratio, then calls append_operating_conditions on each sub-component.
    """
    ones_row = segment.state.ones_row
    p = Conditions()
    p.throttle                      = 0. * ones_row(1)
    p.commanded_thrust_vector_angle = 0. * ones_row(1)
    p.thrust                        = 0. * ones_row(3)
    p.moment                        = 0. * ones_row(3)
    p.power                         = 0. * ones_row(1)
    p.net_thrust                    = 0. * ones_row(1)
    p.electrode_power               = 0. * ones_row(1)
    p.bus_power                     = 0. * ones_row(1)
    p.thrust_to_power_ratio         = 0. * ones_row(1)
    energy_conditions.propulsors[propulsor.tag] = p
    if noise_conditions is not None:
        noise_conditions.propulsors[propulsor.tag] = Conditions()

    for tag, item in propulsor.items():
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment, energy_conditions, noise_conditions)
    return
