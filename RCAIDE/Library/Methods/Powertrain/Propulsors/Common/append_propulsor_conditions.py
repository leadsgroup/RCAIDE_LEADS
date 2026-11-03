# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/append_propulsor_conditions.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Mission.Common import Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_propulsor_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_propulsor_conditions(propulsor, segment):
    """
    Creates the conditions every propulsor has, zero until computed: throttle, thrust vector angle, fuel mass flow
    rate, input and output powers by type, and the output thrust and moment vectors the network sums.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor
    segment : RCAIDE.Framework.Mission.Segments.Segment

    Returns
    -------
    propulsor_conditions : Conditions
        segment.state.conditions.energy.propulsors[propulsor.tag], for the propulsor type's own fields
    """
    ones_row                     = segment.state.ones_row
    propulsor_conditions         = Conditions()
    propulsor_conditions.inputs  = Conditions()
    propulsor_conditions.outputs = Conditions()
    for node in (propulsor_conditions.inputs, propulsor_conditions.outputs):
        node.power = Conditions()
        for power in ['propulsive', 'mechanical', 'electrical', 'chemical', 'pneumatic', 'hydraulic', 'thermal']:
            node.power[power] = 0. * ones_row(1)
    propulsor_conditions.outputs.thrust                = 0. * ones_row(3)
    propulsor_conditions.outputs.moment                = 0. * ones_row(3)
    propulsor_conditions.fuel_mass_flow_rate           = 0. * ones_row(1)
    propulsor_conditions.throttle                      = 0. * ones_row(1)
    propulsor_conditions.commanded_thrust_vector_angle = 0. * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag] = propulsor_conditions
    return propulsor_conditions
