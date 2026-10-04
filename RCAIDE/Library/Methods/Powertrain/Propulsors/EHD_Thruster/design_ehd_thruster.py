# RCAIDE/Library/Methods/Powertrain/Propulsors/EHD_Thruster/design_ehd_thruster.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core                                         import Data
from RCAIDE.Library.Methods.Powertrain                             import setup_operating_conditions
from .compute_ehd_thruster_performance                             import compute_ehd_thruster_performance

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  design_ehd_thruster
# ----------------------------------------------------------------------------------------------------------------------
def design_ehd_thruster(propulsor, design_thrust, design_velocity, design_altitude, design_throttle=1.0,
                        sizing_variable='number_of_units'):
    """
    Sizes the number of units N or the span b of an EHD thruster to deliver a required net thrust at a
    design point (spec 3.3, design_ehd_thruster; 3.7 step 2).

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster
    design_thrust : float
        Required net thrust T_net [N]
    design_velocity : float
        Freestream speed at the design point [m/s]
    design_altitude : float
        Altitude of the design point [m] (US Standard Atmosphere 1976)
    design_throttle : float, optional
        Throttle at the design point, 0-1 [-]; default 1.0 (V_a = V_max)
    sizing_variable : str, optional
        'number_of_units' (N rounded up to an integer) or 'span' (b, with N fixed). Default
        'number_of_units'.

    Returns
    -------
    design : RCAIDE.Framework.Core.Data
        number_of_units, span, unit_net_thrust [N], net_thrust [N], electrode_power [W], bus_power [W]
        at the design point. The sized attribute is written to propulsor.electrode_array.

    Notes
    -----
    Every per-unit term (electrical thrust, collector drag, wire drag) is linear in b and independent
    of N (spec 3.3.2: exact scaling), so one evaluation of the per-point model is enough. Only N and b
    may be sized here; unit_spacing and collector_chord are fixed design values (spec 3.3.1).

    Raises ValueError if the per-unit net thrust is not positive at the design point.
    """
    array = propulsor.electrode_array
    if sizing_variable not in ('number_of_units', 'span'):
        raise ValueError("sizing_variable must be 'number_of_units' or 'span'.")
    if sizing_variable == 'span' and array.number_of_units is None:
        raise ValueError("number_of_units must be set when sizing the span.")

    N_user = array.number_of_units
    array.number_of_units = 1
    state = setup_operating_conditions(propulsor, velocity_range=np.array([design_velocity]), altitude=design_altitude)
    state.conditions.energy.propulsors[propulsor.tag].throttle[:,0] = design_throttle
    compute_ehd_thruster_performance(propulsor, state)
    unit_net = state.conditions.energy.propulsors[propulsor.tag].net_thrust[0,0]
    array.number_of_units = N_user

    if unit_net <= 0:
        raise ValueError("EHD unit net thrust is not positive at the design point (electrode drag "
                         "exceeds electrical thrust, or V_a is below inception).")

    if sizing_variable == 'number_of_units':
        array.number_of_units = int(np.ceil(design_thrust / unit_net))
    else:
        array.span = array.span * design_thrust / (array.number_of_units * unit_net)

    state = setup_operating_conditions(propulsor, velocity_range=np.array([design_velocity]), altitude=design_altitude)
    state.conditions.energy.propulsors[propulsor.tag].throttle[:,0] = design_throttle
    compute_ehd_thruster_performance(propulsor, state)
    p_cond = state.conditions.energy.propulsors[propulsor.tag]

    design                 = Data()
    design.number_of_units = array.number_of_units
    design.span            = array.span
    design.unit_net_thrust = p_cond.net_thrust[0,0] / array.number_of_units
    design.net_thrust      = p_cond.net_thrust[0,0]
    design.electrode_power = p_cond.electrode_power[0,0]
    design.bus_power       = p_cond.bus_power[0,0]
    return design
