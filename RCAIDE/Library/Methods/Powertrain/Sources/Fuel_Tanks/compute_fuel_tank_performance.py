# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/compute_fuel_tank_performance.py
#
#
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# package imports
import numpy as np
# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def compute_fuel_tank_performance(tank,state,distributor):
    """
    Computes the fuel consumption of a fuel tank, working backward from its assigned
    distributor's already-computed chemical power demand.

    Parameters
    ----------
    tank : RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank
        Fuel tank component with the following attributes:
            - fuel : RCAIDE.Library.Attributes.Propellants.Propellant
                Propellant stored in the tank
            - assigned_distributors : list
                Distributor(s) this tank supplies, e.g. [[fuel_line.tag]]
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state
    distributor : RCAIDE.Framework.Networks.Network
        The network this tank belongs to (unused directly; the tank's assigned
        distributor is looked up from state.conditions.energy.distributors instead)

    Returns
    -------
    inputs : Conditions
        Tank input conditions
    outputs : Conditions
        Tank output conditions (power.chemical, the chemical power drawn out as fuel)
    stored_results_flag : bool
        Flag indicating if results are stored
    stored_source_tag : str
        Tag of the tank with stored results

    Notes
    -----
    The fuel mass flow rate is found from the tank's assigned distributor's chemical
    power demand (already computed, since distributors run after sources in
    ``Network.evaluate()``) divided by the fuel's lower heating value, split between
    multiple tanks on the same distributor by ``power_split_ratio``. Remaining fuel
    mass is then integrated over the mission using ``state.numerics.time.integrate``.

    Cryogenic_Tank overrides ``compute_performance`` entirely (see
    ``compute_cryogenic_tank_performance``) rather than going through this function,
    since its boil-off physics are solved as implicit mission unknowns rather than by
    explicit forward integration.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank
    """
    # unpack  
    I    = state.numerics.time.integrate
    fuel = tank.fuel
     
    tank_conditions     = state.conditions.energy.sources[tank.tag] 
             
    chemical_power      = tank_conditions.power_split_ratio * state.conditions.energy.distributors[tank.assigned_distributors[0][0]].outputs.power.chemical
    fuel_mass_flow_rate = chemical_power / tank.fuel.lower_heating_value

    m_0_fuel                               = state.conditions.weights.components.mass[fuel.tag][0,0]
    total_mass_flow_rate                   = fuel_mass_flow_rate + tank_conditions.boil_off_flow_rate +  tank_conditions.secondary_mass_flow_rate
    tank_conditions.outputs.power.chemical = total_mass_flow_rate * tank.fuel.lower_heating_value

    net_mass_flow_rate                     = total_mass_flow_rate - tank_conditions.refuel_mass_flow_rate
    tank_conditions.mass_flow_rate         = net_mass_flow_rate

    if len(net_mass_flow_rate) > 1:
        fuel_mass     = m_0_fuel + np.dot(I, -net_mass_flow_rate).flatten()
        refuel_target = tank_conditions.refuel_target_mass[0,0]
        if np.isfinite(refuel_target) and m_0_fuel < refuel_target:
            fuel_mass = np.minimum(fuel_mass, refuel_target)
        tank_conditions.fuel_mass[:,0] = fuel_mass

    stored_results_flag            = True
    stored_source_tag              = tank.tag   
    return tank_conditions.inputs, tank_conditions.outputs, stored_results_flag, stored_source_tag
 