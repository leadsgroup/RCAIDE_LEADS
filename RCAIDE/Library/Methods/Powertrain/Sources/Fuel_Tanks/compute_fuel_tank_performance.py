# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/compute_fuel_tank_performance.py
#
#
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE

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

    For Cryogenic_Tank tanks (regardless of which cryogenic propellant they hold --
    e.g. Liquid_Hydrogen or Liquid_Natural_Gas), a boil-off mass flow rate is also
    added to account for heat leak into the tank. This boil-off calculation is
    currently a placeholder: convective/radiative heat transfer coefficients and the
    latent heat of vaporization are hardcoded to 0, so ``m_dot_boil_off`` always
    evaluates to 0 until these are implemented.

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
    
    if isinstance(tank, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank):
        # Cryogenic boil-off mass flow rate from heat leak into the tank. Not yet
        # implemented -- h (convection coefficient), epsilon (emissivity), sigma
        # (Stefan-Boltzmann constant), and h_fg (latent heat of vaporization) are
        # not yet sourced from the tank/fuel, so boil-off is fixed at 0. Intended
        # physics, to be implemented in a future PR:
        #   T_amb        = state.conditions.freestream.temperature
        #   T_s          = tank_conditions.surface_temperature
        #   Q_convection = h * (T_amb - T_s)
        #   Q_radiation  = epsilon * sigma * (T_amb**4 - T_s**4)
        #   m_dot_boil_off = (Q_convection + Q_radiation) / h_fg
        m_dot_boil_off = 0
        tank_conditions.boil_off_flow_rate = m_dot_boil_off
     
    m_0_fuel                               = state.conditions.weights.components.mass[fuel.tag][0,0]  
    total_mass_flow_rate                   = fuel_mass_flow_rate + tank_conditions.boil_off_flow_rate +  tank_conditions.secondary_mass_flow_rate             
    tank_conditions.mass_flow_rate         = total_mass_flow_rate
    tank_conditions.outputs.power.chemical = total_mass_flow_rate * tank.fuel.lower_heating_value
    
    if len(total_mass_flow_rate) > 1: 
        tank_conditions.fuel_mass[:,0]  = m_0_fuel +  np.dot(I, -total_mass_flow_rate).flatten()  

    stored_results_flag            = True
    stored_source_tag              = tank.tag   
    return tank_conditions.inputs, tank_conditions.outputs, stored_results_flag, stored_source_tag
 