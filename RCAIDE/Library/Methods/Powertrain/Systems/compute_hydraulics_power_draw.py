# RCAIDE/Library/Methods/Powertrain/Systems/compute_hydraulics_power_draw.py
# 
# Created:  May 2026, M. Clarke, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports
import numpy as np

def compute_hydraulics_power_draw(hydraulics,vehicle,bus,state):
    """
    Computes the power draw of a hydraulic systems system.
    
    Parameters
    ----------
    hydraulics : Hydraulics
        The hydraulic system component object, containing attributes for the left, 
        right, and central redundant circuits (flowspeeds, pressures, number of pumps)
            - power_draw : float
                Power consumption of the hydraulic systems component [W]
    vehicle : Vehicle()
        The vehicle object, used to extract the Maximum Takeoff Weight (MTOW) for mass scaling
    bus : Electrical_Bus
        The electrical bus that powers the hydraulic pumps
    state : State
        Object containing the dynamic mission state arrays
    
    Returns
    -------
    None
        This function modifies the hydraulics_conditions.power array in-place.
    
    Notes
    -----
    This function calculates the electrical power required to maintain nominal 
    volumetric flow rates against system pressure for a triple-redundant hydraulic 
    architecture. 
    
    To account for varying aircraft sizes, the nominal volumetric flow rates are 
    scaled linearly based on the current aircraft's MTOW relative to a baseline 
    Airbus A320-200 reference model.
    
    For more complex hydraulic systems models, this function could be extended to calculate
    power draw based on operating mode, altitude, or other mission parameters such as angular 
    velocity and deflection angle for flight control surface actuation. Additionally, the model 
    could be expanded to include other hydraulic system architectures such as dual-redundant or 
    single hydraulic systems or pumps driven by engine shafts.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.append_hydraulics_conditions
    """
    # 1. Define The Baseline (A320-200 parameters from MDPI Paper)
    MTOW_baseline  = 75166.0 # kg
    P_res          = 3.52    # bar (Reservoir pressure)
    eta_pump       = 0.855   # 85.5% pump efficiency  
    
    # 2. Extract current aircraft MTOW and compute scaling factor
    MTOW           = vehicle.mass_properties.max_takeoff
    scaling_factor = MTOW / MTOW_baseline
    
    # 5. Compute power     
    # Scale the volumetric flow rate based on aircraft size
    V_flow_left    = (hydraulics.left_system.flowspeed    * scaling_factor) / 60000.0 # m^3/s
    V_flow_right   = (hydraulics.right_system.flowspeed   * scaling_factor) / 60000.0 # m^3/s
    V_flow_central = (hydraulics.central_system.flowspeed * scaling_factor) / 60000.0 # m^3/s
    
    # Pressure difference (System P - Reservoir P)
    delta_p_left    = (hydraulics.left_system.system_power    - P_res) * 100000.0 # Pa
    delta_p_right   = (hydraulics.right_system.system_power   - P_res) * 100000.0 # Pa
    delta_p_central = (hydraulics.central_system.system_power - P_res) * 100000.0 # Pa
    
    # Calculate mechanical pump power: P = (V_flow * Delta_P) / eta
    P_sys_left    = hydraulics.left_system.number_of_pumps    * ((V_flow_left    * delta_p_left   ) / eta_pump)
    P_sys_right   = hydraulics.right_system.number_of_pumps   * ((V_flow_right   * delta_p_right  ) / eta_pump)
    P_sys_central = hydraulics.central_system.number_of_pumps * ((V_flow_central * delta_p_central) / eta_pump)
         
    P_act                            = P_sys_left + P_sys_right  + P_sys_central  
    bus_conditions                   = state.conditions.energy.busses[bus.tag]
    hydraulics_conditions            = bus_conditions[hydraulics.tag]
    hydraulics_conditions.power[:,0] = P_act
    bus_conditions.power_draw       += hydraulics_conditions.power*bus.power_split_ratio /bus.efficiency    
    
    return 