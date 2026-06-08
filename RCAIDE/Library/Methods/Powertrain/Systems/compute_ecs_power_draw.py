# RCAIDE/Library/Methods/Powertrain/Systems/compute_ecs_power_draw.py
# 
# Created:  May 2026, M. Clarke, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports
import RCAIDE
import numpy as np

def compute_ecs_power_draw(environmental_controls,vehicle,bus,state):
    """
    Computes the power draw of an environmental control system.
    
    Parameters
    ----------
    environmental_controls : Environmental_Controls
        The ECS component object containing efficiency and thermal attributes
            - power_draw : float
                Power consumption of the environmental control component [W]
    vehicle : Vehicle()
        The vehicle object, used to extract the total passenger capacity
    bus : Electrical_Bus
        The electrical bus that powers the cabin system
    state : State
        Object containing the dynamic mission state arrays (altitude, Mach, pressure, temperature)
    
    Returns
    -------
    None
        This function modifies the environmental_controls_conditions.power array in-place.
    
    Notes
    -----
    This function dynamically calculates the ECS power draw in two parts:
    1. Electric Cabin Air Compressors: Evaluated dynamically across the mission profile 
       based on the pressure differential between the freestream ram air and the 
       required cabin pressurization schedule.
    2. Vapor Cycle Cooling: Evaluated as a steady-state load based on the coefficient 
       of performance (COP) required to dissipate passenger, system, and solar heat.
    
    For more complex environmental controls models, this function could be extended to calculate
    power draw based on operating mode, altitude, or other mission parameters.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.append_environmental_control_conditions
    """
    
    N_pax      =  vehicle.number_of_passengers

    m_dot_per_pax = 0.00416 # kg/s (0.25 kg/min per passenger)
    Q_per_pax     = 70      # W (70 W per passenger, 100 W per flight crew member, 200 W per cabin crew member)
    Q_sys_per_pax = 40      # W (IFE/Avionics/Galley heat)
    COP           = 2.5     # Coefficient of Performance for the cooling system, MEA Vapor Cycle Systems typically have a COP between 2.0 and 3.0
    Q_sun         = 1367    # W/m² (The paper uses a solar constant of 1367 W/m²)
    A_window      = 0.08    # m² (The paper assumes 0.08 m² per window)
    N_windows     = 0       # Number of Windows (Number of rows in cabin * 2)
    
    # Step 1: Compute Compressor Power
    altitude = state.conditions.freestream.altitude
    Mach     = state.conditions.freestream.mach_number        
    Cp       = state.conditions.freestream.constant_pressure_specific_heat
    T1       = state.conditions.freestream.temperature 
    P1       = state.conditions.freestream.pressure
    gamma    = state.conditions.freestream.specific_heat 
    m_dot    = m_dot_per_pax * N_pax
    eta_c    = environmental_controls.cabin_compressor_efficiency 
    
    # Compute the ram pressure at the inlet of the compressor
    P_ram    = P1 * (1 + ((gamma - 1) / 2) * Mach**2)
    
    # Determine Design Cabin Pressure based on altitude threshold
    P_cabin = np.where(
        altitude < 2438.4,       # If below 8,000 ft
        P_ram + 20000,           # Ram pressure + 0.2 bar (in Pascals)
        78000                    # Else: Constant 0.78 bar (in Pascals)
    )
    P2      = P_cabin
    
    # Calculate isentropic compressor power
    P_comp  = (m_dot * Cp * T1 / eta_c) * ((P2/P1)**((gamma-1)/gamma) - 1)
    
    # Step 2: Compute Vapor Cycle Cooling   
    Q_pax   = N_pax * Q_per_pax
    Q_sys   = N_pax * Q_sys_per_pax
    Q_solar = Q_sun * A_window * N_windows
    P_cool  = (Q_pax + Q_sys + Q_solar) / COP
      
    # Step 3: Compute total power 
    P_evs   =  P_comp +  P_cool
  
    bus_conditions                               = state.conditions.energy.busses[bus.tag]
    environmental_controls_conditions            = bus_conditions[environmental_controls.tag]    
    environmental_controls_conditions.power[:,0] = P_evs[:,0]
    bus_conditions.power_draw                   += environmental_controls_conditions.power*bus.power_split_ratio /bus.efficiency    
    
    return 