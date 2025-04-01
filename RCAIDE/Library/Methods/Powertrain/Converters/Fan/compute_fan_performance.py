# RCAIDE/Library/Methods/Powertrain/Converters/Fan/compute_fan_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke    

# ---------------------------------------------------------------------------------------------------------------------- 
# Imports 
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Fan 
# ----------------------------------------------------------------------------------------------------------------------            
def compute_fan_performance(fan, fan_conditions, conditions):
    """
    Computes the thermodynamic properties at the exit of a fan based on pressure ratio and efficiency.
    
    Parameters
    ----------
    fan : Fan
        The fan component with the following attributes:
            - pressure_ratio : float
                Ratio of exit to inlet pressure
            - polytropic_efficiency : float
                Efficiency of the compression process
            - working_fluid : FluidModel
                Object containing methods to compute fluid properties
    fan_conditions : Conditions
        Object containing fan inlet conditions with the following attributes:
            - inputs.stagnation_temperature : numpy.ndarray
                Inlet stagnation temperature [K]
            - inputs.stagnation_pressure : numpy.ndarray
                Inlet stagnation pressure [Pa]
            - inputs.static_temperature : numpy.ndarray
                Inlet static temperature [K]
            - inputs.static_pressure : numpy.ndarray
                Inlet static pressure [Pa]
            - inputs.mach_number : numpy.ndarray
                Inlet Mach number [unitless]
    conditions : Conditions
        Object containing freestream conditions (not directly used in this function)
    
    Returns
    -------
    None
        This function modifies the fan_conditions.outputs object in-place with the following attributes:
            - stagnation_temperature : numpy.ndarray
                Exit stagnation temperature [K]
            - stagnation_pressure : numpy.ndarray
                Exit stagnation pressure [Pa]
            - static_temperature : numpy.ndarray
                Exit static temperature [K]
            - static_pressure : numpy.ndarray
                Exit static pressure [Pa]
            - stagnation_enthalpy : numpy.ndarray
                Exit stagnation enthalpy [J/kg]
            - work_done : numpy.ndarray
                Work done by the fan [J/kg]
            - mach_number : numpy.ndarray
                Exit Mach number [unitless]
    
    Notes
    -----
    This function computes the thermodynamic properties at the exit of a fan
    using gas dynamic relations and the specified pressure ratio and efficiency.
    
    **Major Assumptions**
        * Constant polytropic efficiency and pressure ratio
        * Adiabatic process
    
    **Theory**
    
    The temperature rise across the fan is calculated using:
    
    .. math::
        T_{t,out} = T_{t,in} \\cdot PR^{\\frac{\\gamma-1}{\\gamma \\cdot \\eta_{polytropic}}}
    
    The work done by the fan is:
    
    .. math::
        W = C_p \\cdot (T_{t,out} - T_{t,in})
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_BOOK/AA283_Aircraft_and_Rocket_Propulsion_BOOK_Brian_J_Cantwell_May_28_2024.pdf

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Fan.append_fan_conditions
    """        
     
    # unpack from fan
    PR                      = fan.pressure_ratio
    etapold                 = fan.polytropic_efficiency
    Tt_in                   = fan_conditions.inputs.stagnation_temperature
    Pt_in                   = fan_conditions.inputs.stagnation_pressure 
    P0                      = fan_conditions.inputs.static_pressure 
    T0                      = fan_conditions.inputs.static_temperature
    M0                      = fan_conditions.inputs.mach_number    
    
    # Unpack ram inputs
    working_fluid           = fan.working_fluid
 
    # Compute the working fluid properties 
    gamma  = working_fluid.compute_gamma(T0,P0) 
    Cp     = working_fluid.compute_cp(T0,P0)    
    
    # Compute the output quantities  
    Pt_out    = Pt_in*PR
    Tt_out    = Tt_in*PR**((gamma-1)/(gamma*etapold))
    T_out     = Tt_out/(1.+(gamma-1.)/2.*M0*M0)
    P_out     = Pt_out/((1.+(gamma-1.)/2.*M0*M0)**(gamma/(gamma-1.))) 
    ht_out    = Tt_out*Cp   
    ht_in     = Tt_in*Cp 
    M_out     = np.sqrt( (((Pt_out/P_out)**((gamma-1.)/gamma))-1.) *2./(gamma-1.) )     
    
    # Compute the work done by the fan (normalized by mass flow i.e. J/(kg/s)
    work_done = ht_out - ht_in
    
    # Store computed quantities into outputs
    fan_conditions.outputs.stagnation_temperature  = Tt_out
    fan_conditions.outputs.stagnation_pressure     = Pt_out
    fan_conditions.outputs.static_temperature      = T_out
    fan_conditions.outputs.static_pressure         = P_out    
    fan_conditions.outputs.work_done               = work_done
    fan_conditions.outputs.stagnation_enthalpy     = ht_out
    fan_conditions.outputs.mach_number             = M_out
    
    return 