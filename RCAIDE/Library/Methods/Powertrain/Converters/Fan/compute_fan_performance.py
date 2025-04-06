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
def compute_fan_performance(fan, conditions):
    """
    Computes the thermodynamic performance of a fan in a gas turbine engine.
    
    Parameters
    ----------
    fan : RCAIDE.Library.Components.Converters.Fan
        Fan component with the following attributes:
            - tag : str
                Identifier for the fan
            - working_fluid : Data
                Working fluid properties object
            - pressure_ratio : float
                Pressure ratio across the fan
            - polytropic_efficiency : float
                Polytropic efficiency of the compression process
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - energy : Data
                Energy conditions
                - converters : dict
                    Converter energy conditions indexed by tag
                    - inputs : Data
                        Input conditions
                        - stagnation_temperature : numpy.ndarray
                            Stagnation temperature at fan inlet [K]
                        - stagnation_pressure : numpy.ndarray
                            Stagnation pressure at fan inlet [Pa]
                        - static_pressure : numpy.ndarray
                            Static pressure at fan inlet [Pa]
                        - static_temperature : numpy.ndarray
                            Static temperature at fan inlet [K]
                        - mach_number : numpy.ndarray
                            Mach number at fan inlet
    
    Returns
    -------
    None
        Results are stored in conditions.energy.converters[fan.tag].outputs:
            - stagnation_temperature : numpy.ndarray
                Stagnation temperature at fan exit [K]
            - stagnation_pressure : numpy.ndarray
                Stagnation pressure at fan exit [Pa]
            - static_temperature : numpy.ndarray
                Static temperature at fan exit [K]
            - static_pressure : numpy.ndarray
                Static pressure at fan exit [Pa]
            - stagnation_enthalpy : numpy.ndarray
                Stagnation enthalpy at fan exit [J/kg]
            - work_done : numpy.ndarray
                Work done by the fan [J/kg]
            - mach_number : numpy.ndarray
                Mach number at fan exit
    
    Notes
    -----
    This function computes the thermodynamic properties at the fan exit based on
    the inlet conditions and fan characteristics. It calculates the temperature rise,
    pressure rise, and work done by the fan during the compression process.
    
    The computation follows these steps:
        1. Extract inlet conditions (temperature, pressure, Mach number)
        2. Compute working fluid properties (gamma, Cp)
        3. Calculate stagnation pressure at exit using pressure ratio
        4. Compute stagnation temperature at exit using polytropic efficiency
        5. Calculate static temperature and pressure at exit based on exit Mach number
        6. Compute stagnation enthalpy at inlet and exit
        7. Calculate work done by the fan (exit - inlet stagnation enthalpy)
        8. Store all results in the conditions data structure
    
    **Major Assumptions**
        * Constant polytropic efficiency and pressure ratio
        * Mach number is preserved from inlet to exit
    
    **Theory**
    The stagnation temperature ratio across the fan is related to the pressure ratio by:
    
    .. math::
        \\frac{T_{t,out}}{T_{t,in}} = \\left(\\frac{P_{t,out}}{P_{t,in}}\\right)^{\\frac{\\gamma-1}{\\gamma \\eta_{p}}}
    
    where:
      - T_{t,out} is the exit stagnation temperature
      - T_{t,in} is the inlet stagnation temperature
      - P_{t,out} is the exit stagnation pressure
      - P_{t,in} is the inlet stagnation pressure
      - γ is the ratio of specific heats
      - η_{p} is the polytropic efficiency
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Compressor.compute_compressor_performance
    """        
     
    # unpack from fan
    PR                      = fan.pressure_ratio
    etapold                 = fan.polytropic_efficiency
    fan_conditions          = conditions.energy.converters[fan.tag]
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