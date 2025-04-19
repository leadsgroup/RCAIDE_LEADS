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
def compute_fan_performance(fan,fan_conditions,conditions):
    """ This computes the output values from the input values according to
    equations from the source. The following outputs are computed: 
    fan_conditions.outputs.
      stagnation_temperature  (numpy.ndarray): exit stagnation_temperature  [K]  
      stagnation_pressure     (numpy.ndarray): exit stagnation_pressure     [Pa]
      stagnation_enthalpy     (numpy.ndarray): exit stagnation_enthalpy     [J/kg]
      work_done               (numpy.ndarray): work_done                    [J/(kg-s)]
 

    Assumptions:
        Constant polytropic efficiency and pressure ratio

    Source:
        https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/

    Args:
        conditions.freestream.
          isentropic_expansion_factor         (numpy.ndarray): isentropic_expansion_factor         [unitless]
          specific_heat_at_constant_pressure  (numpy.ndarray): specific_heat_at_constant_pressure  [J/(kg K)]
        fan
          .inputs.stagnation_temperature      (numpy.ndarray): entering stagnation temperature [K]
          .inputs.stagnation_pressure         (numpy.ndarray): entering stagnation pressure    [Pa] 
          .pressure_ratio                             (float): pressure ratio of fan           [unitless]
          .polytropic_efficiency                      (float): polytropic_efficiency           [unitless]
      
    Returns:
        None
        
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