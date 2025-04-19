# RCAIDE/Library/Methods/Powertrain/Converters/Compressor/compute_compressor_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
# Imports 
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
# compute_compression_nozzle_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_compressor_performance(compressor,compressor_conditions,conditions):
    """ Computes the performance of a compressor bases on its polytropic efficiency.
        The following properties are computed: 
       compressor.outputs.
         stagnation_temperature  (numpy.ndarray): exit stagnation_temperature   [K]  
         stagnation_pressure     (numpy.ndarray): exit stagnation_pressure      [Pa]
         stagnation_enthalpy     (numpy.ndarray): exit stagnation_enthalpy      [J/kg]
         work_done               (numpy.ndarray): work done                     [J/kg] 

    Assumptions:
        Constant polytropic efficiency and pressure ratio

    Source:
        https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/

    Args:
        conditions.freestream.
          isentropic_expansion_factor         (numpy.ndarray): isentropic_expansion_factor        [unitless]
          specific_heat_at_constant_pressure  (numpy.ndarray): specific_heat_at_constant_pressure [J/(kg K)]
        compressor.
           inputs.stagnation_temperature      (numpy.ndarray): entering stagnation temperature [K]
           inputs.stagnation_pressure         (numpy.ndarray): entering stagnation pressure    [Pa] 
           pressure_ratio                             (float): pressure ratio                  [unitless]
           polytropic_efficiency                      (float): polytropic efficiency           [unitless]

    Returns:
        None 
    """          
    
    # Unpack component inputs
    Tt_in    = compressor_conditions.inputs.stagnation_temperature
    Pt_in    = compressor_conditions.inputs.stagnation_pressure 
    PR       = compressor.pressure_ratio
    etapold  = compressor.polytropic_efficiency 
    T0       = compressor_conditions.inputs.static_temperature
    P0       = compressor_conditions.inputs.static_pressure  
    M0       = compressor_conditions.inputs.mach_number    
    
    # Unpack ram inputs
    working_fluid           = compressor.working_fluid
 
    # Compute the working fluid properties 
    gamma  = working_fluid.compute_gamma(T0,P0) 
    Cp     = working_fluid.compute_cp(T0,P0)    
    R      = working_fluid.compute_R(T0,P0)
        
    # Compute the output properties based on the pressure ratio of the component
    ht_in     = Tt_in*Cp 
    Pt_out    = Pt_in*PR
    Tt_out    = Tt_in*(PR**((gamma-1)/(gamma*etapold)))
    ht_out    = Tt_out*Cp
    T_out     = Tt_out/(1.+(gamma-1.)/2.*M0*M0)
    P_out     = Pt_out/((1.+(gamma-1.)/2.*M0*M0)**(gamma/(gamma-1.))) 
    M_out     = np.sqrt( (((Pt_out/P_out)**((gamma-1.)/gamma))-1.) *2./(gamma-1.) ) 
    # Compute the work done by the compressor (normalized by mass flow i.e. J/(kg/s)
    work_done = ht_out - ht_in
    
    # Pack results  
    compressor_conditions.outputs.work_done               = work_done 
    compressor_conditions.outputs.stagnation_temperature  = Tt_out
    compressor_conditions.outputs.stagnation_pressure     = Pt_out
    compressor_conditions.outputs.stagnation_enthalpy     = ht_out
    compressor_conditions.outputs.static_temperature      = T_out
    compressor_conditions.outputs.static_pressure         = P_out 
    compressor_conditions.outputs.mach_number             = M_out
    compressor_conditions.outputs.gas_constant            = R
    compressor_conditions.outputs.gamma                   = gamma 
    compressor_conditions.outputs.cp                      = Cp   
    
    return 

