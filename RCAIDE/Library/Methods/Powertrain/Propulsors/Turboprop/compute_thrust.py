# RCAIDE/Methods/Energy/Propulsors/Turboprop/compute_thrust.py
# 
# 
# Created:  Sep 2024, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports  
from RCAIDE.Framework.Core      import Units 

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_thrust
# ----------------------------------------------------------------------------------------------------------------------
def compute_thrust(turboprop,conditions):
    
    """Computes thrust and other properties as below.

    Assumptions:
    Perfect gas

    Source:
    Mattingly, Jack D.. “Elements of Gas Turbine Propulsion.” (1996).

    Inputs:
    
    
    Outputs:
    turboprop_conditions.thrust                                  
    """       
    
    g                                              = conditions.freestream.gravity                      
    T0                                             = conditions.freestream.temperature                  
    P0                                             = conditions.freestream.pressure                     
    M0                                             = conditions.freestream.mach_number                  
    a0                                             = conditions.freestream.speed_of_sound               
    V0                                             = M0 *a0  
     
    compressor                                     = turboprop.compressor
    combustor                                      = turboprop.combustor
    high_pressure_turbine                          = turboprop.high_pressure_turbine
    low_pressure_turbine                           = turboprop.low_pressure_turbine
    core_nozzle                                    = turboprop.core_nozzle  
    Tt4                                            = turboprop.combustor.turbine_inlet_temperature                                                               
    propeller_efficiency                           = turboprop.propeller_efficiency                                                                      
    gearbox_efficiency                             = turboprop.gearbox.efficiency                                                                        
    low_pressure_turbine_mechanical_efficiency     = turboprop.low_pressure_turbine.mechanical_efficiency                                                       
    lower_heating_value                            = turboprop.combustor.fuel_data.lower_heating_value 
    SFC_adjustment                                 = turboprop.specific_fuel_consumption_reduction_factor 

    # unpack component conditions
    turboprop_conditions                           = conditions.energy.propulsors[turboprop.tag] 
    core_nozzle_conditions                         = conditions.energy.converters[core_nozzle.tag] 
    compressor_conditions                          = conditions.energy.converters[compressor.tag]  
    combustor_conditions                           = conditions.energy.converters[combustor.tag]
    lpt_conditions                                 = conditions.energy.converters[low_pressure_turbine.tag]
    hpt_conditions                                 = conditions.energy.converters[high_pressure_turbine.tag] 
                                                                                                                                                                          
    # unpack from turboprop                                                                                                                                     
    fuel_to_air_ratio                              = combustor_conditions.outputs.fuel_to_air_ratio  
    turbine_cp                                     = lpt_conditions.outputs.cp                                                                                  
    turbine_gas_constant                           = lpt_conditions.outputs.gas_constant                                                                           
    compressor_cp                                  = compressor_conditions.outputs.cp                                                                        
    compressor_gas_constant                        = compressor_conditions.outputs.gas_constant                                                            
    compressor_gamma                               = compressor_conditions.outputs.gamma                                                                           
    core_exit_temperature                          = core_nozzle_conditions.outputs.static_temperature                                                          
    core_exit_pressure                             = core_nozzle_conditions.outputs.static_pressure                                                                 
    core_exit_velocity                             = core_nozzle_conditions.outputs.velocity
    
    high_pressure_turbine_temperature_ratio        = (hpt_conditions.outputs.stagnation_temperature/hpt_conditions.inputs.stagnation_temperature)                             
    low_pressure_turbine_temperature_ratio         = (lpt_conditions.outputs.stagnation_temperature/lpt_conditions.inputs.stagnation_temperature)                             
    propeller_work_output_coefficient              = propeller_efficiency*gearbox_efficiency*low_pressure_turbine_mechanical_efficiency*(1 + fuel_to_air_ratio)*(turbine_cp*Tt4)/(compressor_cp*T0)*high_pressure_turbine_temperature_ratio*(1 - low_pressure_turbine_temperature_ratio)                                  
    compressor_work_output_coefficient             = (compressor_gamma - 1)*M0*((1 + fuel_to_air_ratio)*(core_exit_velocity/a0) - M0 + (1 + fuel_to_air_ratio)*(turbine_gas_constant/compressor_gas_constant)*((core_exit_temperature/T0)/((core_exit_velocity/a0)))*((1 - (P0/core_exit_pressure))/compressor_gamma))    
    total_work_output_coefficient                  = propeller_work_output_coefficient + compressor_work_output_coefficient                                                                                              
    
    # Computing Specifc Thrust
    Fsp                                            = (total_work_output_coefficient*compressor_cp*T0)/(V0)     # [(N*s)/kg] 
    
    # Computing the TSFC
    TSFC                                           = (1 - SFC_adjustment) * (fuel_to_air_ratio/(Fsp)) * Units.hour    # [kg/(N*hr)] 
    
    W_dot_mdot0                                    = total_work_output_coefficient*compressor_cp*T0     # [(W*s)/kg] 
    
    # Computing the Power Specific Fuel Consumption
    PSFC                                           = (1 - SFC_adjustment) * (fuel_to_air_ratio/(total_work_output_coefficient*compressor_cp*T0)) * Units.hour      # [kg/(W*hr)]
    
    # Computing the Thermal Efficiency
    eta_T                                          = total_work_output_coefficient/((fuel_to_air_ratio*lower_heating_value)/(compressor_cp*T0))   # [-]
 
    # Computing the Propulsive Efficiency
    eta_P                                          = total_work_output_coefficient/((propeller_work_output_coefficient/propeller_efficiency) + ((compressor_gamma - 1)/2)*((1 + fuel_to_air_ratio)*((core_exit_velocity/a0))**2 - M0**2))                         
    
    # Computing the core mass flow
    Tref                                           = turboprop.reference_temperature
    Pref                                           = turboprop.reference_pressure
    mdhc                                           = turboprop.compressor_nondimensional_massflow    
    total_temperature_reference                    = turboprop_conditions.total_temperature_reference
    total_pressure_reference                       = turboprop_conditions.total_pressure_reference     
    mdot_core                                      = mdhc*np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref)

    # computing the dimensional thrust
    FD2                                            = Fsp*mdot_core*turboprop_conditions.throttle

    # fuel flow rate
    a                                              = np.array([0.]) 
    fuel_flow_rate                                 = np.fmax(FD2*TSFC/g,a)*1./Units.hour    

    # computing the power 
    power                                          = FD2*V0 
    
    # pack outputs 
    turboprop_conditions.thrust                            = FD2 
    turboprop_conditions.thrust_specific_fuel_consumption  = TSFC
    turboprop_conditions.non_dimensional_thrust            = Fsp 
    turboprop_conditions.core_mass_flow_rate               = mdot_core
    turboprop_conditions.fuel_flow_rate                    = fuel_flow_rate    
    turboprop_conditions.power                             = power  
    turboprop_conditions.specific_power                    = W_dot_mdot0  
    turboprop_conditions.power_specific_fuel_consumption   = PSFC 
    turboprop_conditions.thermal_efficiency                = eta_T
    turboprop_conditions.propulsive_efficiency             = eta_P

    return 