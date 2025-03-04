# RCAIDE/Methods/Energy/Propulsors/Turbofan_Propulsor/compute_turbofan_performance.py
# 
# 
# Created:  Jul 2024, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core import Data   
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor          import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor         import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Fan                import compute_fan_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine            import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle   import compute_expansion_nozzle_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan_Propulsor            import compute_thrust

import  numpy as  np
from copy import  deepcopy

# ----------------------------------------------------------------------------------------------------------------------
# compute_performance
# ----------------------------------------------------------------------------------------------------------------------   
def compute_turbofan_performance(turbofan,state,center_of_gravity= [[0.0, 0.0,0.0]]):
    ''' Computes the perfomrance of one turbofan
    
    Assumptions: 
    N/A

    Source:
    N/A
    
    Inputs:  
    turbofan             - turbofan data structure               [-]  
    state                - operating conditions data structure   [-]  
    fuel_line            - fuelline                              [-]
    center_of_gravity    - aircraft center of gravity            [m]

    Outputs:  
    total_thrust         - thrust of turbofan group              [N]
    total_momnet         - moment of turbofan group              [Nm]
    total_power          - power of turbofan group               [W] 
    stored_results_flag  - boolean for stored results            [-]     
    stored_propulsor_tag - name of turbojet with stored results  [-]
    
    Properties Used: 
    N.A.        
    ''' 
    conditions                = state.conditions   
    noise_conditions          = conditions.noise[turbofan.tag] 
    turbofan_conditions       = conditions.energy[turbofan.tag]  
    U0                        = conditions.freestream.velocity
    T                         = conditions.freestream.temperature
    P                         = conditions.freestream.pressure
    ram                       = turbofan.ram
    inlet_nozzle              = turbofan.inlet_nozzle
    fan                       = turbofan.fan
    low_pressure_compressor   = turbofan.low_pressure_compressor
    high_pressure_compressor  = turbofan.high_pressure_compressor
    combustor                 = turbofan.combustor
    high_pressure_turbine     = turbofan.high_pressure_turbine
    low_pressure_turbine      = turbofan.low_pressure_turbine
    core_nozzle               = turbofan.core_nozzle
    fan_nozzle                = turbofan.fan_nozzle 
    bypass_ratio              = turbofan.bypass_ratio
    
    # unpack component conditions 
    ram_conditions          = turbofan_conditions[ram.tag]    
    inlet_nozzle_conditions = turbofan_conditions[inlet_nozzle.tag]
    fan_conditions          = turbofan_conditions[fan.tag]    
    lpc_conditions          = turbofan_conditions[low_pressure_compressor.tag]
    hpc_conditions          = turbofan_conditions[high_pressure_compressor.tag]
    combustor_conditions    = turbofan_conditions[combustor.tag]     
    lpt_conditions          = turbofan_conditions[low_pressure_turbine.tag]
    hpt_conditions          = turbofan_conditions[high_pressure_turbine.tag]
    core_nozzle_conditions  = turbofan_conditions[core_nozzle.tag]
    fan_nozzle_conditions   = turbofan_conditions[fan_nozzle.tag]    
 
    # Set the working fluid to determine the fluid properties
    ram.working_fluid = turbofan.working_fluid

    # Flow through the ram , this computes the necessary flow quantities and stores it into conditions
    compute_ram_performance(ram,ram_conditions,conditions)

    # Link inlet nozzle to ram 
    inlet_nozzle_conditions.inputs.stagnation_temperature             = ram_conditions.outputs.stagnation_temperature 
    inlet_nozzle_conditions.inputs.stagnation_pressure                = ram_conditions.outputs.stagnation_pressure
    inlet_nozzle_conditions.inputs.static_temperature                 = ram_conditions.outputs.static_temperature
    inlet_nozzle_conditions.inputs.static_pressure                    = ram_conditions.outputs.static_pressure
    inlet_nozzle_conditions.inputs.mach_number                        = ram_conditions.outputs.mach_number
    inlet_nozzle.working_fluid                                        = ram.working_fluid

    # Flow through the inlet nozzle
    compute_compression_nozzle_performance(inlet_nozzle,inlet_nozzle_conditions,conditions)
    
    # Link the fan to the inlet nozzle
    fan_conditions.inputs.stagnation_temperature                      = inlet_nozzle_conditions.outputs.stagnation_temperature
    fan_conditions.inputs.stagnation_pressure                         = inlet_nozzle_conditions.outputs.stagnation_pressure
    fan_conditions.inputs.static_temperature                          = inlet_nozzle_conditions.outputs.static_temperature
    fan_conditions.inputs.static_pressure                             = inlet_nozzle_conditions.outputs.static_pressure
    fan_conditions.inputs.mach_number                                 = inlet_nozzle_conditions.outputs.mach_number  
    fan.working_fluid                                                 = turbofan.working_fluid
    
    # Flow through the fan
    compute_fan_performance(fan,fan_conditions,conditions)    

    # Link low pressure compressor to the inlet nozzle
    lpc_conditions.inputs.stagnation_temperature  = fan_conditions.outputs.stagnation_temperature
    lpc_conditions.inputs.stagnation_pressure     = fan_conditions.outputs.stagnation_pressure
    lpc_conditions.inputs.static_temperature      = fan_conditions.outputs.static_temperature
    lpc_conditions.inputs.static_pressure         = fan_conditions.outputs.static_pressure
    lpc_conditions.inputs.mach_number             = fan_conditions.outputs.mach_number  
    low_pressure_compressor.working_fluid         = turbofan.working_fluid
        
    # Flow through the low pressure compressor
    compute_compressor_performance(low_pressure_compressor,lpc_conditions,conditions)

    # Link the high pressure compressor to the low pressure compressor
    hpc_conditions.inputs.stagnation_temperature = lpc_conditions.outputs.stagnation_temperature
    hpc_conditions.inputs.stagnation_pressure    = lpc_conditions.outputs.stagnation_pressure
    hpc_conditions.inputs.static_temperature     = lpc_conditions.outputs.static_temperature
    hpc_conditions.inputs.static_pressure        = lpc_conditions.outputs.static_pressure
    hpc_conditions.inputs.mach_number            = lpc_conditions.outputs.mach_number  
    high_pressure_compressor.working_fluid       = low_pressure_compressor.working_fluid    
        
    # Flow through the high pressure compressor
    compute_compressor_performance(high_pressure_compressor,hpc_conditions,conditions)

    # Link the combustor to the high pressure compressor
    combustor_conditions.inputs.stagnation_temperature                = hpc_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure                   = hpc_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature                    = hpc_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure                       = hpc_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number                           = hpc_conditions.outputs.mach_number  
    combustor.working_fluid                                           = high_pressure_compressor.working_fluid     
        
    # Flow through the high pressor compressor 
    compute_combustor_performance(combustor,combustor_conditions,conditions)

    # Link the shaft power output to the low pressure compressor
    try:
        shaft_power                                        = turbofan.Shaft_Power_Off_Take       
        shaft_power.inputs.mdhc                            = turbofan.compressor_nondimensional_massflow
        shaft_power.inputs.Tref                            = turbofan.reference_temperature
        shaft_power.inputs.Pref                            = turbofan.reference_pressure
        shaft_power.inputs.total_temperature_reference     = lpc_conditions.outputs.stagnation_temperature
        shaft_power.inputs.total_pressure_reference        = lpc_conditions.outputs.stagnation_pressure

        shaft_power(conditions)
        lpt_conditions.inputs.shaft_power_off_take   = shaft_power.outputs
    except:
        pass

    # Link the high pressure turbine to the combustor
    hpt_conditions.inputs.stagnation_temperature    = combustor_conditions.outputs.stagnation_temperature
    hpt_conditions.inputs.stagnation_pressure       = combustor_conditions.outputs.stagnation_pressure
    hpt_conditions.inputs.fuel_to_air_ratio         = combustor_conditions.outputs.fuel_to_air_ratio
    hpt_conditions.inputs.static_temperature        = combustor_conditions.outputs.static_temperature
    hpt_conditions.inputs.static_pressure           = combustor_conditions.outputs.static_pressure
    hpt_conditions.inputs.mach_number               = combustor_conditions.outputs.mach_number  
    hpt_conditions.inputs.compressor                = hpc_conditions.outputs 
    hpt_conditions.inputs.fan                       = fan_conditions.outputs
    hpt_conditions.inputs.bypass_ratio              = 0.0 #set to zero to ensure that fan not linked here 
    high_pressure_turbine.working_fluid             = combustor.working_fluid 
        
    # Flow through the high pressure turbine
    compute_turbine_performance(high_pressure_turbine,hpt_conditions,conditions)

    # Link the low pressure turbine to the high pressure turbine
    lpt_conditions.inputs.stagnation_temperature     = hpt_conditions.outputs.stagnation_temperature
    lpt_conditions.inputs.stagnation_pressure        = hpt_conditions.outputs.stagnation_pressure
    lpt_conditions.inputs.static_temperature         = hpt_conditions.outputs.static_temperature
    lpt_conditions.inputs.static_pressure            = hpt_conditions.outputs.static_pressure  
    lpt_conditions.inputs.mach_number                = hpt_conditions.outputs.mach_number    
    lpt_conditions.inputs.compressor                 = lpc_conditions.outputs 
    lpt_conditions.inputs.fuel_to_air_ratio          = combustor_conditions.outputs.fuel_to_air_ratio 
    lpt_conditions.inputs.fan                        = fan_conditions.outputs  
    lpt_conditions.inputs.bypass_ratio               = bypass_ratio
    low_pressure_turbine.working_fluid               = high_pressure_turbine.working_fluid  

    # Flow through the low pressure turbine
    compute_turbine_performance(low_pressure_turbine,lpt_conditions,conditions)

    # Link the core nozzle to the low pressure turbine
    core_nozzle_conditions.inputs.stagnation_temperature     = lpt_conditions.outputs.stagnation_temperature
    core_nozzle_conditions.inputs.stagnation_pressure        = lpt_conditions.outputs.stagnation_pressure
    core_nozzle_conditions.inputs.static_temperature         = lpt_conditions.outputs.static_temperature
    core_nozzle_conditions.inputs.static_pressure            = lpt_conditions.outputs.static_pressure  
    core_nozzle_conditions.inputs.mach_number                = lpt_conditions.outputs.mach_number   
    core_nozzle.working_fluid                                = turbofan.working_fluid 
        
    # Flow through the core nozzle
    compute_expansion_nozzle_performance(core_nozzle,core_nozzle_conditions,conditions)

    # Link the dan nozzle to the fan
    fan_nozzle_conditions.inputs.stagnation_temperature     = fan_conditions.outputs.stagnation_temperature
    fan_nozzle_conditions.inputs.stagnation_pressure        = fan_conditions.outputs.stagnation_pressure
    fan_nozzle_conditions.inputs.static_temperature         = fan_conditions.outputs.static_temperature
    fan_nozzle_conditions.inputs.static_pressure            = fan_conditions.outputs.static_pressure  
    fan_nozzle_conditions.inputs.mach_number                = fan_conditions.outputs.mach_number   
    fan_nozzle.working_fluid                                = turbofan.working_fluid
        
    # Flow through the fan nozzle
    compute_expansion_nozzle_performance(fan_nozzle,fan_nozzle_conditions,conditions)
 
    # Link the thrust component to the fan nozzle
    turbofan_conditions.fan_nozzle_exit_velocity                        = fan_nozzle_conditions.outputs.velocity
    turbofan_conditions.fan_nozzle_area_ratio                           = fan_nozzle_conditions.outputs.area_ratio  
    turbofan_conditions.fan_nozzle_static_pressure                      = fan_nozzle_conditions.outputs.static_pressure
    turbofan_conditions.core_nozzle_area_ratio                          = core_nozzle_conditions.outputs.area_ratio 
    turbofan_conditions.core_nozzle_static_pressure                     = core_nozzle_conditions.outputs.static_pressure
    turbofan_conditions.core_nozzle_exit_velocity                       = core_nozzle_conditions.outputs.velocity  

    # Link the thrust component to the combustor
    turbofan_conditions.fuel_to_air_ratio                        = combustor_conditions.outputs.fuel_to_air_ratio

    # Link the thrust component to the low pressure compressor 
    turbofan_conditions.total_temperature_reference              = lpc_conditions.outputs.stagnation_temperature
    turbofan_conditions.total_pressure_reference                 = lpc_conditions.outputs.stagnation_pressure 
    turbofan_conditions.bypass_ratio                             = bypass_ratio
    turbofan_conditions.flow_through_core                        = 1./(1.+bypass_ratio) #scaled constant to turn on core thrust computation
    turbofan_conditions.flow_through_fan                         = bypass_ratio/(1.+bypass_ratio) #scaled constant to turn on fan thrust computation        

    # Compute the thrust
    compute_thrust(turbofan,turbofan_conditions,conditions)

    # Compute forces and moments
    moment_vector              = 0*state.ones_row(3)
    thrust_vector              = 0*state.ones_row(3)
    thrust_vector[:,0]         =  turbofan_conditions.thrust[:,0]
    center_of_gravity = [[0.0, 0.0,0.0]] 
    moment_vector[:,0]         =  turbofan.origin[0][0] -   center_of_gravity[0][0]
    moment_vector[:,1]         =  turbofan.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]         =  turbofan.origin[0][2]  -  center_of_gravity[0][2]
    M                          =  np.cross(moment_vector, thrust_vector)   
    moment                     = M 
    power                      = turbofan_conditions.power 
    turbofan_conditions.moment = moment 
        
    # compute efficiencies 
    mdot_air_core                                  = turbofan_conditions.core_mass_flow_rate
    mdot_air_fan                                   = bypass_ratio *  mdot_air_core  
    fuel_enthalpy                                  = combustor.fuel_data.specific_energy 
    mdot_fuel                                      = turbofan_conditions.fuel_flow_rate  
    h_e_f                                          = fan_nozzle_conditions.outputs.static_enthalpy
    h_e_c                                          = core_nozzle_conditions.outputs.static_enthalpy
    h_0                                            = turbofan.working_fluid.compute_cp(T,P) * T 
    h_t4                                           = combustor_conditions.outputs.stagnation_enthalpy
    h_t3                                           = hpc_conditions.outputs.stagnation_enthalpy 
    turbofan_conditions.overall_efficiency         = thrust_vector* U0 / (mdot_fuel * fuel_enthalpy)  
    turbofan_conditions.thermal_efficiency         = 1 - ((mdot_air_core +  mdot_fuel)*(h_e_c -  h_0) + mdot_air_fan*(h_e_f - h_0) + mdot_fuel *h_0)/((mdot_air_core +  mdot_fuel)*h_t4 - mdot_air_core *h_t3)  
  
    # store data
    core_nozzle_res = Data(
                exit_static_temperature             = core_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = core_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = core_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = core_nozzle_conditions.outputs.static_pressure,
                exit_velocity                       = core_nozzle_conditions.outputs.velocity
            )

    fan_nozzle_res = Data(
                exit_static_temperature             = fan_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = fan_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = fan_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = fan_nozzle_conditions.outputs.static_pressure,
                exit_velocity                       = fan_nozzle_conditions.outputs.velocity
            )

    noise_conditions.turbofan.fan_nozzle    = fan_nozzle_res
    noise_conditions.turbofan.core_nozzle   = core_nozzle_res  
    noise_conditions.turbofan.fan           = None
    stored_results_flag                     = True
    stored_propulsor_tag                    = turbofan.tag 
    
    return thrust_vector,moment,power,stored_results_flag,stored_propulsor_tag 
    
def reuse_stored_turbofan_data(turbofan,state,network,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one turbofan for identical turbofans
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    turbofan             - turbofan data structure                [-]
    state                - operating conditions data structure   [-]  
    fuel_line            - fuelline                              [-]  
    total_thrust         - thrust of turbofan group              [N]
    total_power          - power of turbofan group               [W] 

    Outputs:  
    total_thrust         - thrust of turbofan group              [N]
    total_power          - power of turbofan group               [W] 
    
    Properties Used: 
    N.A.        
    ''' 
    conditions                                      = state.conditions  
    conditions.energy[turbofan.tag]  = deepcopy(conditions.energy[stored_propulsor_tag])
    conditions.noise[turbofan.tag]   = deepcopy(conditions.noise[stored_propulsor_tag])
    
    # compute moment  
    moment_vector      = 0*state.ones_row(3)
    thrust_vector      = 0*state.ones_row(3)
    thrust_vector[:,0] = conditions.energy[turbofan.tag].thrust[:,0] 
    moment_vector[:,0] = turbofan.origin[0][0] -   center_of_gravity[0][0] 
    moment_vector[:,1] = turbofan.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2] = turbofan.origin[0][2]  -  center_of_gravity[0][2]
    moment             = np.cross(moment_vector,thrust_vector)    
  
    power                                   = conditions.energy[turbofan.tag].power 
    conditions.energy[turbofan.tag].moment =  moment 
 
    return thrust_vector,moment,power    