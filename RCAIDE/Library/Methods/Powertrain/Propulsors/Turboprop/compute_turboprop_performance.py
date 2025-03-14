# RCAIDE/Methods/Energy/Propulsors/Networks/Turboprop_Propulsor/compute_turboprop_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports      

from RCAIDE.Framework.Core                                             import Data 
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                  import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor            import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor           import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine              import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle     import compute_expansion_nozzle_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle   import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop            import compute_thrust
 
# python imports 
from   copy import deepcopy
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# compute_turboprop_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_turboprop_performance(turboprop,state,fuel_line=None,bus=None,center_of_gravity= [[0.0, 0.0,0.0]]):    
    
    ''' Computes the performance of one turboprop
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure     [-]  
    fuel_line            - fuel line                               [-] 
    turboprop            - turboprop data structure                [-] 
    total_power          - power of turboprop group                [W] 

    Outputs:  
    thrust               - thrust of turboprop group               [W]
    power                - power of turboprop group                [W] 
    stored_results_flag  - boolean for stored results              [-]     
    stored_propulsor_tag - name of turboprop with stored results   [-]
    
    Properties Used: 
    N.A.        
    ''' 
    conditions               = state.conditions 
    noise_conditions         = conditions.noise.propulsors[turboprop.tag]  
    turboprop_conditions     = conditions.energy.propulsors[turboprop.tag]
    U0                       = conditions.freestream.velocity
    T                        = conditions.freestream.temperature
    P                        = conditions.freestream.pressure 
    ram                      = turboprop.ram
    inlet_nozzle             = turboprop.inlet_nozzle
    compressor               = turboprop.compressor
    combustor                = turboprop.combustor
    high_pressure_turbine    = turboprop.high_pressure_turbine
    low_pressure_turbine     = turboprop.low_pressure_turbine
    core_nozzle              = turboprop.core_nozzle 
    ram_conditions           = conditions.energy.converters[ram.tag]     
    inlet_nozzle_conditions  = conditions.energy.converters[inlet_nozzle.tag]
    core_nozzle_conditions   = conditions.energy.converters[core_nozzle.tag] 
    compressor_conditions    = conditions.energy.converters[compressor.tag]  
    combustor_conditions     = conditions.energy.converters[combustor.tag]
    lpt_conditions           = conditions.energy.converters[low_pressure_turbine.tag]
    hpt_conditions           = conditions.energy.converters[high_pressure_turbine.tag] 
     
    # Step 1: Set the working fluid to determine the fluid properties
    ram.working_fluid                                     = turboprop.working_fluid

    # Step 2: Compute flow through the ram , this computes the necessary flow quantities and stores it into conditions
    compute_ram_performance(ram,ram_conditions,conditions)

    # Step 3: link inlet nozzle to ram 
    inlet_nozzle_conditions.inputs.stagnation_temperature = ram_conditions.outputs.stagnation_temperature
    inlet_nozzle_conditions.inputs.stagnation_pressure    = ram_conditions.outputs.stagnation_pressure
    inlet_nozzle_conditions.inputs.static_temperature     = ram_conditions.outputs.static_temperature
    inlet_nozzle_conditions.inputs.static_pressure        = ram_conditions.outputs.static_pressure
    inlet_nozzle_conditions.inputs.mach_number            = ram_conditions.outputs.mach_number
    inlet_nozzle.working_fluid                            = ram.working_fluid

    # Step 4: Compute flow through the inlet nozzle
    compute_compression_nozzle_performance(inlet_nozzle,inlet_nozzle_conditions,conditions)      

    # Step 5: Link low pressure compressor to the inlet nozzle 
    compressor_conditions.inputs.stagnation_temperature   = inlet_nozzle_conditions.outputs.stagnation_temperature
    compressor_conditions.inputs.stagnation_pressure      = inlet_nozzle_conditions.outputs.stagnation_pressure
    compressor_conditions.inputs.static_temperature       = inlet_nozzle_conditions.outputs.static_temperature
    compressor_conditions.inputs.static_pressure          = inlet_nozzle_conditions.outputs.static_pressure
    compressor_conditions.inputs.mach_number              = inlet_nozzle_conditions.outputs.mach_number  
    compressor.working_fluid                              = inlet_nozzle.working_fluid
    compressor.nondimensional_massflow                    = turboprop.compressor_nondimensional_massflow
    compressor_conditions.reference_temperature           = turboprop.reference_temperature
    compressor_conditions.reference_pressure              = turboprop.reference_pressure
    
    # Step 6: Compute flow through the low pressure compressor
    compute_compressor_performance(compressor,compressor_conditions,conditions)
    
    # Step 7: Link the combustor to the high pressure compressor
    combustor_conditions.inputs.stagnation_temperature    = compressor_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure       = compressor_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature        = compressor_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure           = compressor_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number               = compressor_conditions.outputs.mach_number  
    combustor.working_fluid                               = compressor.working_fluid 
    
    # Step 8: Compute flow through the high pressor compressor 
    compute_combustor_performance(combustor,combustor_conditions,conditions)
    
    #link the high pressure turbione to the combustor 
    hpt_conditions.inputs.stagnation_temperature          = combustor_conditions.outputs.stagnation_temperature
    hpt_conditions.inputs.stagnation_pressure             = combustor_conditions.outputs.stagnation_pressure
    hpt_conditions.inputs.fuel_to_air_ratio               = combustor_conditions.outputs.fuel_to_air_ratio 
    hpt_conditions.inputs.static_temperature              = combustor_conditions.outputs.static_temperature
    hpt_conditions.inputs.static_pressure                 = combustor_conditions.outputs.static_pressure
    hpt_conditions.inputs.mach_number                     = combustor_conditions.outputs.mach_number 
    hpt_conditions.inputs.compressor                      = compressor_conditions.outputs 
    high_pressure_turbine.working_fluid                   = combustor.working_fluid
    hpt_conditions.inputs.bypass_ratio                    = 0.0
    
    compute_turbine_performance(high_pressure_turbine,hpt_conditions,conditions)
    
    #link the low pressure turbine to the high pressure turbine 
    lpt_conditions.inputs.stagnation_temperature          = hpt_conditions.outputs.stagnation_temperature
    lpt_conditions.inputs.stagnation_pressure             = hpt_conditions.outputs.stagnation_pressure 
    lpt_conditions.inputs.static_temperature              = hpt_conditions.outputs.static_temperature
    lpt_conditions.inputs.static_pressure                 = hpt_conditions.outputs.static_pressure 
    lpt_conditions.inputs.mach_number                     = hpt_conditions.outputs.mach_number     
    lpt_conditions.inputs.compressor                      = Data()
    lpt_conditions.inputs.compressor.work_done            = 0.0   
    lpt_conditions.inputs.compressor.external_shaft_work_done = 0.0
    lpt_conditions.inputs.bypass_ratio                    = 0.0 
    lpt_conditions.inputs.fuel_to_air_ratio               = combustor_conditions.outputs.fuel_to_air_ratio 
    low_pressure_turbine.working_fluid                    = high_pressure_turbine.working_fluid    
     
    compute_turbine_performance(low_pressure_turbine,lpt_conditions,conditions)
    
    #link the core nozzle to the low pressure turbine
    core_nozzle_conditions.inputs.stagnation_temperature  = lpt_conditions.outputs.stagnation_temperature
    core_nozzle_conditions.inputs.stagnation_pressure     = lpt_conditions.outputs.stagnation_pressure
    core_nozzle_conditions.inputs.static_temperature      = lpt_conditions.outputs.static_temperature
    core_nozzle_conditions.inputs.static_pressure         = lpt_conditions.outputs.static_pressure  
    core_nozzle_conditions.inputs.mach_number             = lpt_conditions.outputs.mach_number   
    core_nozzle.working_fluid                             = low_pressure_turbine.working_fluid 
    
    #flow through the core nozzle
    compute_expansion_nozzle_performance(core_nozzle,core_nozzle_conditions,conditions)

    # compute the thrust using the thrust component
    
    turboprop_conditions.total_temperature_reference      = compressor_conditions.inputs.stagnation_temperature
    turboprop_conditions.total_pressure_reference         = compressor_conditions.inputs.stagnation_pressure 

    # Compute the power
    compute_thrust(turboprop,turboprop_conditions,conditions) 

    # Compute forces and moments
    moment_vector      = 0*state.ones_row(3)
    thrust_vector      = 0*state.ones_row(3)
    thrust_vector[:,0] = turboprop_conditions.thrust[:,0]
    moment_vector[:,0] = turboprop.origin[0][0] -   center_of_gravity[0][0] 
    moment_vector[:,1] = turboprop.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2] = turboprop.origin[0][2]  -  center_of_gravity[0][2]
    M                  =  np.cross(moment_vector, thrust_vector)   
    moment             = M 
    power              = turboprop_conditions.power 
  
    # compute efficiencies 
    mdot_air_core                                  = turboprop_conditions.core_mass_flow_rate 
    fuel_enthalpy                                  = combustor.fuel_data.specific_energy 
    mdot_fuel                                      = turboprop_conditions.fuel_flow_rate   
    h_e_c                                          = core_nozzle_conditions.outputs.static_enthalpy
    h_0                                            = turboprop.working_fluid.compute_cp(T,P) * T 
    h_t4                                           = combustor_conditions.outputs.stagnation_enthalpy
    h_t3                                           = compressor_conditions.outputs.stagnation_enthalpy 
    turboprop_conditions.overall_efficiency        = thrust_vector* U0 / (mdot_fuel * fuel_enthalpy)  
    turboprop_conditions.thermal_efficiency        = 1 - ((mdot_air_core +  mdot_fuel)*(h_e_c -  h_0) + mdot_fuel *h_0)/((mdot_air_core +  mdot_fuel)*h_t4 - mdot_air_core *h_t3)   
    compressor_conditions.omega                    = compressor.design_angular_velocity * turboprop_conditions.throttle 

    if compressor.motor != None and  len(state.numerics.time.differentiate) > 0:
        D     = state.numerics.time.differentiate   
        compressor_motor_conditions                 = conditions.energy.converters[compressor.motor.tag] 
        compressor_motor_conditions.outputs.power   = np.dot(D,compressor_motor_conditions.outputs.work_done) 
        compressor_motor_conditions.outputs.omega   = compressor_conditions.omega
    
    if compressor.generator != None and len(state.numerics.time.differentiate) > 0:
        D     = state.numerics.time.differentiate   
        compressor_generator_conditions                 = conditions.energy.converters[compressor.generator.tag] 
        compressor_generator_conditions.inputs.power   = np.dot(D,compressor_generator_conditions.inputs.work_done)         
        compressor_generator_conditions.inputs.omega   = compressor_conditions.omega
        
    # Store data
    core_nozzle_res = Data(
                exit_static_temperature             = core_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = core_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = core_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = core_nozzle_conditions.outputs.static_pressure,
                exit_velocity                       = core_nozzle_conditions.outputs.velocity
            )
  
    noise_conditions.core_nozzle   = core_nozzle_res  
    
    # Pack results    
    stored_results_flag    = True
    stored_propulsor_tag   = turboprop.tag
     
    power_elec =  compressor_conditions.outputs.external_electrical_power  
    
    return thrust_vector,moment,power,power_elec,stored_results_flag,stored_propulsor_tag 

def reuse_stored_turboprop_data(turboprop,state,network,fuel_line,bus,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one turboprop for identical propulsors
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure     [-]  
    fuel_line            - fuelline                                [-] 
    turboprop            - turboprop data structure              [-] 
    total_power          - power of turboprop group               [W] 

    Outputs:  
    total_power          - power of turboprop group               [W] 
    
    Properties Used: 
    N.A.        
    ''' 
    # unpack
    conditions                  = state.conditions 
    ram                         = turboprop.ram
    inlet_nozzle                = turboprop.inlet_nozzle 
    low_pressure_compressor     = turboprop.low_pressure_compressor
    high_pressure_compressor    = turboprop.high_pressure_compressor
    combustor                   = turboprop.combustor
    high_pressure_turbine       = turboprop.high_pressure_turbine
    low_pressure_turbine        = turboprop.low_pressure_turbine
    core_nozzle                 = turboprop.core_nozzle
    fan_nozzle                  = turboprop.fan_nozzle  
    ram_0                       = network.propulsors[stored_propulsor_tag].ram
    inlet_nozzle_0              = network.propulsors[stored_propulsor_tag].inlet_nozzle 
    low_pressure_compressor_0   = network.propulsors[stored_propulsor_tag].low_pressure_compressor
    high_pressure_compressor_0  = network.propulsors[stored_propulsor_tag].high_pressure_compressor
    combustor_0                 = network.propulsors[stored_propulsor_tag].combustor
    high_pressure_turbine_0     = network.propulsors[stored_propulsor_tag].high_pressure_turbine
    low_pressure_turbine_0      = network.propulsors[stored_propulsor_tag].low_pressure_turbine
    core_nozzle_0               = network.propulsors[stored_propulsor_tag].core_nozzle
    fan_nozzle_0                = network.propulsors[stored_propulsor_tag].fan_nozzle 

    # deep copy results 
    conditions.energy.propulsors[turboprop.tag]                 = deepcopy(conditions.energy.propulsors[stored_propulsor_tag])
    conditions.noise.propulsors[turboprop.tag]                  = deepcopy(conditions.noise.propulsors[stored_propulsor_tag]) 
    conditions.energy.converters[ram.tag]                      = deepcopy(conditions.energy.converters[ram_0.tag]                     )
    conditions.energy.converters[inlet_nozzle.tag]             = deepcopy(conditions.energy.converters[inlet_nozzle_0.tag]            ) 
    conditions.energy.converters[low_pressure_compressor.tag]  = deepcopy(conditions.energy.converters[low_pressure_compressor_0.tag] )
    conditions.energy.converters[high_pressure_compressor.tag] = deepcopy(conditions.energy.converters[high_pressure_compressor_0.tag])
    conditions.energy.converters[combustor.tag]                = deepcopy(conditions.energy.converters[combustor_0.tag]               )
    conditions.energy.converters[low_pressure_turbine.tag]     = deepcopy(conditions.energy.converters[low_pressure_turbine_0.tag]    )
    conditions.energy.converters[high_pressure_turbine.tag]    = deepcopy(conditions.energy.converters[high_pressure_turbine_0.tag]   )
    conditions.energy.converters[core_nozzle.tag]              = deepcopy(conditions.energy.converters[core_nozzle_0.tag]             )
    conditions.energy.converters[fan_nozzle.tag]               = deepcopy(conditions.energy.converters[fan_nozzle_0.tag]              )

    # compute moment  
    moment_vector      = 0*state.ones_row(3)
    thrust_vector      = 0*state.ones_row(3)
    thrust_vector[:,0] = conditions.energy.propulsors[turboprop.tag].thrust[:,0] 
    moment_vector[:,0] = turboprop.origin[0][0] -   center_of_gravity[0][0] 
    moment_vector[:,1] = turboprop.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2] = turboprop.origin[0][2]  -  center_of_gravity[0][2]
    moment             = np.cross(moment_vector,thrust_vector)    

    power                                              = conditions.energy.propulsors[turboprop.tag].power 
    conditions.energy.propulsors[turboprop.tag].moment = moment

    power_elec =  conditions.energy.converters[low_pressure_compressor.tag].outputs.external_electrical_power + conditions.energy.converters[high_pressure_compressor.tag].outputs.external_electrical_power 

    return thrust_vector,moment,power, power_elec