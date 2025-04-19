# RCAIDE/Methods/Energy/Propulsors/Networks/turboshaft_Propulsor/compute_turboshaft_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports      
from RCAIDE.Framework.Core import Data   
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor          import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor         import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine            import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle   import compute_expansion_nozzle_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboshaft_Propulsor          import compute_power
 
# python imports 
from copy import deepcopy 
# ----------------------------------------------------------------------------------------------------------------------
# compute_turboshaft_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_turboshaft_performance(turboshaft,state,center_of_gravity= [[0.0, 0.0,0.0]]):    
    ''' Computes the perfomrance of one turboshaft
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure     [-]  
    fuel_line            - fuelline                                [-] 
    turboshaft           - turboshaft data structure               [-] 
    total_power          - power of turboshaft group               [W] 

    Outputs:  
    total_power          - power of turboshaft group               [W] 
    stored_results_flag  - boolean for stored results              [-]     
    stored_propulsor_tag - name of turboshaft with stored results  [-]
    
    Properties Used: 
    N.A.        
    ''' 
    conditions                = state.conditions 
    noise_conditions          = conditions.noise[turboshaft.tag]  
    turboshaft_conditions     = conditions.energy[turboshaft.tag]
    rho                       = conditions.freestream.density  
    ram                       = turboshaft.ram
    inlet_nozzle              = turboshaft.inlet_nozzle
    compressor                = turboshaft.compressor
    combustor                 = turboshaft.combustor
    high_pressure_turbine     = turboshaft.high_pressure_turbine
    low_pressure_turbine      = turboshaft.low_pressure_turbine 
    core_nozzle               = turboshaft.core_nozzle

    # unpack component conditions 
    ram_conditions          = turboshaft_conditions[ram.tag]     
    inlet_nozzle_conditions = turboshaft_conditions[inlet_nozzle.tag]
    core_nozzle_conditions  = turboshaft_conditions[core_nozzle.tag] 
    compressor_conditions   = turboshaft_conditions[compressor.tag] 
    lpt_conditions          = turboshaft_conditions[low_pressure_turbine.tag]
    hpt_conditions          = turboshaft_conditions[high_pressure_turbine.tag]
    combustor_conditions    = turboshaft_conditions[combustor.tag] 

    # Step 1: Set the working fluid to determine the fluid properties
    ram.working_fluid                             = turboshaft.working_fluid

    # Step 2: Compute flow through the ram , this computes the necessary flow quantities and stores it into conditions
    compute_ram_performance(ram,ram_conditions,conditions)

    # Step 3: link inlet nozzle to ram 
    inlet_nozzle_conditions.inputs.stagnation_temperature             = ram_conditions.outputs.stagnation_temperature
    inlet_nozzle_conditions.inputs.stagnation_pressure                = ram_conditions.outputs.stagnation_pressure
    inlet_nozzle_conditions.inputs.static_temperature                 = ram_conditions.outputs.static_temperature
    inlet_nozzle_conditions.inputs.static_pressure                    = ram_conditions.outputs.static_pressure
    inlet_nozzle_conditions.inputs.mach_number                        = ram_conditions.outputs.mach_number
    inlet_nozzle.working_fluid                                        = ram.working_fluid

    # Step 4: Compute flow through the inlet nozzle
    compute_compression_nozzle_performance(inlet_nozzle,inlet_nozzle_conditions,conditions)      

    # Step 5: Link low pressure compressor to the inlet nozzle 
    compressor_conditions.inputs.stagnation_temperature  = inlet_nozzle_conditions.outputs.stagnation_temperature
    compressor_conditions.inputs.stagnation_pressure     = inlet_nozzle_conditions.outputs.stagnation_pressure
    compressor_conditions.inputs.static_temperature      = inlet_nozzle_conditions.outputs.static_temperature
    compressor_conditions.inputs.static_pressure         = inlet_nozzle_conditions.outputs.static_pressure
    compressor_conditions.inputs.mach_number             = inlet_nozzle_conditions.outputs.mach_number  
    compressor.working_fluid                             = inlet_nozzle.working_fluid 

    # Step 6: Compute flow through the low pressure compressor
    compute_compressor_performance(compressor,compressor_conditions,conditions)

    # Step 11: Link the combustor to the high pressure compressor
    combustor_conditions.inputs.stagnation_temperature                = compressor_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure                   = compressor_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature                    = compressor_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure                       = compressor_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number                           = compressor_conditions.outputs.mach_number  
    combustor.working_fluid                                           = compressor.working_fluid 

    # Step 12: Compute flow through the high pressor compressor 
    compute_combustor_performance(combustor,combustor_conditions,conditions)

    #link the high pressure turbione to the combustor 
    hpt_conditions.inputs.stagnation_temperature    = combustor_conditions.outputs.stagnation_temperature
    hpt_conditions.inputs.stagnation_pressure       = combustor_conditions.outputs.stagnation_pressure
    hpt_conditions.inputs.fuel_to_air_ratio         = combustor_conditions.outputs.fuel_to_air_ratio 
    hpt_conditions.inputs.static_temperature        = combustor_conditions.outputs.static_temperature
    hpt_conditions.inputs.static_pressure           = combustor_conditions.outputs.static_pressure
    hpt_conditions.inputs.mach_number               = combustor_conditions.outputs.mach_number 
    hpt_conditions.inputs.compressor                = compressor_conditions.outputs 
    hpt_conditions.inputs.bypass_ratio              = 0.0
    hpt_conditions.inputs.fan                       = Data()
    hpt_conditions.inputs.fan.work_done             = 0.0
    high_pressure_turbine.working_fluid             = combustor.working_fluid
    
    compute_turbine_performance(high_pressure_turbine,hpt_conditions,conditions)

    #link the low pressure turbine to the high pressure turbine 
    lpt_conditions.inputs.stagnation_temperature     = hpt_conditions.outputs.stagnation_temperature
    lpt_conditions.inputs.stagnation_pressure        = hpt_conditions.outputs.stagnation_pressure 
    lpt_conditions.inputs.static_temperature         = hpt_conditions.outputs.static_temperature
    lpt_conditions.inputs.static_pressure            = hpt_conditions.outputs.static_pressure 
    lpt_conditions.inputs.mach_number                = hpt_conditions.outputs.mach_number  
    low_pressure_turbine.working_fluid               = high_pressure_turbine.working_fluid    
    lpt_conditions.inputs.compressor                 = Data()
    lpt_conditions.inputs.compressor.work_done       = 0.0     
    lpt_conditions.inputs.fuel_to_air_ratio          = combustor_conditions.outputs.fuel_to_air_ratio 
    lpt_conditions.inputs.bypass_ratio               = 0.0
    lpt_conditions.inputs.fan                        = Data()
    lpt_conditions.inputs.fan.work_done              = 0.0

    compute_turbine_performance(low_pressure_turbine,lpt_conditions,conditions)

    #link the core nozzle to the low pressure turbine
    core_nozzle_conditions.inputs.stagnation_temperature     = lpt_conditions.outputs.stagnation_temperature
    core_nozzle_conditions.inputs.stagnation_pressure        = lpt_conditions.outputs.stagnation_pressure
    core_nozzle_conditions.inputs.static_temperature         = lpt_conditions.outputs.static_temperature
    core_nozzle_conditions.inputs.static_pressure            = lpt_conditions.outputs.static_pressure  
    core_nozzle_conditions.inputs.mach_number                = lpt_conditions.outputs.mach_number   
    core_nozzle.working_fluid                                = low_pressure_turbine.working_fluid  

    #flow through the core nozzle
    compute_expansion_nozzle_performance(core_nozzle,core_nozzle_conditions,conditions) 
 
    # Link the thrust component to the core nozzle
    turboshaft_conditions.core_exit_velocity                       = core_nozzle_conditions.outputs.velocity
    turboshaft_conditions.core_area_ratio                          = core_nozzle_conditions.outputs.area_ratio
    turboshaft_conditions.core_nozzle                              = core_nozzle_conditions.outputs

    # Link the thrust component to the combustor
    turboshaft_conditions.fuel_to_air_ratio                        = combustor_conditions.outputs.fuel_to_air_ratio 

    # Link the thrust component to the low pressure compressor 
    turboshaft_conditions.combustor_stagnation_temperature         = combustor_conditions.outputs.stagnation_temperature
    turboshaft_conditions.total_temperature_reference              = compressor_conditions.inputs.stagnation_temperature
    turboshaft_conditions.total_pressure_reference                 = compressor_conditions.inputs.stagnation_pressure 
    turboshaft_conditions.flow_through_core                        =  1.0 #scaled constant to turn on core thrust computation
    turboshaft_conditions.flow_through_fan                         =  0.0 #scaled constant to turn on fan thrust computation        
    
    # Compute the power
    compute_power(turboshaft,turboshaft_conditions,conditions) 

    # Store data
    core_nozzle_res = Data(
                exit_static_temperature             = core_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = core_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = core_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = core_nozzle_conditions.outputs.static_pressure,
                exit_velocity                       = core_nozzle_conditions.outputs.velocity
            )
  
    noise_conditions.turboshaft.core_nozzle   = core_nozzle_res  
    
    # Pack results   
    moment                 = 0*state.ones_row(3)
    thrust                 = 0*state.ones_row(3) 
    power                  = turboshaft_conditions.power  
    stored_results_flag    = True
    stored_propulsor_tag   = turboshaft.tag
    
    return thrust,moment,power,stored_results_flag,stored_propulsor_tag

def reuse_stored_turboshaft_data(turboshaft,state,network,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one turboshaft for identical propulsors
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure     [-]  
    fuel_line            - fuelline                                [-] 
    turboshaft            - turboshaft data structure              [-] 
    total_power          - power of turboshaft group               [W] 

    Outputs:  
    total_power          - power of turboshaft group               [W] 
    
    Properties Used: 
    N.A.        
    ''' 
    conditions                         = state.conditions   
    conditions.energy[turboshaft.tag]  = deepcopy(conditions.energy[stored_propulsor_tag])
    conditions.noise[turboshaft.tag]   = deepcopy(conditions.noise[stored_propulsor_tag])
      
    power    = conditions.energy[turboshaft.tag].power    
    moment   = 0*state.ones_row(3)
    thrust   = 0*state.ones_row(3)
    return thrust,moment,power    