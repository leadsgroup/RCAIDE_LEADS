# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan_Propulsor/design_turbofan.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE 
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor          import compute_combustor_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor         import compute_compressor_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Fan                import compute_fan_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine            import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle   import compute_expansion_nozzle_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan_Propulsor            import size_core 
from RCAIDE.Library.Methods.Powertrain.Systems                        import setup_operating_conditions 


# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Design Turbofan
# ---------------------------------------------------------------------------------------------------------------------- 
def design_turbofan(turbofan):
    """Compute perfomance properties of a turbofan based on polytropic ration and combustor properties.
    Turbofan is created by manually linking the different components
    
    
    Assumtions:
       None 
    
    Source:
    
    Args:
        turbofan (dict): turbofan data structure [-]
    
    Returns:
        None 
    
    """
    # check if mach number and temperature are passed
    if(turbofan.design_mach_number==None) and (turbofan.design_altitude==None): 
        raise NameError('The sizing conditions require an altitude and a Mach number') 
    else:
        #call the atmospheric model to get the conditions at the specified altitude
        atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmo_data  = atmosphere.compute_values(turbofan.design_altitude,turbofan.design_isa_deviation)
        planet     = RCAIDE.Library.Attributes.Planets.Earth()
        
        p   = atmo_data.pressure          
        T   = atmo_data.temperature       
        rho = atmo_data.density          
        a   = atmo_data.speed_of_sound    
        mu  = atmo_data.dynamic_viscosity           
        U   = a*turbofan.design_mach_number
        # setup conditions
        conditions = RCAIDE.Framework.Mission.Common.Results()
    
        # freestream conditions    
        conditions.freestream.altitude                    = np.atleast_1d(turbofan.design_altitude)
        conditions.freestream.mach_number                 = np.atleast_1d(turbofan.design_mach_number)
        conditions.freestream.pressure                    = np.atleast_1d(p)
        conditions.freestream.temperature                 = np.atleast_1d(T)
        conditions.freestream.density                     = np.atleast_1d(rho)
        conditions.freestream.dynamic_viscosity           = np.atleast_1d(mu)
        conditions.freestream.gravity                     = np.atleast_1d(planet.compute_gravity(turbofan.design_altitude))
        conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turbofan.working_fluid.compute_gamma(T,p))
        conditions.freestream.Cp                          = np.atleast_1d(turbofan.working_fluid.compute_cp(T,p))
        conditions.freestream.R                           = np.atleast_1d(turbofan.working_fluid.gas_specific_constant)
        conditions.freestream.speed_of_sound              = np.atleast_1d(a)
        conditions.freestream.velocity                    = np.atleast_1d(U) 
     
    segment                  = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions = conditions 
    turbofan.append_operating_conditions(segment) 
    for tag, item in  turbofan.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,turbofan) 
    
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
    turbofan_conditions     = conditions.energy[turbofan.tag]
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
     
    # Step 1: Set the working fluid to determine the fluid properties
    ram.working_fluid                             = turbofan.working_fluid
    
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
    
    # Step 5: Link the fan to the inlet nozzle
    fan_conditions.inputs.stagnation_temperature                      = inlet_nozzle_conditions.outputs.stagnation_temperature
    fan_conditions.inputs.stagnation_pressure                         = inlet_nozzle_conditions.outputs.stagnation_pressure
    fan_conditions.inputs.static_temperature                          = inlet_nozzle_conditions.outputs.static_temperature
    fan_conditions.inputs.static_pressure                             = inlet_nozzle_conditions.outputs.static_pressure
    fan_conditions.inputs.mach_number                                 = inlet_nozzle_conditions.outputs.mach_number  
    fan.working_fluid                                                 = inlet_nozzle.working_fluid
     
    # Step 6: Compute flow through the fan
    compute_fan_performance(fan,fan_conditions,conditions)    

    # Step 7: Link low pressure compressor to the inlet nozzle
    lpc_conditions.inputs.stagnation_temperature                      = fan_conditions.outputs.stagnation_temperature
    lpc_conditions.inputs.stagnation_pressure                         = fan_conditions.outputs.stagnation_pressure
    lpc_conditions.inputs.static_temperature                          = fan_conditions.outputs.static_temperature
    lpc_conditions.inputs.static_pressure                             = fan_conditions.outputs.static_pressure
    lpc_conditions.inputs.mach_number                                 = fan_conditions.outputs.mach_number  
    low_pressure_compressor.working_fluid                             = fan.working_fluid
    
    # Step 8: Compute flow through the low pressure compressor
    compute_compressor_performance(low_pressure_compressor,lpc_conditions,conditions)
    
    # Step 9: Link the high pressure compressor to the low pressure compressor
    hpc_conditions.inputs.stagnation_temperature                      = lpc_conditions.outputs.stagnation_temperature
    hpc_conditions.inputs.stagnation_pressure                         = lpc_conditions.outputs.stagnation_pressure
    hpc_conditions.inputs.static_temperature                          = lpc_conditions.outputs.static_temperature
    hpc_conditions.inputs.static_pressure                             = lpc_conditions.outputs.static_pressure
    hpc_conditions.inputs.mach_number                                 = lpc_conditions.outputs.mach_number  
    high_pressure_compressor.working_fluid                            = low_pressure_compressor.working_fluid    
    
    # Step 10: Compute flow through the high pressure compressor
    compute_compressor_performance(high_pressure_compressor,hpc_conditions,conditions)

    # Step 11: Link the combustor to the high pressure compressor    
    combustor_conditions.inputs.stagnation_temperature                = hpc_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure                   = hpc_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature                    = hpc_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure                       = hpc_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number                           = hpc_conditions.outputs.mach_number  
    combustor.working_fluid                                           = high_pressure_compressor.working_fluid     
    
    # Step 12: Compute flow through the high pressor compressor 
    compute_combustor_performance(combustor,combustor_conditions,conditions)
    
    # Step 13: Link the high pressure turbione to the combustor
    hpt_conditions.inputs.stagnation_temperature    = combustor_conditions.outputs.stagnation_temperature
    hpt_conditions.inputs.stagnation_pressure       = combustor_conditions.outputs.stagnation_pressure
    hpt_conditions.inputs.fuel_to_air_ratio         = combustor_conditions.outputs.fuel_to_air_ratio 
    hpt_conditions.inputs.static_temperature        = combustor_conditions.outputs.static_temperature
    hpt_conditions.inputs.static_pressure           = combustor_conditions.outputs.static_pressure
    hpt_conditions.inputs.mach_number               = combustor_conditions.outputs.mach_number       
    hpt_conditions.inputs.compressor                = hpc_conditions.outputs  
    hpt_conditions.inputs.fan                       = fan_conditions.outputs 
    hpt_conditions.inputs.bypass_ratio              = 0.0
    high_pressure_turbine.working_fluid             = combustor.working_fluid    
    
    # Step 14: Compute flow through the high pressure turbine
    compute_turbine_performance(high_pressure_turbine,hpt_conditions,conditions)
            
    # Step 15: Link the low pressure turbine to the high pressure turbine
    lpt_conditions.inputs.stagnation_temperature     = hpt_conditions.outputs.stagnation_temperature
    lpt_conditions.inputs.stagnation_pressure        = hpt_conditions.outputs.stagnation_pressure
    lpt_conditions.inputs.static_temperature         = hpt_conditions.outputs.static_temperature
    lpt_conditions.inputs.static_pressure            = hpt_conditions.outputs.static_pressure  
    lpt_conditions.inputs.mach_number                = hpt_conditions.outputs.mach_number    
    low_pressure_turbine.working_fluid               = high_pressure_turbine.working_fluid     
    lpt_conditions.inputs.compressor                 = lpc_conditions.outputs 
    lpt_conditions.inputs.fuel_to_air_ratio          = combustor_conditions.outputs.fuel_to_air_ratio 
    lpt_conditions.inputs.fan                        = fan_conditions.outputs 
    lpt_conditions.inputs.bypass_ratio               = bypass_ratio 
    
    # Step 16: Compute flow through the low pressure turbine
    compute_turbine_performance(low_pressure_turbine,lpt_conditions,conditions)
    
    # Step 17: Link the core nozzle to the low pressure turbine
    core_nozzle_conditions.inputs.stagnation_temperature     = lpt_conditions.outputs.stagnation_temperature
    core_nozzle_conditions.inputs.stagnation_pressure        = lpt_conditions.outputs.stagnation_pressure
    core_nozzle_conditions.inputs.static_temperature         = lpt_conditions.outputs.static_temperature
    core_nozzle_conditions.inputs.static_pressure            = lpt_conditions.outputs.static_pressure  
    core_nozzle_conditions.inputs.mach_number                = lpt_conditions.outputs.mach_number   
    core_nozzle.working_fluid                                = low_pressure_turbine.working_fluid 
    
    # Step 18: Compute flow through the core nozzle
    compute_expansion_nozzle_performance(core_nozzle,core_nozzle_conditions,conditions)
   
    # Step 19: Link the fan nozzle to the fan
    fan_nozzle_conditions.inputs.stagnation_temperature     = fan_conditions.outputs.stagnation_temperature
    fan_nozzle_conditions.inputs.stagnation_pressure        = fan_conditions.outputs.stagnation_pressure
    fan_nozzle_conditions.inputs.static_temperature         = fan_conditions.outputs.static_temperature
    fan_nozzle_conditions.inputs.static_pressure            = fan_conditions.outputs.static_pressure  
    fan_nozzle_conditions.inputs.mach_number                = fan_conditions.outputs.mach_number   
    fan_nozzle.working_fluid                                = fan.working_fluid
    
    # Step 20: Compute flow through the fan nozzle
    compute_expansion_nozzle_performance(fan_nozzle,fan_nozzle_conditions,conditions)
     
    # Step 21: Link the turbofan to outputs from various compoments    
    turbofan_conditions.bypass_ratio                             = bypass_ratio
    turbofan_conditions.fan_nozzle_exit_velocity                 = fan_nozzle_conditions.outputs.velocity
    turbofan_conditions.fan_nozzle_area_ratio                    = fan_nozzle_conditions.outputs.area_ratio  
    turbofan_conditions.fan_nozzle_static_pressure               = fan_nozzle_conditions.outputs.static_pressure
    turbofan_conditions.core_nozzle_area_ratio                   = core_nozzle_conditions.outputs.area_ratio 
    turbofan_conditions.core_nozzle_static_pressure              = core_nozzle_conditions.outputs.static_pressure
    turbofan_conditions.core_nozzle_exit_velocity                = core_nozzle_conditions.outputs.velocity 
    turbofan_conditions.fuel_to_air_ratio                        = combustor_conditions.outputs.fuel_to_air_ratio 
    turbofan_conditions.total_temperature_reference              = lpc_conditions.outputs.stagnation_temperature
    turbofan_conditions.total_pressure_reference                 = lpc_conditions.outputs.stagnation_pressure
    turbofan_conditions.flow_through_core                        = 1./(1.+bypass_ratio) #scaled constant to turn on core thrust computation
    turbofan_conditions.flow_through_fan                         = bypass_ratio/(1.+bypass_ratio) #scaled constant to turn on fan thrust computation        

    # Step 22: Size the core of the turbofan  
    size_core(turbofan,turbofan_conditions,conditions) 
    mass_flow                     = turbofan.mass_flow_rate_design
    turbofan.design_core_massflow = mass_flow   
    
    # Step 23: Static Sea Level Thrust  
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state,_     = setup_operating_conditions(turbofan, altitude = 0,velocity_vector=np.array([[V, 0, 0]]))  
    operating_state.conditions.energy[turbofan.tag].throttle[:,0] = 1.0  
    sls_T,_,sls_P,_,_                             = turbofan.compute_performance(operating_state) 
    turbofan.sealevel_static_thrust              = sls_T[0][0]
    turbofan.sealevel_static_power               = sls_P[0][0]
     
    return 