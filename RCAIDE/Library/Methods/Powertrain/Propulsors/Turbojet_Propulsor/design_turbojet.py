# RCAIDE/Methods/Energy/Propulsors/Turbojet_Propulsor/design_turbojet.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

# RCAIDE Imports     
import RCAIDE 
from RCAIDE.Framework.Mission.Common                                 import Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor          import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor         import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine            import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Supersonic_Nozzle  import compute_supersonic_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet_Propulsor            import size_core  
from RCAIDE.Library.Methods.Powertrain.Systems                        import setup_operating_conditions 

# Python package imports   
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------  
#  Design Turbojet
# ----------------------------------------------------------------------------------------------------------------------   
def design_turbojet(turbojet):  
    #check if mach number and temperature are passed
    if(turbojet.design_mach_number==None or turbojet.design_altitude==None):
        
        #raise an error
        raise NameError('The sizing conditions require an altitude and a Mach number')
    
    else:
        #call the atmospheric model to get the conditions at the specified altitude
        atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmo_data  = atmosphere.compute_values(turbojet.design_altitude,turbojet.design_isa_deviation)
        planet     = RCAIDE.Library.Attributes.Planets.Earth()
        
        p   = atmo_data.pressure          
        T   = atmo_data.temperature       
        rho = atmo_data.density          
        a   = atmo_data.speed_of_sound    
        mu  = atmo_data.dynamic_viscosity   
    
        # setup conditions
        conditions = RCAIDE.Framework.Mission.Common.Results()
    
        # freestream conditions    
        conditions.freestream.altitude                    = np.atleast_1d(turbojet.design_altitude)
        conditions.freestream.mach_number                 = np.atleast_1d(turbojet.design_mach_number)
        conditions.freestream.pressure                    = np.atleast_1d(p)
        conditions.freestream.temperature                 = np.atleast_1d(T)
        conditions.freestream.density                     = np.atleast_1d(rho)
        conditions.freestream.dynamic_viscosity           = np.atleast_1d(mu)
        conditions.freestream.gravity                     = np.atleast_1d(planet.compute_gravity(turbojet.design_altitude))
        conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turbojet.working_fluid.compute_gamma(T,p))
        conditions.freestream.Cp                          = np.atleast_1d(turbojet.working_fluid.compute_cp(T,p))
        conditions.freestream.R                           = np.atleast_1d(turbojet.working_fluid.gas_specific_constant)
        conditions.freestream.speed_of_sound              = np.atleast_1d(a)
        conditions.freestream.velocity                    = np.atleast_1d(a*turbojet.design_mach_number)
  
    fuel_line                                      = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    segment                                        = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions                       = conditions
    segment.state.conditions.energy[fuel_line.tag] = Conditions()
    segment.state.conditions.noise[fuel_line.tag]  = Conditions()
    turbojet.append_operating_conditions(segment) 
    for tag, item in  turbojet.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,turbojet)
    
    ram                       = turbojet.ram
    inlet_nozzle              = turbojet.inlet_nozzle
    low_pressure_compressor   = turbojet.low_pressure_compressor
    high_pressure_compressor  = turbojet.high_pressure_compressor
    combustor                 = turbojet.combustor
    high_pressure_turbine     = turbojet.high_pressure_turbine
    low_pressure_turbine      = turbojet.low_pressure_turbine
    core_nozzle               = turbojet.core_nozzle

    # unpack component conditions
    turbojet_conditions     = conditions.energy[turbojet.tag]
    ram_conditions          = turbojet_conditions[ram.tag]     
    inlet_nozzle_conditions = turbojet_conditions[inlet_nozzle.tag]
    core_nozzle_conditions  = turbojet_conditions[core_nozzle.tag] 
    lpc_conditions          = turbojet_conditions[low_pressure_compressor.tag]
    hpc_conditions          = turbojet_conditions[high_pressure_compressor.tag]
    lpt_conditions          = turbojet_conditions[low_pressure_turbine.tag]
    hpt_conditions          = turbojet_conditions[high_pressure_turbine.tag]
    combustor_conditions    = turbojet_conditions[combustor.tag] 
     
    # Step 1: Set the working fluid to determine the fluid properties
    ram.working_fluid                             = turbojet.working_fluid

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
    lpc_conditions.inputs.stagnation_temperature  = inlet_nozzle_conditions.outputs.stagnation_temperature
    lpc_conditions.inputs.stagnation_pressure     = inlet_nozzle_conditions.outputs.stagnation_pressure
    lpc_conditions.inputs.static_temperature      = inlet_nozzle_conditions.outputs.static_temperature
    lpc_conditions.inputs.static_pressure         = inlet_nozzle_conditions.outputs.static_pressure
    lpc_conditions.inputs.mach_number             = inlet_nozzle_conditions.outputs.mach_number  
    low_pressure_compressor.working_fluid         = inlet_nozzle.working_fluid

    # Step 6: Compute flow through the low pressure compressor
    compute_compressor_performance(low_pressure_compressor,lpc_conditions,conditions)

    # Step 7: Link the high pressure compressor to the low pressure compressor
    hpc_conditions.inputs.stagnation_temperature = lpc_conditions.outputs.stagnation_temperature
    hpc_conditions.inputs.stagnation_pressure    = lpc_conditions.outputs.stagnation_pressure
    hpc_conditions.inputs.static_temperature     = lpc_conditions.outputs.static_temperature
    hpc_conditions.inputs.static_pressure        = lpc_conditions.outputs.static_pressure
    hpc_conditions.inputs.mach_number            = lpc_conditions.outputs.mach_number  
    high_pressure_compressor.working_fluid       = low_pressure_compressor.working_fluid   

    # Step 8: Compute flow through the high pressure compressor
    compute_compressor_performance(high_pressure_compressor,hpc_conditions,conditions)
   
    # Step 9: Link the combustor to the high pressure compressor
    combustor_conditions.inputs.stagnation_temperature                = hpc_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure                   = hpc_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature                    = hpc_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure                       = hpc_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number                           = hpc_conditions.outputs.mach_number  
    combustor.working_fluid                                           = high_pressure_compressor.working_fluid  
    
    # Step 10: Compute flow through the high pressor compressor 
    compute_combustor_performance(combustor,combustor_conditions,conditions)

    hpt_conditions.inputs.stagnation_temperature    = combustor_conditions.outputs.stagnation_temperature
    hpt_conditions.inputs.stagnation_pressure       = combustor_conditions.outputs.stagnation_pressure
    hpt_conditions.inputs.fuel_to_air_ratio         = combustor_conditions.outputs.fuel_to_air_ratio 
    hpt_conditions.inputs.static_temperature        = combustor_conditions.outputs.static_temperature
    hpt_conditions.inputs.static_pressure           = combustor_conditions.outputs.static_pressure
    hpt_conditions.inputs.mach_number               = combustor_conditions.outputs.mach_number       
    hpt_conditions.inputs.compressor                = hpc_conditions.outputs   
    hpt_conditions.inputs.bypass_ratio              = 0.0
    high_pressure_turbine.working_fluid             = combustor.working_fluid 

    # Step 11: Compute flow through the high pressure turbine
    compute_turbine_performance(high_pressure_turbine,hpt_conditions,conditions)
            
    # Step 12: Link the low pressure turbine to the high pressure turbine
    lpt_conditions.inputs.stagnation_temperature     = hpt_conditions.outputs.stagnation_temperature
    lpt_conditions.inputs.stagnation_pressure        = hpt_conditions.outputs.stagnation_pressure 
    lpt_conditions.inputs.static_temperature         = hpt_conditions.outputs.static_temperature
    lpt_conditions.inputs.static_pressure            = hpt_conditions.outputs.static_pressure 
    lpt_conditions.inputs.mach_number                = hpt_conditions.outputs.mach_number  
    low_pressure_turbine.working_fluid               = high_pressure_turbine.working_fluid    
    lpt_conditions.inputs.compressor                 = lpc_conditions.outputs 
    lpt_conditions.inputs.fuel_to_air_ratio          = combustor_conditions.outputs.fuel_to_air_ratio 
    lpt_conditions.inputs.bypass_ratio               = 0.0 

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
    compute_supersonic_nozzle_performance(core_nozzle,core_nozzle_conditions,conditions)
 
    # Step 19: link the thrust component to the core nozzle
    turbojet_conditions.core_nozzle_area_ratio                   = core_nozzle_conditions.outputs.area_ratio 
    turbojet_conditions.core_nozzle_static_pressure              = core_nozzle_conditions.outputs.static_pressure
    turbojet_conditions.core_nozzle_exit_velocity                = core_nozzle_conditions.outputs.velocity 
    turbojet_conditions.fuel_to_air_ratio                        = combustor_conditions.outputs.fuel_to_air_ratio 
    turbojet_conditions.stag_temp_lpt_exit                       = lpc_conditions.outputs.stagnation_temperature
    turbojet_conditions.stag_press_lpt_exit                      = lpc_conditions.outputs.stagnation_pressure 
    turbojet_conditions.total_temperature_reference              = lpc_conditions.outputs.stagnation_temperature
    turbojet_conditions.total_pressure_reference                 = lpc_conditions.outputs.stagnation_pressure 
    turbojet_conditions.flow_through_core                        = 1.0 #scaled constant to turn on core thrust computation
    turbojet_conditions.flow_through_fan                         = 0.0 #scaled constant to turn on fan thrust computation      
    
    # Step 20: Size the core of the turbojet  
    size_core(turbojet,turbojet_conditions,conditions)
    mass_flow                     = turbojet.mass_flow_rate_design
    turbojet.design_core_massflow = mass_flow  
    
    # Step 21: Static Sea Level Thrust 
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state,_     = setup_operating_conditions(turbojet, altitude = 0,velocity_vector=np.array([[V, 0, 0]]))  
    operating_state.conditions.energy[turbojet.tag].throttle[:,0] = 1.0  
    sls_T,_,sls_P,_,_                            = turbojet.compute_performance(operating_state) 
    turbojet.sealevel_static_thrust              = sls_T[0][0]
    turbojet.sealevel_static_power               = sls_P[0][0]
     
    return      
  