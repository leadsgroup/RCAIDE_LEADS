# RCAIDE/Library/Methods/Energy/Converters/Turboshaft/design_turboshaft.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

# RCAIDE Imports     
import RCAIDE
from RCAIDE.Framework.Core                                           import Data
from RCAIDE.Framework.Mission.Common                                 import Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor          import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor         import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine            import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle   import compute_expansion_nozzle_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft         import size_core  
from RCAIDE.Library.Methods.Powertrain                               import setup_operating_conditions 

# Python package imports   
import numpy                                                                as np

# ----------------------------------------------------------------------------------------------------------------------  
#  Design Turboshaft
# ----------------------------------------------------------------------------------------------------------------------   
def design_turboshaft(converter,segment,fuel_line):  
    # need to put if statements to check if Turboelectric generator houses the turboshaft or it is standalone
    if isinstance(converter, RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator):
        turboshaft = converter.turboshaft
    else:
        turboshaft = converter
    # check if mach number and temperature are passed
    if(turboshaft.design_mach_number==None) and (turboshaft.design_altitude==None): 
        raise NameError('The sizing conditions require an altitude and a Mach number') 
    else:
        #call the atmospheric model to get the conditions at the specified altitude
        atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmo_data  = atmosphere.compute_values(turboshaft.design_altitude,turboshaft.design_isa_deviation)
        planet     = RCAIDE.Library.Attributes.Planets.Earth()
        
        p   = atmo_data.pressure          
        T   = atmo_data.temperature       
        rho = atmo_data.density          
        a   = atmo_data.speed_of_sound    
        mu  = atmo_data.dynamic_viscosity           
        U   = a*turboshaft.design_mach_number
        
        # setup conditions
        conditions = RCAIDE.Framework.Mission.Common.Results()
    
        # freestream conditions    
        conditions.freestream.altitude                    = np.atleast_1d(turboshaft.design_altitude)
        conditions.freestream.mach_number                 = np.atleast_1d(turboshaft.design_mach_number)
        conditions.freestream.pressure                    = np.atleast_1d(p)
        conditions.freestream.temperature                 = np.atleast_1d(T)
        conditions.freestream.density                     = np.atleast_1d(rho)
        conditions.freestream.dynamic_viscosity           = np.atleast_1d(mu)
        conditions.freestream.gravity                     = np.atleast_1d(planet.compute_gravity(turboshaft.design_altitude))
        conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turboshaft.working_fluid.compute_gamma(T,p))
        conditions.freestream.Cp                          = np.atleast_1d(turboshaft.working_fluid.compute_cp(T,p))
        conditions.freestream.R                           = np.atleast_1d(turboshaft.working_fluid.gas_specific_constant)
        conditions.freestream.speed_of_sound              = np.atleast_1d(a)
        conditions.freestream.velocity                    = np.atleast_1d(U) 
     
    segment                  = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions = conditions 
    turboshaft.append_operating_conditions(segment,conditions.energy) 
            
    ram                     = turboshaft.ram
    inlet_nozzle            = turboshaft.inlet_nozzle
    compressor              = turboshaft.compressor
    combustor               = turboshaft.combustor
    high_pressure_turbine   = turboshaft.high_pressure_turbine
    low_pressure_turbine    = turboshaft.low_pressure_turbine
    core_nozzle             = turboshaft.core_nozzle
    
    turboshaft_conditions   = conditions.energy.converters[turboshaft.tag]
    ram_conditions          = conditions.energy.converters[ram.tag]     
    inlet_nozzle_conditions = conditions.energy.converters[inlet_nozzle.tag]
    core_nozzle_conditions  = conditions.energy.converters[core_nozzle.tag] 
    compressor_conditions   = conditions.energy.converters[compressor.tag]  
    combustor_conditions    = conditions.energy.converters[combustor.tag]
    lpt_conditions          = conditions.energy.converters[low_pressure_turbine.tag]
    hpt_conditions          = conditions.energy.converters[high_pressure_turbine.tag] 
     
    # Step 1: Set the working fluid to determine the fluid properties
    ram.working_fluid                             = turboshaft.working_fluid

    # Step 2: Compute flow through the ram , this computes the necessary flow quantities and stores it into conditions
    compute_ram_performance(ram,conditions)

    # Step 3: link inlet nozzle to ram 
    inlet_nozzle_conditions.inputs.stagnation_temperature             = ram_conditions.outputs.stagnation_temperature
    inlet_nozzle_conditions.inputs.stagnation_pressure                = ram_conditions.outputs.stagnation_pressure
    inlet_nozzle_conditions.inputs.static_temperature                 = ram_conditions.outputs.static_temperature
    inlet_nozzle_conditions.inputs.static_pressure                    = ram_conditions.outputs.static_pressure
    inlet_nozzle_conditions.inputs.mach_number                        = ram_conditions.outputs.mach_number
    inlet_nozzle.working_fluid                                        = ram.working_fluid

    # Step 4: Compute flow through the inlet nozzle
    compute_compression_nozzle_performance(inlet_nozzle,conditions)      

    # Step 5: Link low pressure compressor to the inlet nozzle 
    compressor_conditions.inputs.stagnation_temperature  = inlet_nozzle_conditions.outputs.stagnation_temperature
    compressor_conditions.inputs.stagnation_pressure     = inlet_nozzle_conditions.outputs.stagnation_pressure
    compressor_conditions.inputs.static_temperature      = inlet_nozzle_conditions.outputs.static_temperature
    compressor_conditions.inputs.static_pressure         = inlet_nozzle_conditions.outputs.static_pressure
    compressor_conditions.inputs.mach_number             = inlet_nozzle_conditions.outputs.mach_number  
    compressor.working_fluid                             = inlet_nozzle.working_fluid 
    compressor.reference_temperature                     = turboshaft.reference_temperature
    compressor.reference_pressure                        = turboshaft.reference_pressure

    # Step 6: Compute flow through the low pressure compressor
    compute_compressor_performance(compressor,conditions)
    
    # Step 11: Link the combustor to the high pressure compressor
    combustor_conditions.inputs.stagnation_temperature                = compressor_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure                   = compressor_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature                    = compressor_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure                       = compressor_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number                           = compressor_conditions.outputs.mach_number  
    combustor.working_fluid                                           = compressor.working_fluid 
    compressor.reference_temperature                                  = turboshaft.reference_temperature
    compressor.reference_pressure                                     = turboshaft.reference_pressure  
    
    # Step 12: Compute flow through the high pressor compressor 
    compute_combustor_performance(combustor,conditions)

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
    
    compute_turbine_performance(high_pressure_turbine,conditions)
            
    #link the low pressure turbine to the high pressure turbine 
    lpt_conditions.inputs.stagnation_temperature              = hpt_conditions.outputs.stagnation_temperature
    lpt_conditions.inputs.stagnation_pressure                 = hpt_conditions.outputs.stagnation_pressure 
    lpt_conditions.inputs.static_temperature                  = hpt_conditions.outputs.static_temperature
    lpt_conditions.inputs.static_pressure                     = hpt_conditions.outputs.static_pressure 
    lpt_conditions.inputs.mach_number                         = hpt_conditions.outputs.mach_number  
    low_pressure_turbine.working_fluid                        = high_pressure_turbine.working_fluid    
    lpt_conditions.inputs.compressor                          = Data()
    lpt_conditions.inputs.compressor.work_done                = 0.0 
    lpt_conditions.inputs.compressor.external_shaft_work_done = 0.0 
    lpt_conditions.inputs.fuel_to_air_ratio                   = combustor_conditions.outputs.fuel_to_air_ratio 
    lpt_conditions.inputs.bypass_ratio                        = 0.0
    lpt_conditions.inputs.fan                                 = Data()
    lpt_conditions.inputs.fan.work_done                       = 0.0
     
    compute_turbine_performance(low_pressure_turbine,conditions)
    
    #link the core nozzle to the low pressure turbine
    core_nozzle_conditions.inputs.stagnation_temperature     = lpt_conditions.outputs.stagnation_temperature
    core_nozzle_conditions.inputs.stagnation_pressure        = lpt_conditions.outputs.stagnation_pressure
    core_nozzle_conditions.inputs.static_temperature         = lpt_conditions.outputs.static_temperature
    core_nozzle_conditions.inputs.static_pressure            = lpt_conditions.outputs.static_pressure  
    core_nozzle_conditions.inputs.mach_number                = lpt_conditions.outputs.mach_number   
    core_nozzle.working_fluid                                = low_pressure_turbine.working_fluid  
    
    #flow through the core nozzle
    compute_expansion_nozzle_performance(core_nozzle,conditions)

    # compute the thrust using the thrust component
    #link the thrust component to the core nozzle
    turboshaft_conditions.core_exit_velocity                  = core_nozzle_conditions.outputs.velocity
    turboshaft_conditions.core_area_ratio                     = core_nozzle_conditions.outputs.area_ratio
    turboshaft_conditions.core_nozzle                         = core_nozzle_conditions.outputs
    
    #link the thrust component to the combustor
    turboshaft_conditions.fuel_to_air_ratio                   = combustor_conditions.outputs.fuel_to_air_ratio
    
    #link the thrust component to the low pressure compressor
    turboshaft_conditions.combustor_stagnation_temperature    = combustor_conditions.outputs.stagnation_temperature
    turboshaft_conditions.stag_temp_lpt_exit                  = compressor_conditions.inputs.stagnation_temperature
    turboshaft_conditions.stag_press_lpt_exit                 = compressor_conditions.inputs.stagnation_pressure 
    turboshaft_conditions.total_temperature_reference         = compressor_conditions.inputs.stagnation_temperature
    turboshaft_conditions.total_pressure_reference            = compressor_conditions.inputs.stagnation_pressure  

    #compute the power
    turboshaft_conditions.fan_nozzle                          = Data()
    turboshaft_conditions.fan_nozzle.velocity                 = 0.0
    turboshaft_conditions.fan_nozzle.area_ratio               = 0.0
    turboshaft_conditions.fan_nozzle.static_pressure          = 0.0
    turboshaft_conditions.bypass_ratio                        = 0.0
    turboshaft_conditions.flow_through_core                   = 1.0 #scaled constant to turn on core power computation
    turboshaft_conditions.flow_through_fan                    = 0.0 #scaled constant to turn on fan power computation      
    
    # Step 25: Size the core of the turboshaft  
    size_core(turboshaft,conditions)
    
    # Step 26: Static Sea Level Thrust  
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data_sea_level  = atmosphere.compute_values(0.0,0.0)   
    V                    = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state      = setup_operating_conditions(converter, altitude = 0,velocity_range=np.array([V]))  
    operating_state.conditions.energy.converters[turboshaft.tag].throttle[:,0] = 1.0  
    sls_P,_,_                                                       = turboshaft.compute_performance(operating_state,converter,fuel_line) 
    turboshaft.sealevel_static_power                                = sls_P[0][0]
     
    return      
  