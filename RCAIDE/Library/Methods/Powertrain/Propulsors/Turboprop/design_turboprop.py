# RCAIDE/Library/Methods/Energy/Powertrain/Propulsors/Turboprop/design_turboprop.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

# RCAIDE Imports     
import RCAIDE
from RCAIDE.Framework.Core                                                    import Data 
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                         import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor                   import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor                  import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine                     import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle            import compute_expansion_nozzle_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle          import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop                   import size_core 
from RCAIDE.Library.Methods.Powertrain                                        import setup_operating_conditions 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor                       import design_optimal_motor 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common   import compute_motor_weight

# Python package imports   
import numpy                                                                as np

# ----------------------------------------------------------------------------------------------------------------------  
#  Design Turboshaft
# ----------------------------------------------------------------------------------------------------------------------   
def design_turboprop(turboprop):  
    """
    Sizes a turboprop engine based on design point conditions and computes its performance characteristics.

    Parameters
    ----------
    turboprop : Turboprop
        Turboprop engine object containing all component definitions and design parameters
            - design_mach_number : float
                Design point Mach number
            - design_altitude : float
                Design point altitude [m]
            - design_isa_deviation : float
                ISA temperature deviation [K]
            - working_fluid : FluidProperties
                Working fluid properties object
            - Components:
                - ram : Ram
                - inlet_nozzle : Compression_Nozzle
                - compressor : Compressor
                - combustor : Combustor
                - high_pressure_turbine : Turbine
                - low_pressure_turbine : Turbine
                - core_nozzle : Expansion_Nozzle

    Returns
    -------
    None
        Results are stored in the turboprop object attributes:
            - design_thrust_specific_fuel_consumption : float
                TSFC at design point [kg/N/s]
            - design_non_dimensional_thrust : float
                Non-dimensional thrust at design point [-]
            - design_core_mass_flow_rate : float
                Core mass flow rate at design point [kg/s]
            - design_fuel_flow_rate : float
                Fuel flow rate at design point [kg/s]
            - design_power : float
                Power output at design point [W]
            - design_specific_power : float
                Specific power at design point [W/kg]
            - design_power_specific_fuel_consumption : float
                Power specific fuel consumption [kg/W/s]
            - design_thermal_efficiency : float
                Thermal efficiency at design point [-]
            - design_propulsive_efficiency : float
                Propulsive efficiency at design point [-]

    Notes
    -----
    The function performs the following steps:
        1. Computes atmospheric conditions at design altitude
        2. Sets up freestream conditions
        3. Analyzes flow through each component sequentially
        4. Sizes the core based on design point requirements
        5. Computes sea level static performance

    **Major Assumptions**
        * Standard atmospheric conditions (with possible ISA deviation)
        * Steady state operation
        * Perfect gas behavior
        * Adiabatic component processes except combustor
        * No bleed air extraction

    **Theory**

    The design process follows standard gas turbine cycle analysis, with each
    component modeled using appropriate thermodynamic relations. The core sizing
    is based on achieving the required power output while maintaining component
    matching throughout the engine.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Ram.compute_ram_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Combustor.compute_combustor_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Compressor.compute_compressor_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Turbine.compute_turbine_performance
    """
    #check if mach number and temperature are passed
    if turboprop.design_altitude==None:
        if turboprop.design_mach_number==None and turboprop.design_freestream_velocity ==None:  
            raise NameError('The sizing conditions require an altitude and a Mach number or Velocity ')
    
    else:
        #call the atmospheric model to get the conditions at the specified altitude
        atmosphere                                        = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmo_data                                         = atmosphere.compute_values(turboprop.design_altitude,turboprop.design_isa_deviation)
        planet                                            = RCAIDE.Library.Attributes.Planets.Earth()
                                                          
        p                                                 = atmo_data.pressure          
        T                                                 = atmo_data.temperature       
        rho                                               = atmo_data.density          
        a                                                 = atmo_data.speed_of_sound    
        mu                                                = atmo_data.dynamic_viscosity   
        
        if turboprop.design_mach_number==None:
            turboprop.design_mach_number =   turboprop.design_freestream_velocity / a 
            
        # setup conditions
        conditions                                        = RCAIDE.Framework.Mission.Common.Results()
    
        # freestream conditions    
        conditions.freestream.altitude                    = np.atleast_1d(turboprop.design_altitude)
        conditions.freestream.mach_number                 = np.atleast_1d(turboprop.design_mach_number)
        conditions.freestream.pressure                    = np.atleast_1d(p)
        conditions.freestream.temperature                 = np.atleast_1d(T)
        conditions.freestream.density                     = np.atleast_1d(rho)
        conditions.freestream.dynamic_viscosity           = np.atleast_1d(mu)
        conditions.freestream.gravity                     = np.atleast_1d(planet.compute_gravity(turboprop.design_altitude))
        conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turboprop.working_fluid.compute_gamma(T,p))
        conditions.freestream.Cp                          = np.atleast_1d(turboprop.working_fluid.compute_cp(T,p))
        conditions.freestream.R                           = np.atleast_1d(turboprop.working_fluid.gas_specific_constant)
        conditions.freestream.speed_of_sound              = np.atleast_1d(a)
        conditions.freestream.velocity                    = np.atleast_1d(a*turboprop.design_mach_number)
          
    segment                                               = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions                              = conditions 
    turboprop.append_operating_conditions(segment,conditions.energy,conditions.noise)       
         
    ram                                                   = turboprop.ram
    inlet_nozzle                                          = turboprop.inlet_nozzle
    compressor                                            = turboprop.compressor
    combustor                                             = turboprop.combustor
    high_pressure_turbine                                 = turboprop.high_pressure_turbine
    low_pressure_turbine                                  = turboprop.low_pressure_turbine
    core_nozzle                                           = turboprop.core_nozzle  

    # unpack component conditions
    turboprop_conditions                                  = conditions.energy.propulsors[turboprop.tag]
    ram_conditions                                        = conditions.energy.converters[ram.tag]     
    inlet_nozzle_conditions                               = conditions.energy.converters[inlet_nozzle.tag]
    core_nozzle_conditions                                = conditions.energy.converters[core_nozzle.tag] 
    compressor_conditions                                 = conditions.energy.converters[compressor.tag]  
    combustor_conditions                                  = conditions.energy.converters[combustor.tag]
    lpt_conditions                                        = conditions.energy.converters[low_pressure_turbine.tag]
    hpt_conditions                                        = conditions.energy.converters[high_pressure_turbine.tag] 
     
    # Step 1: Set the working fluid to determine the fluid properties
    ram.working_fluid                                     = turboprop.working_fluid

    # Step 2: Compute flow through the ram , this computes the necessary flow quantities and stores it into conditions
    compute_ram_performance(ram,conditions)

    # Step 3: link inlet nozzle to ram 
    inlet_nozzle_conditions.inputs.stagnation_temperature = ram_conditions.outputs.stagnation_temperature
    inlet_nozzle_conditions.inputs.stagnation_pressure    = ram_conditions.outputs.stagnation_pressure
    inlet_nozzle_conditions.inputs.static_temperature     = ram_conditions.outputs.static_temperature
    inlet_nozzle_conditions.inputs.static_pressure        = ram_conditions.outputs.static_pressure
    inlet_nozzle_conditions.inputs.mach_number            = ram_conditions.outputs.mach_number
    inlet_nozzle.working_fluid                            = ram.working_fluid

    # Step 4: Compute flow through the inlet nozzle
    compute_compression_nozzle_performance(inlet_nozzle,conditions)      

    # Step 5: Link low pressure compressor to the inlet nozzle 
    compressor_conditions.inputs.stagnation_temperature   = inlet_nozzle_conditions.outputs.stagnation_temperature
    compressor_conditions.inputs.stagnation_pressure      = inlet_nozzle_conditions.outputs.stagnation_pressure
    compressor_conditions.inputs.static_temperature       = inlet_nozzle_conditions.outputs.static_temperature
    compressor_conditions.inputs.static_pressure          = inlet_nozzle_conditions.outputs.static_pressure
    compressor_conditions.inputs.mach_number              = inlet_nozzle_conditions.outputs.mach_number  
    compressor.working_fluid                              = inlet_nozzle.working_fluid 
    compressor.nondimensional_massflow                    = turboprop.compressor_nondimensional_massflow
    compressor.reference_temperature                      = turboprop.reference_temperature
    compressor.reference_pressure                         = turboprop.reference_pressure  

    # Step 6: Compute flow through the low pressure compressor
    compute_compressor_performance(compressor,conditions)
    
    # Step 7: Link the combustor to the high pressure compressor
    combustor_conditions.inputs.stagnation_temperature    = compressor_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure       = compressor_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature        = compressor_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure           = compressor_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number               = compressor_conditions.outputs.mach_number  
    combustor.working_fluid                               = compressor.working_fluid 
    
    # Step 8: Compute flow through the high pressor compressor 
    compute_combustor_performance(combustor,conditions)

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
    
    compute_turbine_performance(high_pressure_turbine,conditions)
            
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
     
    compute_turbine_performance(low_pressure_turbine,conditions)
    
    #link the core nozzle to the low pressure turbine
    core_nozzle_conditions.inputs.stagnation_temperature  = lpt_conditions.outputs.stagnation_temperature
    core_nozzle_conditions.inputs.stagnation_pressure     = lpt_conditions.outputs.stagnation_pressure
    core_nozzle_conditions.inputs.static_temperature      = lpt_conditions.outputs.static_temperature
    core_nozzle_conditions.inputs.static_pressure         = lpt_conditions.outputs.static_pressure  
    core_nozzle_conditions.inputs.mach_number             = lpt_conditions.outputs.mach_number   
    core_nozzle.working_fluid                             = low_pressure_turbine.working_fluid 
    
    #flow through the core nozzle
    compute_expansion_nozzle_performance(core_nozzle,conditions)

    # compute the thrust using the thrust component
    turboprop_conditions.total_temperature_reference      = compressor_conditions.inputs.stagnation_temperature
    turboprop_conditions.total_pressure_reference         = compressor_conditions.inputs.stagnation_pressure 

    # Step 25: Size the core of the turboprop  
    size_core(turboprop,conditions)
    
    # Step 26: Static Sea Level Thrust   
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state       = setup_operating_conditions(turboprop, altitude = 0,velocity_range=np.array([V]), angle_of_attack=0, temperature_deviation=0)  
    operating_state.conditions.energy.propulsors[turboprop.tag].throttle[:,0] = 1.0  
    sls_T,_,sls_P,_,_,_                           = turboprop.compute_performance(operating_state) 
    turboprop.sealevel_static_thrust              = sls_T[0][0]
    turboprop.sealevel_static_power               = sls_P[0][0]
    
    turboprop.design_thrust_specific_fuel_consumption = turboprop_conditions.thrust_specific_fuel_consumption  
    turboprop.design_non_dimensional_thrust           = turboprop_conditions.non_dimensional_thrust            
    turboprop.design_core_mass_flow_rate              = turboprop_conditions.core_mass_flow_rate               
    turboprop.design_fuel_flow_rate                   = turboprop_conditions.fuel_flow_rate                           
    turboprop.design_specific_power                   = turboprop_conditions.specific_power                    
    turboprop.design_power_specific_fuel_consumption  = turboprop_conditions.power_specific_fuel_consumption   
    turboprop.design_thermal_efficiency               = turboprop_conditions.thermal_efficiency                
    turboprop.design_propulsive_efficiency            = turboprop_conditions.propulsive_efficiency
    
    if compressor.motor != None: 
        V                     = turboprop.design_freestream_velocity
        operating_state       = setup_operating_conditions(turboprop, altitude = turboprop.design_altitude,velocity_range=np.array([V]), angle_of_attack=0, temperature_deviation=0)  
        operating_state.conditions.energy.propulsors[turboprop.tag].throttle[:,0] = 1.0  
        T,_,P,_,_,_           = turboprop.compute_performance(operating_state)
        
        motor                         = compressor.motor 
        motor.design_torque           = P[0][0] /compressor.design_angular_velocity   
        motor.design_angular_velocity = compressor.design_angular_velocity
        motor.mass_properties.mass    = compute_motor_weight(motor) 
        design_optimal_motor(motor)
    
    return      
  