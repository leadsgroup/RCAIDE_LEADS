# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/design_turbofan.py 
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
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan           import size_core 
from RCAIDE.Library.Methods.Powertrain                               import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.design_optimal_motor import   design_optimal_motor
from RCAIDE.Library.Methods.Powertrain.Converters.Generator.design_optimal_generator import design_optimal_generator

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Design Turbofan
# ---------------------------------------------------------------------------------------------------------------------- 
def design_turbofan(turbofan):
    """
    Computes performance properties of a turbofan engine at the design point by linking
    and analyzing the thermodynamic cycle of its components.
    
    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Propulsors.Turbofan
        Turbofan engine component with the following attributes:
            - tag : str
                Identifier for the turbofan
            - design_mach_number : float
                Design Mach number
            - design_altitude : float
                Design altitude [m]
            - design_isa_deviation : float
                ISA temperature deviation at design point [K]
            - working_fluid : Data
                Working fluid properties object
            - reference_temperature : float
                Reference temperature for mass flow scaling [K]
            - reference_pressure : float
                Reference pressure for mass flow scaling [Pa]
            - bypass_ratio : float
                Bypass ratio of the turbofan
            - ram : Data
                Ram component
                    - tag : str
                        Identifier for the ram
            - inlet_nozzle : Data
                Inlet nozzle component
                    - tag : str
                        Identifier for the inlet nozzle
            - fan : Data
                Fan component
                    - tag : str
                        Identifier for the fan
            - low_pressure_compressor : Data
                Low pressure compressor component
                    - tag : str
                        Identifier for the low pressure compressor
            - high_pressure_compressor : Data
                High pressure compressor component
                    - tag : str
                        Identifier for the high pressure compressor
            - combustor : Data
                Combustor component
                    - tag : str
                        Identifier for the combustor
            - high_pressure_turbine : Data
                High pressure turbine component
                    - tag : str
                        Identifier for the high pressure turbine
            - low_pressure_turbine : Data
                Low pressure turbine component
                    - tag : str
                        Identifier for the low pressure turbine
            - core_nozzle : Data
                Core nozzle component
                    - tag : str
                        Identifier for the core nozzle
            - fan_nozzle : Data
                Fan nozzle component
                    - tag : str
                        Identifier for the fan nozzle
    
    Returns
    -------
    None
    
    Notes
    -----
    This function performs a complete design analysis of a turbofan engine by:
        1. Setting up atmospheric conditions at the design point
        2. Creating a mission segment for the design point
        3. Sequentially analyzing each component in the engine's thermodynamic cycle
        4. Linking the output conditions of each component to the input conditions of the next
        5. Sizing the core flow to meet the design requirements
        6. Computing sea level static performance
    
    The function follows this sequence for component analysis:
        1. Ram (inlet)
        2. Inlet nozzle
        3. Fan
        4. Low pressure compressor
        5. High pressure compressor
        6. Combustor
        7. High pressure turbine
        8. Low pressure turbine
        9. Core nozzle
        10. Fan nozzle
    
    **Major Assumptions**
        * US Standard Atmosphere 1976
        * Steady state operation
        * One-dimensional flow through components
        * Adiabatic components except for the combustor
        * Perfect gas behavior with variable properties
    
    References
    ----------
    [1] Mattingly, J.D., "Elements of Gas Turbine Propulsion", 2nd Edition, AIAA Education Series, 2005. https://soaneemrana.org/onewebmedia/ELEMENTS%20OF%20GAS%20TURBINE%20PROPULTION2.pdf
    [2] Cantwell, B., "AA283 Course Notes", Stanford University. https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_BOOK/AA283_Aircraft_and_Rocket_Propulsion_BOOK_Brian_J_Cantwell_May_28_2024.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.compute_turbofan_performance
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.size_core
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
        conditions.freestream.altitude                    = np.atleast_2d(turbofan.design_altitude)
        conditions.freestream.mach_number                 = np.atleast_2d(turbofan.design_mach_number)
        conditions.freestream.pressure                    = np.atleast_2d(p)
        conditions.freestream.temperature                 = np.atleast_2d(T)
        conditions.freestream.density                     = np.atleast_2d(rho)
        conditions.freestream.dynamic_viscosity           = np.atleast_2d(mu)
        conditions.freestream.gravity                     = np.atleast_2d(planet.compute_gravity(turbofan.design_altitude))
        conditions.freestream.isentropic_expansion_factor = np.atleast_2d(turbofan.working_fluid.compute_gamma(T,p))
        conditions.freestream.Cp                          = np.atleast_2d(turbofan.working_fluid.compute_cp(T,p))
        conditions.freestream.R                           = np.atleast_2d(turbofan.working_fluid.gas_specific_constant)
        conditions.freestream.speed_of_sound              = np.atleast_2d(a)
        conditions.freestream.velocity                    = np.atleast_2d(U) 
     
    segment                  = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions = conditions 
    turbofan.append_operating_conditions(segment)
                    
    ram                       = turbofan.ram
    inlet_nozzle              = turbofan.inlet_nozzle
    fan                       = turbofan.fan
    low_pressure_compressor   = turbofan.low_pressure_compressor
    high_pressure_compressor  = turbofan.high_pressure_compressor
    combustor                 = turbofan.combustor
    high_pressure_turbine     = turbofan.high_pressure_turbine
    low_pressure_turbine      = turbofan.low_pressure_turbine
    integrated_drive_generator= turbofan.integrated_drive_generator 
    integrated_drive_motor    = turbofan.integrated_drive_motor
    core_nozzle               = turbofan.core_nozzle
    fan_nozzle                = turbofan.fan_nozzle  
    bypass_ratio              = turbofan.bypass_ratio 
    design_power_offtake      = turbofan.design_power_offtake 

    # unpack component conditions
    turbofan_conditions     = conditions.energy.propulsors[turbofan.tag]
    ram_conditions          = conditions.energy.converters[ram.tag]    
    inlet_nozzle_conditions = conditions.energy.converters[inlet_nozzle.tag]
    fan_conditions          = conditions.energy.converters[fan.tag]    
    lpc_conditions          = conditions.energy.converters[low_pressure_compressor.tag]
    hpc_conditions          = conditions.energy.converters[high_pressure_compressor.tag]
    combustor_conditions    = conditions.energy.converters[combustor.tag] 
    lpt_conditions          = conditions.energy.converters[low_pressure_turbine.tag]
    hpt_conditions          = conditions.energy.converters[high_pressure_turbine.tag]
    core_nozzle_conditions  = conditions.energy.converters[core_nozzle.tag]
    fan_nozzle_conditions   = conditions.energy.converters[fan_nozzle.tag]

    # instantiate dummy network 
    dummy_network          = RCAIDE.Framework.Networks.Fuel()
    fuel_line              = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dummy_network.distributors.append(fuel_line)     
     
    # Step 1: Set the working fluid to determine the fluid properties
    ram.working_fluid                             = turbofan.working_fluid
    
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
    
    # Step 5: Link the fan to the inlet nozzle
    fan_conditions.inputs.stagnation_temperature                      = inlet_nozzle_conditions.outputs.stagnation_temperature
    fan_conditions.inputs.stagnation_pressure                         = inlet_nozzle_conditions.outputs.stagnation_pressure
    fan_conditions.inputs.static_temperature                          = inlet_nozzle_conditions.outputs.static_temperature
    fan_conditions.inputs.static_pressure                             = inlet_nozzle_conditions.outputs.static_pressure
    fan_conditions.inputs.mach_number                                 = inlet_nozzle_conditions.outputs.mach_number  
    fan.working_fluid                                                 = inlet_nozzle.working_fluid
     
    # Step 6: Compute flow through the fan
    compute_fan_performance(fan,conditions)    

    # Step 7: Link low pressure compressor to the inlet nozzle
    lpc_conditions.inputs.stagnation_temperature                      = fan_conditions.outputs.stagnation_temperature
    lpc_conditions.inputs.stagnation_pressure                         = fan_conditions.outputs.stagnation_pressure
    lpc_conditions.inputs.static_temperature                          = fan_conditions.outputs.static_temperature
    lpc_conditions.inputs.static_pressure                             = fan_conditions.outputs.static_pressure
    lpc_conditions.inputs.mach_number                                 = fan_conditions.outputs.mach_number  
    low_pressure_compressor.working_fluid                             = fan.working_fluid
    low_pressure_compressor.reference_temperature                     = turbofan.reference_temperature
    low_pressure_compressor.reference_pressure                        = turbofan.reference_pressure
    
    # Step 8: Compute flow through the low pressure compressor
    compute_compressor_performance(low_pressure_compressor,conditions)
    
    # Step 9: Link the high pressure compressor to the low pressure compressor
    hpc_conditions.inputs.stagnation_temperature                      = lpc_conditions.outputs.stagnation_temperature
    hpc_conditions.inputs.stagnation_pressure                         = lpc_conditions.outputs.stagnation_pressure
    hpc_conditions.inputs.static_temperature                          = lpc_conditions.outputs.static_temperature
    hpc_conditions.inputs.static_pressure                             = lpc_conditions.outputs.static_pressure
    hpc_conditions.inputs.mach_number                                 = lpc_conditions.outputs.mach_number  
    high_pressure_compressor.working_fluid                            = low_pressure_compressor.working_fluid  
    high_pressure_compressor.reference_temperature                    = turbofan.reference_temperature
    high_pressure_compressor.reference_pressure                       = turbofan.reference_pressure  
    
    # Step 10: Compute flow through the high pressure compressor
    compute_compressor_performance(high_pressure_compressor,conditions)

    # Step 11: Link the combustor to the high pressure compressor    
    combustor_conditions.inputs.stagnation_temperature                = hpc_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure                   = hpc_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature                    = hpc_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure                       = hpc_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number                           = hpc_conditions.outputs.mach_number  
    combustor.working_fluid                                           = high_pressure_compressor.working_fluid     
    
    # Step 12: Compute flow through the high pressor compressor 
    compute_combustor_performance(combustor,conditions) 
       
    net_external_shaft_power  = np.array([[0.0]])
    
    # Design and size integrated drive motor (parallel hybrid)
    if integrated_drive_motor != None: 
        motor_electrical_power = design_power_offtake
        motor_mechanical_power = motor_electrical_power * integrated_drive_motor.efficiency
        integrated_drive_motor.design_power             = motor_electrical_power
        integrated_drive_motor.design_angular_velocity  = low_pressure_compressor.design_angular_velocity
        integrated_drive_motor.design_torque            = motor_mechanical_power / integrated_drive_motor.design_angular_velocity  
        design_optimal_motor(integrated_drive_motor) 
        net_external_shaft_power -= motor_mechanical_power

    # Design and size integrated drive generator (IDG)
    if integrated_drive_generator != None:
        gen_mechanical_power = design_power_offtake / integrated_drive_generator.efficiency
        integrated_drive_generator.design_power             = design_power_offtake
        integrated_drive_generator.design_angular_velocity  = low_pressure_compressor.design_angular_velocity
        integrated_drive_generator.design_torque            = gen_mechanical_power / integrated_drive_generator.design_angular_velocity
        integrated_drive_generator.design_current           = design_power_offtake / integrated_drive_generator.nominal_voltage if integrated_drive_generator.nominal_voltage > 0 else 0.0
        if integrated_drive_generator.voltage_type == 'DC' and integrated_drive_generator.nominal_voltage > 0:
            design_optimal_generator(integrated_drive_generator)
        net_external_shaft_power += gen_mechanical_power

    has_shaft_offtake = (integrated_drive_motor is not None) or (integrated_drive_generator is not None)
 
    mass_flow_rate_estimate = None
    max_offtake_iterations  = 5 if has_shaft_offtake else 1
    for offtake_iteration in range(max_offtake_iterations):
        if has_shaft_offtake and mass_flow_rate_estimate is not None:
            external_shaft_work = net_external_shaft_power / mass_flow_rate_estimate
        else:
            external_shaft_work = np.array([[0.0]])

        # Step 13: Link the high pressure turbine to the combustor
        hpt_conditions.inputs.stagnation_temperature    = combustor_conditions.outputs.stagnation_temperature
        hpt_conditions.inputs.stagnation_pressure       = combustor_conditions.outputs.stagnation_pressure
        hpt_conditions.inputs.fuel_to_air_ratio         = combustor_conditions.outputs.fuel_to_air_ratio
        hpt_conditions.inputs.static_temperature        = combustor_conditions.outputs.static_temperature
        hpt_conditions.inputs.static_pressure           = combustor_conditions.outputs.static_pressure
        hpt_conditions.inputs.mach_number               = combustor_conditions.outputs.mach_number
        hpt_conditions.inputs.compressor                = hpc_conditions.outputs
        hpt_conditions.inputs.bypass_ratio              = 0.0
        hpt_conditions.inputs.external_shaft.work_done  = external_shaft_work
        high_pressure_turbine.working_fluid             = combustor.working_fluid

        # Step 14: Compute flow through the high pressure turbine
        compute_turbine_performance(high_pressure_turbine,conditions)

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
        compute_turbine_performance(low_pressure_turbine,conditions)

        # Step 17: Link the core nozzle to the low pressure turbine
        core_nozzle_conditions.inputs.stagnation_temperature     = lpt_conditions.outputs.stagnation_temperature
        core_nozzle_conditions.inputs.stagnation_pressure        = lpt_conditions.outputs.stagnation_pressure
        core_nozzle_conditions.inputs.static_temperature         = lpt_conditions.outputs.static_temperature
        core_nozzle_conditions.inputs.static_pressure            = lpt_conditions.outputs.static_pressure
        core_nozzle_conditions.inputs.mach_number                = lpt_conditions.outputs.mach_number
        core_nozzle.working_fluid                                = low_pressure_turbine.working_fluid

        # Step 18: Compute flow through the core nozzle
        compute_expansion_nozzle_performance(core_nozzle,conditions)

        # Step 19: Link the fan nozzle to the fan
        fan_nozzle_conditions.inputs.stagnation_temperature     = fan_conditions.outputs.stagnation_temperature
        fan_nozzle_conditions.inputs.stagnation_pressure        = fan_conditions.outputs.stagnation_pressure
        fan_nozzle_conditions.inputs.static_temperature         = fan_conditions.outputs.static_temperature
        fan_nozzle_conditions.inputs.static_pressure            = fan_conditions.outputs.static_pressure
        fan_nozzle_conditions.inputs.mach_number                = fan_conditions.outputs.mach_number
        fan_nozzle.working_fluid                                = fan.working_fluid

        # Step 20: Compute flow through the fan nozzle
        compute_expansion_nozzle_performance(fan_nozzle,conditions)

        # Step 21: Link the turbofan to outputs from various Components
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
        size_core(turbofan,conditions)

        if not has_shaft_offtake:
            break
        new_mass_flow_rate_estimate = float(np.ravel(turbofan.design_mass_flow_rate)[0])
        if mass_flow_rate_estimate is not None and \
                abs(new_mass_flow_rate_estimate - mass_flow_rate_estimate) < 1e-4 * new_mass_flow_rate_estimate:
            mass_flow_rate_estimate = new_mass_flow_rate_estimate
            break
        mass_flow_rate_estimate = new_mass_flow_rate_estimate

    # Specific (not absolute) shaft work [J/kg] at the converged design point -- read back
    # by design_turbofan_offdesign_matching(), which cannot get this from a fresh
    # compute_performance() call instead: that dispatch only computes motor/generator power
    # when state.numerics.time.differentiate is non-empty (a real mission-segment time
    # discretization), which a synthetic single-point verification state doesn't have, so
    # the offtake would silently read back as zero there.
    turbofan.design_shaft_work_specific = float(np.ravel(external_shaft_work)[0])

    # Step 23: Static Sea Level Thrust
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state       = setup_operating_conditions(turbofan,fuel_line,velocity_range=np.array([V]), altitude = 0, angle_of_attack=0, temperature_deviation=0)  
    operating_state.conditions.energy.propulsors[turbofan.tag].throttle[:,0] = 1.0  
    operating_state.unknowns.network['electrical_power'] = np.array([[design_power_offtake]])
    inputs,outputs,_,_                     = turbofan.compute_performance(operating_state,dummy_network) 
    turbofan.sealevel_static_thrust        = outputs.thrust[0][0]
    turbofan.sealevel_static_power         = outputs.power.propulsive[0][0]
    turbofan.design_power                  = design_power_offtake
 
    return 