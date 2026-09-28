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
from RCAIDE.Library.Methods.Powertrain.Converters.Generator.design_optimal_generator import design_optimal_generator
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.build_turboprop_offdesign_matching import build_turboprop_offdesign_matching
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_actuator_disk_propeller_thrust import propeller_polytropic_efficiency_from_design_point
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.design_turboprop_offdesign_matching import design_turboprop_offdesign_matching

# Python package imports   
import numpy                                                                as np
from scipy.optimize import brentq

# ----------------------------------------------------------------------------------------------------------------------  
#  Design Turboshaft
# ----------------------------------------------------------------------------------------------------------------------   
def design_turboprop(turboprop, build_idle_fallback=True):
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
            - design_thrust : float
                Design-point thrust [N]; solved when only rated_takeoff_power is given
            - rated_takeoff_power : float
                Rated sea-level static takeoff shaft power [W] (see Turboprop)
    build_idle_fallback : bool, optional
        Build the off-design matching model's idle_fallback deck. False while iterating the engine
        size (size_turboprop_to_rated_takeoff_power), where only the design point is needed.

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
            - offdesign_matching : Data
                Off-design matching model (the default performance model)
            - rated_takeoff_throttle : float
                Throttle of the takeoff rating
            - sealevel_static_power : float
                Sea-level static shaft power at the takeoff rating [W]
            - sealevel_static_thrust : float
                Sea-level static thrust at the takeoff rating [N]

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
    # sized from the rated takeoff power alone: solve design_thrust (see size_turboprop_to_rated_takeoff_power)
    if turboprop.rated_takeoff_power > 0 and turboprop.design_thrust <= 0:
        from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.size_turboprop_to_rated_takeoff_power import size_turboprop_to_rated_takeoff_power
        size_turboprop_to_rated_takeoff_power(turboprop)
        return

    # the design point defines the propeller by its design efficiency; its actuator-disk polytropic
    # efficiency is calibrated from it once the engine is sized (Step 25b)
    turboprop.propeller_polytropic_efficiency = None

    #check if mach number and temperature are passed
    if turboprop.design_altitude==None:
        if turboprop.design_mach_number==None and turboprop.design_freestream_velocity ==None:
            raise NameError('The sizing conditions require an altitude and a Mach number or Velocity ')

    if turboprop.low_pressure_turbine.pressure_ratio is None:
        raise NameError('turboprop.low_pressure_turbine.pressure_ratio must be set -- the free '
                         'turbine has no compressor to balance power against, so its design-point '
                         'pressure ratio cannot be derived from design_thrust alone.')

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

    # create dummy distributor for setup_operating_conditions
    fuel_line                                             = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()

    segment                                               = RCAIDE.Framework.Mission.Segments.Segment()
    segment.state.conditions                              = conditions
    turboprop.append_operating_conditions(segment)

    ram                     = turboprop.ram
    inlet_nozzle            = turboprop.inlet_nozzle
    compressor              = turboprop.compressor
    combustor               = turboprop.combustor
    high_pressure_turbine   = turboprop.high_pressure_turbine
    low_pressure_turbine    = turboprop.low_pressure_turbine
    core_nozzle             = turboprop.core_nozzle
    integrated_drive_generator = turboprop.integrated_drive_generator
    integrated_drive_motor     = turboprop.integrated_drive_motor
    design_power_offtake       = turboprop.design_power_offtake


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

    # Two independent shaft-work terms: hpt_shaft_work is the gas-generator accessory tap
    # (design_power_offtake, absolute power, circular via mass flow -- iterated below);
    # lpt_shaft_work (below) is the free turbine's own propeller power, closed directly from
    # low_pressure_turbine.pressure_ratio (specific work, no mass flow involved at all).
    net_external_shaft_power = np.array([[0.0]])

    # Design and size integrated drive motor (parallel hybrid, gas-generator spool)
    if integrated_drive_motor != None:
        motor_electrical_power = design_power_offtake
        motor_mechanical_power = motor_electrical_power * integrated_drive_motor.efficiency
        integrated_drive_motor.design_power             = motor_electrical_power
        integrated_drive_motor.design_angular_velocity  = compressor.design_angular_velocity
        integrated_drive_motor.design_torque            = motor_mechanical_power / integrated_drive_motor.design_angular_velocity
        design_optimal_motor(integrated_drive_motor)
        net_external_shaft_power -= motor_mechanical_power

    # Design and size integrated drive generator (IDG, gas-generator spool)
    if integrated_drive_generator != None:
        gen_mechanical_power = design_power_offtake / integrated_drive_generator.efficiency
        integrated_drive_generator.design_power             = design_power_offtake
        integrated_drive_generator.design_angular_velocity  = compressor.design_angular_velocity
        integrated_drive_generator.design_torque            = gen_mechanical_power / integrated_drive_generator.design_angular_velocity
        integrated_drive_generator.design_current           = design_power_offtake / integrated_drive_generator.nominal_voltage if integrated_drive_generator.nominal_voltage > 0 else 0.0
        if integrated_drive_generator.voltage_type == 'DC' and integrated_drive_generator.nominal_voltage > 0:
            design_optimal_generator(integrated_drive_generator)
        net_external_shaft_power += gen_mechanical_power

    has_gas_generator_offtake = (integrated_drive_motor is not None) or (integrated_drive_generator is not None)

    mass_flow_rate_estimate = None
    max_power_iterations    = 5 if has_gas_generator_offtake else 1
    for power_iteration in range(max_power_iterations):
        if has_gas_generator_offtake and mass_flow_rate_estimate is not None:
            hpt_shaft_work = net_external_shaft_power / mass_flow_rate_estimate
        else:
            hpt_shaft_work = 0.0

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
        hpt_conditions.inputs.external_shaft.work_done        = hpt_shaft_work

        compute_turbine_performance(high_pressure_turbine,conditions)

        T0_lpt_in    = hpt_conditions.outputs.static_temperature
        P0_lpt_in    = hpt_conditions.outputs.static_pressure
        gamma_lpt    = high_pressure_turbine.working_fluid.compute_gamma(T0_lpt_in, P0_lpt_in)
        Cp_lpt       = high_pressure_turbine.working_fluid.compute_cp(T0_lpt_in, P0_lpt_in)
        Tt_in_lpt    = hpt_conditions.outputs.stagnation_temperature
        f            = combustor_conditions.outputs.fuel_to_air_ratio
        tau_tL       = low_pressure_turbine.pressure_ratio ** \
            ((gamma_lpt - 1) * low_pressure_turbine.polytropic_efficiency / gamma_lpt)
        deltah_ht    = Cp_lpt * Tt_in_lpt * (tau_tL - 1)
        lpt_shaft_work = -deltah_ht * (1 + f) * low_pressure_turbine.mechanical_efficiency

        #link the low pressure turbine to the high pressure turbine
        lpt_conditions.inputs.stagnation_temperature          = hpt_conditions.outputs.stagnation_temperature
        lpt_conditions.inputs.stagnation_pressure             = hpt_conditions.outputs.stagnation_pressure
        lpt_conditions.inputs.static_temperature              = hpt_conditions.outputs.static_temperature
        lpt_conditions.inputs.static_pressure                 = hpt_conditions.outputs.static_pressure
        lpt_conditions.inputs.mach_number                     = hpt_conditions.outputs.mach_number
        lpt_conditions.inputs.compressor                      = Data()
        lpt_conditions.inputs.compressor.work_done            = 0.0
        lpt_conditions.inputs.bypass_ratio                    = 0.0
        lpt_conditions.inputs.fuel_to_air_ratio               = combustor_conditions.outputs.fuel_to_air_ratio
        lpt_conditions.inputs.external_shaft.work_done        = lpt_shaft_work
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

        if not has_gas_generator_offtake:
            break
        new_mass_flow_rate_estimate = float(np.ravel(turboprop.design_mass_flow_rate)[0])
        if mass_flow_rate_estimate is not None and \
                abs(new_mass_flow_rate_estimate - mass_flow_rate_estimate) < 1e-4 * new_mass_flow_rate_estimate:
            mass_flow_rate_estimate = new_mass_flow_rate_estimate
            break
        mass_flow_rate_estimate = new_mass_flow_rate_estimate

    # Specific shaft work at the converged design point, same rationale as design_turbofan.py
    turboprop.design_shaft_work_specific = float(np.ravel(hpt_shaft_work)[0]) if has_gas_generator_offtake else 0.0

    # design_power is an output here, not an input: the free turbine's design point is closed
    # by low_pressure_turbine.pressure_ratio above, and this is just that choice's resulting
    # absolute power once the design mass flow rate is known -- for reporting only.
    turboprop.design_power = float(np.ravel(lpt_shaft_work)[0]) * float(np.ravel(turboprop.design_mass_flow_rate)[0])

    # Step 25b: Propeller polytropic efficiency -- the actuator-disk loss factor that gives the propeller its
    # design efficiency at the design point (see compute_actuator_disk_propeller_thrust); compute_thrust
    # and the off-design matching model use it for the propeller thrust everywhere else
    design_propeller_shaft_power = float(np.ravel(turboprop_conditions.propeller_work_output_coefficient)[0]) / turboprop.propeller.design_efficiency * \
        float(np.ravel(compressor_conditions.outputs.cp)[0]) * float(np.ravel(T)[0]) * float(np.ravel(turboprop.design_mass_flow_rate)[0])
    design_density               = float(np.ravel(p)[0]) / (float(np.ravel(compressor_conditions.outputs.gas_constant)[0]) * float(np.ravel(T)[0]))
    turboprop.propeller_polytropic_efficiency = float(propeller_polytropic_efficiency_from_design_point(
        turboprop.propeller.design_efficiency, float(np.ravel(conditions.freestream.velocity)[0]), design_density,
        np.pi*turboprop.propeller.tip_radius**2, design_propeller_shaft_power))

    # Step 26: Off-design matching is the default performance model -- attach it from the sized design
    # point (see design_turbofan)
    turboprop.offdesign_matching      = None
    design_constants, reference_point = design_turboprop_offdesign_matching(turboprop)
    turboprop.offdesign_matching      = Data(design_constants = design_constants, reference_point = reference_point)

    # Step 27: Sea-level static operating point
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01
    operating_state       = setup_operating_conditions(turboprop,fuel_line,velocity_range=np.array([V]), altitude = 0, angle_of_attack=0, temperature_deviation=0)
    operating_state.unknowns.network['electrical_power'] = np.array([[design_power_offtake]])

    def sea_level_static_performance(throttle):
        operating_state.conditions.energy.propulsors[turboprop.tag].throttle[:,0] = throttle
        _,outputs,_,_ = turboprop.compute_performance(operating_state)
        return outputs

    # Step 28: Takeoff rating -- the throttle (fraction of the design combustor exit temperature) at which
    # the sea-level static shaft power equals the rated takeoff power (see design_turbofan)
    if turboprop.rated_takeoff_power > 0:
        def power_residual(throttle):
            return sea_level_static_performance(throttle).power.mechanical[0][0] - turboprop.rated_takeoff_power
        throttle_step   = 0.05
        lower, upper    = 1.0, 1.0
        residual        = power_residual(1.0)
        direction       = 1.0 if residual < 0 else -1.0
        while (residual < 0) == (direction > 0):
            next_throttle = (upper if direction > 0 else lower) + direction * throttle_step
            if not 0.5 <= next_throttle <= 2.0:
                raise ValueError(f"Turboprop '{turboprop.tag}': no takeoff throttle between 0.5 and 2.0 gives the rated takeoff "
                                 f"power of {turboprop.rated_takeoff_power:.0f} W at sea level; check design_thrust against it.")
            residual = power_residual(next_throttle)
            if direction > 0:
                lower, upper = upper, next_throttle
            else:
                lower, upper = next_throttle, lower
        turboprop.rated_takeoff_throttle = brentq(power_residual, lower, upper, xtol=1e-6)
    else:
        turboprop.rated_takeoff_throttle = 1.0

    # idle_fallback deck spans part power up to the takeoff rating
    if build_idle_fallback:
        combustor_exit_temperature_fractions = sorted(set([turboprop.rated_takeoff_throttle, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4]), reverse=True)
        turboprop.offdesign_matching         = None
        turboprop.offdesign_matching         = build_turboprop_offdesign_matching(turboprop, combustor_exit_temperature_fractions = combustor_exit_temperature_fractions)

    # Step 29: Sea-level static shaft power and thrust at the takeoff rating (the propeller as an actuator
    # disk, finite at zero speed)
    sls_outputs                      = sea_level_static_performance(turboprop.rated_takeoff_throttle)
    turboprop.sealevel_static_thrust = sls_outputs.thrust[0][0]
    turboprop.sealevel_static_power  = sls_outputs.power.mechanical[0][0]
    
    turboprop.design_thrust_specific_fuel_consumption = turboprop_conditions.thrust_specific_fuel_consumption  
    turboprop.design_non_dimensional_thrust           = turboprop_conditions.non_dimensional_thrust            
    turboprop.design_core_mass_flow_rate              = turboprop_conditions.core_mass_flow_rate               
    turboprop.design_fuel_flow_rate                   = turboprop_conditions.fuel_mass_flow_rate                           
    turboprop.design_specific_power                   = turboprop_conditions.specific_power                    
    turboprop.design_power_specific_fuel_consumption  = turboprop_conditions.power_specific_fuel_consumption   
    turboprop.design_thermal_efficiency               = turboprop_conditions.thermal_efficiency                
    turboprop.design_propulsive_efficiency            = turboprop_conditions.propulsive_efficiency
    
    if turboprop.integrated_drive_motor != None:
        V                     = turboprop.design_freestream_velocity
        operating_state       = setup_operating_conditions(turboprop,fuel_line,velocity_range=np.array([V]), altitude = turboprop.design_altitude, angle_of_attack=0, temperature_deviation=0)
        operating_state.conditions.energy.propulsors[turboprop.tag].throttle[:,0] = 1.0
        _,outputs,_,_           = turboprop.compute_performance(operating_state)

        T = outputs.thrust
        P = outputs.power.propulsive

        motor                         = turboprop.integrated_drive_motor
        motor.design_torque           = P[0][0] /compressor.design_angular_velocity
        motor.design_angular_velocity = compressor.design_angular_velocity
        motor.mass_properties.mass    = compute_motor_weight(motor)
        design_optimal_motor(motor)
    
    return      
  