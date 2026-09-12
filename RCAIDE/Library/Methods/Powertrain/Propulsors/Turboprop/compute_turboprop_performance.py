# RCAIDE/Methods/Energy/Propulsors/Networks/Turboprop/compute_turboprop_performance.py
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
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.Turboprop_OffDesign_Matching import (
    solve_turboprop_offdesign_robust, OffDesignMatchingError)

# python imports 
from   copy import deepcopy
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# compute_turboprop_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_turboprop_performance(turboprop, state, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes the performance of a turboprop engine by analyzing the thermodynamic cycle.
    
    Parameters
    ----------
    turboprop : RCAIDE.Library.Components.Propulsors.Turboprop
        Turboprop engine component with the following attributes:
            - tag : str
                Identifier for the turboprop
            - working_fluid : Data
                Working fluid properties object
            - ram : Data
                Ram component
                    - tag : str
                        Identifier for the ram
            - inlet_nozzle : Data
                Inlet nozzle component
                    - tag : str
                        Identifier for the inlet nozzle
            - compressor : Data
                Compressor component
                    - tag : str
                        Identifier for the compressor
                    - design_angular_velocity : float
                        Design angular velocity [rad/s]
            - integrated_drive_motor : Data, optional
                Electric motor on the compressor shaft (parallel-hybrid assist),
                same convention as Turbofan.integrated_drive_motor
            - integrated_drive_generator : Data, optional
                Electric generator on the compressor shaft (shaft power extraction),
                same convention as Turbofan.integrated_drive_generator
            - combustor : Data
                Combustor component
                    - tag : str
                        Identifier for the combustor
                    - fuel_data : Data
                        Fuel properties
                        - specific_energy : float
                            Fuel specific energy [J/kg]
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
            - reference_temperature : float
                Reference temperature for mass flow scaling [K]
            - reference_pressure : float
                Reference pressure for mass flow scaling [Pa]
            - compressor_nondimensional_massflow : float
                Non-dimensional mass flow parameter [kg·√K/(s·Pa)]
            - origin : list of lists
                Origin coordinates [[x, y, z]] [m]
    state : RCAIDE.Framework.Mission.Common.State
        State object containing:
            - conditions : Data
                Flight conditions
                    - freestream : Data
                        Freestream properties
                            - velocity : numpy.ndarray
                                Freestream velocity [m/s]
                            - temperature : numpy.ndarray
                                Freestream temperature [K]
                            - pressure : numpy.ndarray
                                Freestream pressure [Pa]
                    - noise : Data
                        Noise conditions
                            - propulsors : dict
                                Propulsor noise conditions indexed by tag
                    - energy : Data
                        Energy conditions
                            - propulsors : dict
                                Propulsor energy conditions indexed by tag
                            - converters : dict
                                Converter energy conditions indexed by tag
                            - hybrid_power_split_ratio : float
                                Ratio of power split for hybrid systems
            - numerics : Data
                Numerical properties
                    - time : Data
                        Time properties
                            - differentiate : list
                                List of differentiation methods
    center_of_gravity : list of lists, optional
        Center of gravity coordinates [[x, y, z]] [m]
        Default: [[0.0, 0.0, 0.0]]
    
    Returns
    -------
    inputs : Data
        Turboprop input conditions (power.electrical/mechanical/etc.)
    outputs : Data
        Turboprop output conditions (thrust, moment, power.propulsive, etc.)
    stored_results_flag : bool
        Flag indicating if results are stored
    stored_propulsor_tag : str
        Tag of the turboprop with stored results
    
    Notes
    -----
    This function computes the performance of a turboprop engine by sequentially analyzing
    each component in the engine's thermodynamic cycle. It links the output conditions of
    each component to the input conditions of the next component in the flow path.
    
    The function follows this sequence:
        1. Set working fluid properties
        2. Compute ram performance
        3. Compute inlet nozzle performance
        4. Compute compressor performance
        5. Compute combustor performance
        6. Compute high pressure turbine performance
        7. Compute low pressure turbine performance
        8. Compute core nozzle performance
        9. Compute thrust and power output
        10. Calculate efficiencies
        11. Handle electrical power generation/consumption if applicable
    
    **Major Assumptions**
        * Steady state operation
        * One-dimensional flow through components
        * Adiabatic components except for the combustor
        * Perfect gas behavior with variable properties
    
    References
    ----------
    [1] Mattingly, J.D., "Elements of Gas Turbine Propulsion", 2nd Edition, AIAA Education Series, 2005. https://soaneemrana.org/onewebmedia/ELEMENTS%20OF%20GAS%20TURBINE%20PROPULTION2.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_thrust
    """
    if turboprop.offdesign_matching is not None:
        return compute_turboprop_performance_offdesign(turboprop, state, center_of_gravity)

    # else: analytical cycle model (below)

    conditions               = state.conditions
    noise_conditions         = conditions.aeroacoustics.propulsors[turboprop.tag]  
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

    # ----------------------------------------------------------------------------
    # Compute Externally Supplied/Delivered Shaft Power from Electric Motors or Generators
    # ----------------------------------------------------------------------------
    external_shaft_work         = 0*state.ones_row(1)
    integrated_drive_motor      = turboprop.integrated_drive_motor
    integrated_drive_generator  = turboprop.integrated_drive_generator
    compressor_conditions.omega = compressor.design_angular_velocity * turboprop_conditions.throttle

    # Motor: consumes electrical power from the bus, delivers mechanical power to the shaft
    if integrated_drive_motor != None and len(state.numerics.time.differentiate) > 0:
        motor_conditions = conditions.energy.converters[integrated_drive_motor.tag]
        phi = conditions.energy.hybrid_power_split_ratio
        if 'electrical_power' in state.unknowns.network:
            motor_electrical_power = state.unknowns.network['electrical_power'] * phi
        else:
            motor_electrical_power = conditions.energy.inputs.power.electrical * phi

        eta_motor = integrated_drive_motor.efficiency
        motor_mechanical_power = motor_electrical_power * eta_motor

        motor_conditions.inputs.power.electrical  = motor_electrical_power
        motor_conditions.outputs.power.mechanical = motor_mechanical_power
        motor_conditions.outputs.omega            = compressor_conditions.omega
        motor_conditions.outputs.torque           = motor_mechanical_power / compressor_conditions.omega

        # Motor delivers power to shaft (negative = reduces turbine burden)
        external_shaft_work -= motor_mechanical_power

    # Generator: extracts mechanical power from the shaft, provides electrical to bus
    if integrated_drive_generator != None and len(state.numerics.time.differentiate) > 0:
        gen_conditions = conditions.energy.converters[integrated_drive_generator.tag]
        if 'electrical_power' in state.unknowns.network:
            gen_electrical_power = state.unknowns.network['electrical_power'] * integrated_drive_generator.power_split_ratio
        else:
            gen_electrical_power = conditions.energy.inputs.power.electrical * integrated_drive_generator.power_split_ratio

        eta_gen = integrated_drive_generator.efficiency
        gen_mechanical_power = gen_electrical_power / eta_gen

        gen_conditions.outputs.power.electrical  = gen_electrical_power
        gen_conditions.inputs.power.mechanical   = gen_mechanical_power
        gen_conditions.inputs.omega              = compressor_conditions.omega
        gen_conditions.inputs.torque              = gen_mechanical_power / compressor_conditions.omega

        # Generator extracts mechanical power from the shaft (positive = more turbine work needed)
        external_shaft_work += gen_mechanical_power

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
    compressor_conditions.reference_temperature           = turboprop.reference_temperature
    compressor_conditions.reference_pressure              = turboprop.reference_pressure
    
    # Step 6: Compute flow through the low pressure compressor
    compute_compressor_performance(compressor,conditions)

    # Shaft work as specific work [J/kg], not absolute power [W] -- same conversion as
    # design_turbofan.py's IDG/motor fix; non-iterative since mass flow is already fixed.
    mdot_core_for_shaft_work = turboprop.compressor_nondimensional_massflow * \
        np.sqrt(turboprop.reference_temperature / compressor_conditions.inputs.stagnation_temperature) * \
        (compressor_conditions.inputs.stagnation_pressure / turboprop.reference_pressure)
    lpt_shaft_work = turboprop.design_power / mdot_core_for_shaft_work if turboprop.design_power != 0.0 \
        else 0 * mdot_core_for_shaft_work
    # Gas-generator accessory offtake, same conversion, computed above (absolute power)
    hpt_shaft_work = external_shaft_work / mdot_core_for_shaft_work

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
    hpt_conditions.inputs.external_shaft.work_done        = hpt_shaft_work

    compute_turbine_performance(high_pressure_turbine,conditions)
    
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

    # Compute the power
    compute_thrust(turboprop,conditions) 

    # Compute forces and moments
    moment_vector      = 0*state.ones_row(3)
    thrust_vector      = 0*state.ones_row(3)
    thrust_vector[:,0] = turboprop_conditions.thrust[:,0]
    moment_vector[:,0] = turboprop.origin[0][0] -   center_of_gravity[0][0] 
    moment_vector[:,1] = turboprop.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2] = turboprop.origin[0][2]  -  center_of_gravity[0][2]
    M                  = np.cross(moment_vector, thrust_vector)   
    moment             = M 
    power              = turboprop_conditions.power 
  
    # compute efficiencies 
    mdot_air_core                                  = turboprop_conditions.core_mass_flow_rate 
    fuel_enthalpy                                  = combustor.fuel_data.specific_energy 
    mdot_fuel                                      = turboprop_conditions.fuel_mass_flow_rate   
    h_e_c                                          = core_nozzle_conditions.outputs.static_enthalpy
    h_0                                            = turboprop.working_fluid.compute_cp(T,P) * T 
    h_t4                                           = combustor_conditions.outputs.stagnation_enthalpy
    h_t3                                           = compressor_conditions.outputs.stagnation_enthalpy 
    turboprop_conditions.overall_efficiency        = turboprop_conditions.thrust[:, 0]* U0 / (mdot_fuel * fuel_enthalpy)  
    turboprop_conditions.thermal_efficiency        = 1 - ((mdot_air_core +  mdot_fuel)*(h_e_c -  h_0) + mdot_fuel *h_0)/((mdot_air_core +  mdot_fuel)*h_t4 - mdot_air_core *h_t3)   
    # power_elec_in/out: same motor/generator electrical power already computed in the
    # offtake block near the top of this function (reused here, not recomputed)
    power_elec_in  = motor_electrical_power if integrated_drive_motor != None and \
        len(state.numerics.time.differentiate) > 0 else 0*state.ones_row(1)
    power_elec_out = gen_electrical_power if integrated_drive_generator != None and \
        len(state.numerics.time.differentiate) > 0 else 0*state.ones_row(1)

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

    stored_results_flag            = True
    stored_propulsor_tag           = turboprop.tag  

    turboprop_conditions.outputs.thrust                 = thrust_vector
    turboprop_conditions.outputs.moment                 = moment
    turboprop_conditions.outputs.power.propulsive       = power
    turboprop_conditions.outputs.power.electrical       = power_elec_out
    turboprop_conditions.inputs.power.electrical        = power_elec_in
    turboprop_conditions.inputs.power.chemical          = mdot_fuel * combustor.fuel_data.lower_heating_value # negative because it is consumed power

    return turboprop_conditions.inputs ,turboprop_conditions.outputs, stored_results_flag,stored_propulsor_tag 

def reuse_stored_turboprop_data(turboprop,state,network,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
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
    compressor                  = turboprop.compressor 
    combustor                   = turboprop.combustor
    high_pressure_turbine       = turboprop.high_pressure_turbine
    low_pressure_turbine        = turboprop.low_pressure_turbine
    core_nozzle                 = turboprop.core_nozzle
    ram_0                       = network.propulsors[stored_propulsor_tag].ram
    inlet_nozzle_0              = network.propulsors[stored_propulsor_tag].inlet_nozzle 
    compressor_0                = network.propulsors[stored_propulsor_tag].compressor 
    combustor_0                 = network.propulsors[stored_propulsor_tag].combustor
    high_pressure_turbine_0     = network.propulsors[stored_propulsor_tag].high_pressure_turbine
    low_pressure_turbine_0      = network.propulsors[stored_propulsor_tag].low_pressure_turbine
    core_nozzle_0               = network.propulsors[stored_propulsor_tag].core_nozzle

    # deep copy results 
    conditions.energy.propulsors[turboprop.tag]                = deepcopy(conditions.energy.propulsors[stored_propulsor_tag])
    conditions.aeroacoustics.propulsors[turboprop.tag]                 = deepcopy(conditions.aeroacoustics.propulsors[stored_propulsor_tag]) 
    conditions.energy.converters[ram.tag]                      = deepcopy(conditions.energy.converters[ram_0.tag]                     )
    conditions.energy.converters[inlet_nozzle.tag]             = deepcopy(conditions.energy.converters[inlet_nozzle_0.tag]            ) 
    conditions.energy.converters[compressor.tag]               = deepcopy(conditions.energy.converters[compressor_0.tag] ) 
    conditions.energy.converters[combustor.tag]                = deepcopy(conditions.energy.converters[combustor_0.tag]               )
    conditions.energy.converters[low_pressure_turbine.tag]     = deepcopy(conditions.energy.converters[low_pressure_turbine_0.tag]    )
    conditions.energy.converters[high_pressure_turbine.tag]    = deepcopy(conditions.energy.converters[high_pressure_turbine_0.tag]   )
    conditions.energy.converters[core_nozzle.tag]              = deepcopy(conditions.energy.converters[core_nozzle_0.tag]             )

    # compute moment  
    moment_vector      = 0*state.ones_row(3)
    thrust_vector      = 0*state.ones_row(3)
    thrust_vector[:,0] = conditions.energy.propulsors[turboprop.tag].thrust[:,0] 
    moment_vector[:,0] = turboprop.origin[0][0] -   center_of_gravity[0][0] 
    moment_vector[:,1] = turboprop.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2] = turboprop.origin[0][2]  -  center_of_gravity[0][2]
    moment             = np.cross(moment_vector,thrust_vector)    

    power                                              = conditions.energy.propulsors[turboprop.tag].outputs.power.propulsive
    conditions.energy.propulsors[turboprop.tag].moment = moment
    
    power_elec_in  = 0*state.ones_row(1) # drawn from the bus (motor)
    power_elec_out = 0*state.ones_row(1) # supplied to the bus (generator)
    idm   = turboprop.integrated_drive_motor
    idm_0 = network.propulsors[stored_propulsor_tag].integrated_drive_motor
    if idm != None and  len(state.numerics.time.differentiate) > 0:
        conditions.energy.converters[idm.tag]  = deepcopy(conditions.energy.converters[idm_0.tag])
        power_elec_in =  conditions.energy.converters[idm.tag].inputs.power.electrical

    idg   = turboprop.integrated_drive_generator
    idg_0 = network.propulsors[stored_propulsor_tag].integrated_drive_generator
    if idg != None and len(state.numerics.time.differentiate) > 0:
        conditions.energy.converters[idg.tag]  = deepcopy(conditions.energy.converters[idg_0.tag])
        power_elec_out =  conditions.energy.converters[idg.tag].outputs.power.electrical

    conditions.energy.propulsors[turboprop.tag].outputs.power.electrical  = power_elec_out
    conditions.energy.propulsors[turboprop.tag].inputs.power.electrical   = power_elec_in
    conditions.energy.propulsors[turboprop.tag].outputs.moment            = moment
    conditions.energy.propulsors[turboprop.tag].outputs.thrust            = thrust_vector  

    return conditions.energy.propulsors[turboprop.tag].inputs, conditions.energy.propulsors[turboprop.tag].outputs


# ----------------------------------------------------------------------------------------------------------------------
#  compute_turboprop_performance_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def compute_turboprop_performance_offdesign(turboprop, state, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes turboprop thrust and fuel flow by live off-design component
    matching (`Turboprop_OffDesign_Matching.solve_turboprop_offdesign_robust`).
    Dispatched from `compute_turboprop_performance` when
    `turboprop.offdesign_matching` is not None.

    Parameters
    ----------
    turboprop : RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop
        Must have `turboprop.offdesign_matching` set to
        `Data(design_constants=..., reference_point=...)`, from
        `design_turboprop_offdesign_matching`.
    state : RCAIDE.Framework.Mission.Common.State
        Must have `conditions.freestream.mach_number/temperature/pressure`,
        and `conditions.energy.propulsors[turboprop.tag].throttle`.

    Returns
    -------
    inputs, outputs, stored_results_flag, stored_propulsor_tag
        Same return signature as `compute_turboprop_performance`, for a
        common call site in `Turboprop.compute_performance`.

    Raises
    ------
    OffDesignMatchingError
        If the matching solver fails to converge at a control point AND
        `turboprop.offdesign_matching.idle_fallback` is not set. With no
        fallback configured, this fails loudly rather than silently
        substituting an approximation. If `idle_fallback` (a built
        `Turbofan_Surrogate` -- see `generate_turboprop_deck`; the
        interpolator itself has no turbofan-specific coupling, so it's
        reused as-is) is set, a point that fails to converge is routed to it
        instead of raising -- same mechanism as `compute_turbofan_
        performance_offdesign`'s own `idle_fallback`. Points routed there
        have no shaft-power figure (the surrogate's schema doesn't carry
        one) -- `shaft_power` reads NaN for exactly those points.

    Notes
    -----
    Throttle is consumed the same way `compute_turbofan_performance_
    offdesign` does: `Tt4 = reference_point.Tt4 * throttle`.

    There is no fan nozzle at all for a turboprop (all core flow exits
    through the single core nozzle; the propeller's own thrust is folded
    into the work-coefficient thrust formula, not modeled as a separate
    stream) -- `noise_conditions` carries only `core_nozzle`, matching
    `compute_turboprop_performance`'s own schema.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.Turboprop_OffDesign_Matching.solve_turboprop_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.design_turboprop_offdesign_matching
    """
    conditions           = state.conditions
    turboprop_conditions = conditions.energy.propulsors[turboprop.tag]
    noise_conditions      = conditions.aeroacoustics.propulsors[turboprop.tag]

    altitude            = conditions.freestream.altitude[:, 0]
    mach_number         = conditions.freestream.mach_number[:, 0]
    static_temperature  = conditions.freestream.temperature[:, 0]
    static_pressure     = conditions.freestream.pressure[:, 0]
    velocity            = conditions.freestream.velocity[:, 0]
    throttle            = turboprop_conditions.throttle[:, 0]

    design_constants = turboprop.offdesign_matching.design_constants
    reference_point  = turboprop.offdesign_matching.reference_point
    idle_fallback    = getattr(turboprop.offdesign_matching, 'idle_fallback', None)

    n = len(mach_number)
    thrust_N              = np.zeros(n)
    fuel_mass_flow_rate    = np.zeros(n)
    shaft_power            = np.zeros(n)
    core_nozzle_exit_velocity               = np.full(n, np.nan)
    core_nozzle_exit_static_temperature     = np.full(n, np.nan)
    core_nozzle_exit_static_pressure        = np.full(n, np.nan)
    core_nozzle_exit_stagnation_temperature = np.full(n, np.nan)
    core_nozzle_exit_stagnation_pressure    = np.full(n, np.nan)

    for i in range(n):
        combustor_exit_temperature = reference_point.Tt4 * throttle[i]
        try:
            result = solve_turboprop_offdesign_robust(
                design_constants, reference_point, mach_number[i], static_temperature[i], static_pressure[i],
                combustor_exit_temperature)
        except OffDesignMatchingError:
            if idle_fallback is None:
                raise
            F, FF = idle_fallback.query(np.array([altitude[i]]), np.array([mach_number[i]]),
                                         throttle=np.array([throttle[i]]))
            thrust_N[i]             = F[0]
            fuel_mass_flow_rate[i]  = FF[0]
            shaft_power[i]          = np.nan
            continue
        thrust_N[i]                                 = result.thrust
        fuel_mass_flow_rate[i]                       = result.fuel_mass_flow_rate
        shaft_power[i]                              = result.power
        core_nozzle_exit_velocity[i]                = result.core_nozzle_exit_velocity
        core_nozzle_exit_static_temperature[i]      = result.core_nozzle_exit_static_temperature
        core_nozzle_exit_static_pressure[i]         = result.core_nozzle_exit_static_pressure
        core_nozzle_exit_stagnation_temperature[i]  = result.core_nozzle_exit_stagnation_temperature
        core_nozzle_exit_stagnation_pressure[i]     = result.core_nozzle_exit_stagnation_pressure

    thrust_vector      = np.zeros((n, 3))
    thrust_vector[:,0] = thrust_N

    TSFC           = np.zeros(n)
    positive       = thrust_N > 0
    gravity        = conditions.freestream.gravity[:, 0] if hasattr(conditions.freestream, 'gravity') \
                     else 9.80665 * np.ones(n)
    TSFC[positive] = fuel_mass_flow_rate[positive] * gravity[positive] / thrust_N[positive]

    power_propulsive = thrust_N * velocity

    # Compute forces and moments
    moment_vector      = 0*state.ones_row(3)
    moment_vector[:,0] = turboprop.origin[0][0] - center_of_gravity[0][0]
    moment_vector[:,1] = turboprop.origin[0][1] - center_of_gravity[0][1]
    moment_vector[:,2] = turboprop.origin[0][2] - center_of_gravity[0][2]
    moment              = np.cross(moment_vector, thrust_vector)

    turboprop_conditions.thrust                            = thrust_vector
    turboprop_conditions.fuel_mass_flow_rate                = fuel_mass_flow_rate.reshape(-1,1)
    turboprop_conditions.thrust_specific_fuel_consumption   = TSFC.reshape(-1,1)
    turboprop_conditions.moment                             = moment
    turboprop_conditions.outputs.thrust                     = thrust_vector
    turboprop_conditions.outputs.moment                     = moment
    turboprop_conditions.outputs.power.propulsive           = power_propulsive.reshape(-1,1)
    turboprop_conditions.outputs.power.mechanical            = shaft_power.reshape(-1,1)

    if turboprop.combustor is not None and turboprop.combustor.fuel_data is not None:
        turboprop_conditions.inputs.power.chemical = \
            (fuel_mass_flow_rate * turboprop.combustor.fuel_data.lower_heating_value).reshape(-1,1)

    noise_conditions.core_nozzle = Data(
        exit_static_temperature      = core_nozzle_exit_static_temperature.reshape(-1,1),
        exit_static_pressure         = core_nozzle_exit_static_pressure.reshape(-1,1),
        exit_stagnation_temperature  = core_nozzle_exit_stagnation_temperature.reshape(-1,1),
        exit_stagnation_pressure     = core_nozzle_exit_stagnation_pressure.reshape(-1,1),
        exit_velocity                = core_nozzle_exit_velocity.reshape(-1,1),
    )

    stored_results_flag   = True
    stored_propulsor_tag  = turboprop.tag

    return turboprop_conditions.inputs, turboprop_conditions.outputs, stored_results_flag, stored_propulsor_tag