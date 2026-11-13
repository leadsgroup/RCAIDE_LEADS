# RCAIDE/Methods/Energy/Propulsors/Networks/Turbojet/compute_turbojet_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core import Data, array_namespace
from RCAIDE.Library.Methods.Utilities import Linear_Interpolator
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor          import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor         import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine            import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Supersonic_Nozzle  import compute_supersonic_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet           import compute_thrust
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common             import compute_gas_generator_shaft_power_offtake, extend_thrust_below_floor
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.Turbojet_OffDesign_Matching import (
    solve_turbojet_offdesign_robust, OffDesignMatchingError, pack_turbojet_design_constants, pack_turbojet_reference_point,
    apply_turbojet_afterburner)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.solve_turbojet_offdesign_jax import solve_turbojet_offdesign_jax

# python imports 
import  numpy as  np 

# ----------------------------------------------------------------------------------------------------------------------
# compute_turbojet_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_turbojet_performance(turbojet, state, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes the performance of a turbojet engine by analyzing the thermodynamic cycle.
    
    Parameters
    ----------
    turbojet : RCAIDE.Library.Components.Propulsors.Turbojet
        Turbojet engine component with the following attributes:
            - tag : str
                Identifier for the turbojet
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
            - low_pressure_compressor : Data
                Low pressure compressor component
                    - tag : str
                        Identifier for the low pressure compressor
                    - design_angular_velocity : float
                        Design angular velocity [rad/s]
            - integrated_drive_motor : Data, optional
                Electric motor on the compressor shaft (parallel-hybrid assist)
            - integrated_drive_generator : Data, optional
                Electric generator on the compressor shaft (shaft power extraction)
            - high_pressure_compressor : Data
                High pressure compressor component
                    - tag : str
                        Identifier for the high pressure compressor
                    - design_angular_velocity : float
                        Design angular velocity [rad/s]
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
            - afterburner : Data
                Afterburner component
                    - tag : str
                        Identifier for the afterburner
            - core_nozzle : Data
                Core nozzle component
                    - tag : str
                        Identifier for the core nozzle
            - afterburner_active : bool
                Flag indicating if afterburner is active
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
            - ones_row : function
                Function to create array of ones with specified length
    center_of_gravity : list of lists, optional
        Center of gravity coordinates [[x, y, z]] [m]. Default: [[0.0, 0.0, 0.0]]
    
    Returns
    -------
    inputs : Data
        Turbojet input conditions (power.electrical/mechanical/etc.)
    outputs : Data
        Turbojet output conditions (thrust, moment, power.propulsive, etc.)
    stored_results_flag : bool
        Flag indicating if results are stored
    stored_propulsor_tag : str
        Tag of the turbojet with stored results
    
    Notes
    -----
    This function computes the performance of a turbojet engine by sequentially analyzing
    each component in the engine's thermodynamic cycle. It links the output conditions of
    each component to the input conditions of the next component in the flow path.
    
    The function follows this sequence:
        1. Set working fluid properties
        2. Compute ram performance
        3. Compute inlet nozzle performance
        4. Compute low pressure compressor performance
        5. Compute high pressure compressor performance
        6. Compute combustor performance
        7. Compute high pressure turbine performance
        8. Compute low pressure turbine performance
        9. Compute afterburner performance (if active)
        10. Compute core nozzle performance
        11. Compute thrust and power output
        12. Calculate efficiencies
        13. Handle electrical power generation/consumption if applicable
    
    **Major Assumptions**
        * Steady state operation
        * One-dimensional flow through components
        * Adiabatic components except for the combustor and afterburner
        * Perfect gas behavior with variable properties
    
    References
    ----------
    [1] Mattingly, J.D., "Elements of Gas Turbine Propulsion", 2nd Edition, AIAA Education Series, 2005. https://soaneemrana.org/onewebmedia/ELEMENTS%20OF%20GAS%20TURBINE%20PROPULTION2.pdf
    [2] Cantwell, B., "AA283 Course Notes", Stanford University. https://web.stanford.edu/~cantwell/AA283_Course_Material/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.compute_thrust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.reuse_stored_turbojet_data
    """
    if turbojet.offdesign_matching is not None:
        return compute_turbojet_performance_offdesign(turbojet, state, center_of_gravity)

    # else: analytical cycle model (below)

    conditions                = state.conditions
    noise_conditions          = conditions.aeroacoustics.propulsors[turbojet.tag]  
    turbojet_conditions       = conditions.energy.propulsors[turbojet.tag] 
    U0                        = conditions.freestream.velocity
    T                         = conditions.freestream.temperature
    P                         = conditions.freestream.pressure
    ram                       = turbojet.ram
    inlet_nozzle              = turbojet.inlet_nozzle
    low_pressure_compressor   = turbojet.low_pressure_compressor
    high_pressure_compressor  = turbojet.high_pressure_compressor
    combustor                 = turbojet.combustor
    high_pressure_turbine     = turbojet.high_pressure_turbine
    low_pressure_turbine      = turbojet.low_pressure_turbine 
    afterburner               = turbojet.afterburner 
    core_nozzle               = turbojet.core_nozzle   

    # unpack component conditions 
    ram_conditions          = conditions.energy.converters[ram.tag]     
    inlet_nozzle_conditions = conditions.energy.converters[inlet_nozzle.tag]
    core_nozzle_conditions  = conditions.energy.converters[core_nozzle.tag] 
    lpc_conditions          = conditions.energy.converters[low_pressure_compressor.tag]
    hpc_conditions          = conditions.energy.converters[high_pressure_compressor.tag]
    lpt_conditions          = conditions.energy.converters[low_pressure_turbine.tag]
    hpt_conditions          = conditions.energy.converters[high_pressure_turbine.tag]
    combustor_conditions    = conditions.energy.converters[combustor.tag]
    afterburner_conditions  = conditions.energy.converters[afterburner.tag]

    # ----------------------------------------------------------------------------
    # Compute Externally Supplied/Delivered Shaft Power from Electric Motors or Generators
    # ----------------------------------------------------------------------------
    # throttle is a fraction of the takeoff rating; the cycle runs on the fraction of the design point
    design_point_throttle        = turbojet_conditions.throttle * turbojet.rated_takeoff_temperature_ratio

    # absolute power [W], converted to specific work with the core mass flow before the HP turbine below
    lpc_conditions.omega         = low_pressure_compressor.design_angular_velocity * design_point_throttle
    external_shaft_work, _, _, _ = compute_gas_generator_shaft_power_offtake(turbojet, state, lpc_conditions.omega)

    # Set the working fluid to determine the fluid properties
    ram.working_fluid = turbojet.working_fluid

    # Flow through the ram , this computes the necessary flow quantities and stores it into conditions
    compute_ram_performance(ram,conditions)

    # Link inlet nozzle to ram 
    inlet_nozzle_conditions.inputs.stagnation_temperature             = ram_conditions.outputs.stagnation_temperature 
    inlet_nozzle_conditions.inputs.stagnation_pressure                = ram_conditions.outputs.stagnation_pressure
    inlet_nozzle_conditions.inputs.static_temperature                 = ram_conditions.outputs.static_temperature
    inlet_nozzle_conditions.inputs.static_pressure                    = ram_conditions.outputs.static_pressure
    inlet_nozzle_conditions.inputs.mach_number                        = ram_conditions.outputs.mach_number
    inlet_nozzle.working_fluid                                        = ram.working_fluid

    # Flow through the inlet nozzle
    compute_compression_nozzle_performance(inlet_nozzle,conditions)

    # Link low pressure compressor to the inlet nozzle
    lpc_conditions.inputs.stagnation_temperature  = inlet_nozzle_conditions.outputs.stagnation_temperature
    lpc_conditions.inputs.stagnation_pressure     = inlet_nozzle_conditions.outputs.stagnation_pressure
    lpc_conditions.inputs.static_temperature      = inlet_nozzle_conditions.outputs.static_temperature
    lpc_conditions.inputs.static_pressure         = inlet_nozzle_conditions.outputs.static_pressure
    lpc_conditions.inputs.mach_number             = inlet_nozzle_conditions.outputs.mach_number  
    low_pressure_compressor.working_fluid         = inlet_nozzle.working_fluid

    # Flow through the low pressure compressor
    compute_compressor_performance(low_pressure_compressor,conditions)

    # Shaft work as specific work [J/kg], not absolute power [W] -- same conversion as
    # design_turbofan.py's IDG/motor fix; non-iterative since mass flow is already fixed.
    mdot_core_for_shaft_work                        = turbojet.compressor_nondimensional_massflow * \
        np.sqrt(turbojet.reference_temperature / lpc_conditions.outputs.stagnation_temperature) * \
        (lpc_conditions.outputs.stagnation_pressure / turbojet.reference_pressure)
    external_shaft_work_specific                     = external_shaft_work / mdot_core_for_shaft_work

    # Link the high pressure compressor to the low pressure compressor
    hpc_conditions.inputs.stagnation_temperature = lpc_conditions.outputs.stagnation_temperature
    hpc_conditions.inputs.stagnation_pressure    = lpc_conditions.outputs.stagnation_pressure
    hpc_conditions.inputs.static_temperature     = lpc_conditions.outputs.static_temperature
    hpc_conditions.inputs.static_pressure        = lpc_conditions.outputs.static_pressure
    hpc_conditions.inputs.mach_number            = lpc_conditions.outputs.mach_number  
    high_pressure_compressor.working_fluid       = low_pressure_compressor.working_fluid    

    # Flow through the high pressure compressor
    compute_compressor_performance(high_pressure_compressor,conditions)

    # Link the combustor to the high pressure compressor 
    combustor_conditions.inputs.stagnation_temperature   = hpc_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure      = hpc_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature       = hpc_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure          = hpc_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number              = hpc_conditions.outputs.mach_number  
    combustor.working_fluid                              = high_pressure_compressor.working_fluid  

    # Flow through the combustor
    compute_combustor_performance(combustor,conditions)

    # Link the high pressure turbine to the combustor
    hpt_conditions.inputs.stagnation_temperature    = combustor_conditions.outputs.stagnation_temperature
    hpt_conditions.inputs.stagnation_pressure       = combustor_conditions.outputs.stagnation_pressure
    hpt_conditions.inputs.fuel_to_air_ratio         = combustor_conditions.outputs.fuel_to_air_ratio 
    hpt_conditions.inputs.static_temperature        = combustor_conditions.outputs.static_temperature
    hpt_conditions.inputs.static_pressure           = combustor_conditions.outputs.static_pressure
    hpt_conditions.inputs.mach_number               = combustor_conditions.outputs.mach_number  
    hpt_conditions.inputs.compressor                = hpc_conditions.outputs
    hpt_conditions.inputs.bypass_ratio              = 0.0
    hpt_conditions.inputs.external_shaft.work_done  = external_shaft_work_specific
    high_pressure_turbine.working_fluid             = combustor.working_fluid

    # Flow through the high pressure turbine
    compute_turbine_performance(high_pressure_turbine,conditions)

    # Link the low pressure turbine to the high pressure turbine
    lpt_conditions.inputs.stagnation_temperature     = hpt_conditions.outputs.stagnation_temperature
    lpt_conditions.inputs.stagnation_pressure        = hpt_conditions.outputs.stagnation_pressure 
    lpt_conditions.inputs.static_temperature         = hpt_conditions.outputs.static_temperature
    lpt_conditions.inputs.static_pressure            = hpt_conditions.outputs.static_pressure 
    lpt_conditions.inputs.mach_number                = hpt_conditions.outputs.mach_number    
    lpt_conditions.inputs.compressor                 = lpc_conditions.outputs
    lpt_conditions.inputs.fuel_to_air_ratio          = combustor_conditions.outputs.fuel_to_air_ratio 
    lpt_conditions.inputs.bypass_ratio               = 0.0
    low_pressure_turbine.working_fluid               = high_pressure_turbine.working_fluid    

    # Flow through the low pressure turbine
    compute_turbine_performance(low_pressure_turbine,conditions)
 

    if turbojet.afterburner_active == True:
        #link the core nozzle to the afterburner
        afterburner_conditions.inputs.stagnation_temperature = lpt_conditions.outputs.stagnation_temperature
        afterburner_conditions.inputs.stagnation_pressure    = lpt_conditions.outputs.stagnation_pressure   
        afterburner_conditions.inputs.nondim_ratio           = 1.0 + combustor_conditions.outputs.fuel_to_air_ratio
        afterburner_conditions.inputs.static_temperature     = lpt_conditions.outputs.static_temperature
        afterburner_conditions.inputs.static_pressure        = lpt_conditions.outputs.static_pressure
        afterburner_conditions.inputs.mach_number            = lpt_conditions.outputs.mach_number  
        afterburner.working_fluid                            = low_pressure_turbine.working_fluid

        #flow through the afterburner 
        compute_combustor_performance(afterburner,conditions)

        #link the core nozzle to the afterburner
        core_nozzle_conditions.inputs.stagnation_temperature = afterburner_conditions.outputs.stagnation_temperature
        core_nozzle_conditions.inputs.stagnation_pressure    = afterburner_conditions.outputs.stagnation_pressure  
        core_nozzle_conditions.inputs.static_temperature     = afterburner_conditions.outputs.static_temperature
        core_nozzle_conditions.inputs.static_pressure        = afterburner_conditions.outputs.static_pressure  
        core_nozzle_conditions.inputs.mach_number            = afterburner_conditions.outputs.mach_number   
        core_nozzle.working_fluid                            = afterburner.working_fluid  

    else:
        #link the core nozzle to the low pressure turbine
        core_nozzle_conditions.inputs.stagnation_temperature = lpt_conditions.outputs.stagnation_temperature
        core_nozzle_conditions.inputs.stagnation_pressure    = lpt_conditions.outputs.stagnation_pressure
        core_nozzle_conditions.inputs.static_temperature     = lpt_conditions.outputs.static_temperature
        core_nozzle_conditions.inputs.static_pressure        = lpt_conditions.outputs.static_pressure  
        core_nozzle_conditions.inputs.mach_number            = lpt_conditions.outputs.mach_number   
        core_nozzle.working_fluid                            = low_pressure_compressor.working_fluid 
 
    # Flow through the core nozzle
    compute_supersonic_nozzle_performance(core_nozzle,conditions) 
 
    # Link the thrust component to the core nozzle 
    turbojet_conditions.core_nozzle_area_ratio                          = core_nozzle_conditions.outputs.area_ratio 
    turbojet_conditions.core_nozzle_static_pressure                     = core_nozzle_conditions.outputs.static_pressure
    turbojet_conditions.core_nozzle_exit_velocity                       = core_nozzle_conditions.outputs.velocity  

    # Link the thrust component to the combustor
    turbojet_conditions.fuel_to_air_ratio                        = combustor_conditions.outputs.fuel_to_air_ratio 
    if turbojet.afterburner_active == True:
        # previous fuel ratio is neglected when the afterburner fuel ratio is calculated
        turbojet_conditions.fuel_to_air_ratio += afterburner_conditions.outputs.fuel_to_air_ratio

    # Link the thrust component to the low pressure compressor 
    turbojet_conditions.total_temperature_reference              = lpc_conditions.outputs.stagnation_temperature
    turbojet_conditions.total_pressure_reference                 = lpc_conditions.outputs.stagnation_pressure 
    turbojet_conditions.flow_through_core                        = 1.0 #scaled constant to turn on core thrust computation  
    
    # Compute the thrust
    compute_thrust(turbojet,conditions)
    
    # Compute forces and moments
    moment_vector              = 0*state.ones_row(3)
    thrust_vector              = 0*state.ones_row(3)
    thrust_vector[:,0]         = turbojet_conditions.thrust[:,0]
    moment_vector[:,0]         = turbojet.origin[0][0] -   center_of_gravity[0][0] 
    moment_vector[:,1]         = turbojet.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]         = turbojet.origin[0][2]  -  center_of_gravity[0][2]
    M                          = np.cross(moment_vector, thrust_vector)   
    moment                     = M 
    power                      = turbojet_conditions.power 
    turbojet_conditions.moment = moment 

    # compute efficiencies 
    mdot_air_core                                  = turbojet_conditions.core_mass_flow_rate 
    fuel_enthalpy                                  = combustor.fuel_data.specific_energy 
    mdot_fuel                                      = turbojet_conditions.fuel_mass_flow_rate   
    h_e_c                                          = core_nozzle_conditions.outputs.static_enthalpy
    h_0                                            = turbojet.working_fluid.compute_cp(T,P) * T 
    h_t4                                           = combustor_conditions.outputs.stagnation_enthalpy
    h_t3                                           = hpc_conditions.outputs.stagnation_enthalpy 
    turbojet_conditions.overall_efficiency         = turbojet_conditions.thrust[:, 0]* U0 / (mdot_fuel * fuel_enthalpy)  
    turbojet_conditions.thermal_efficiency         = 1 - ((mdot_air_core +  mdot_fuel)*(h_e_c -  h_0) + mdot_fuel *h_0)/((mdot_air_core +  mdot_fuel)*h_t4 - mdot_air_core *h_t3)  
 

    # compute shaft RPMs (lpc_conditions.omega already set above, alongside the offtake block)
    hpc_conditions.omega        = high_pressure_compressor.design_angular_velocity * design_point_throttle

    # store data
    core_nozzle_res = Data(
                exit_static_temperature             = core_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = core_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = core_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = core_nozzle_conditions.outputs.static_pressure,
                exit_velocity                       = core_nozzle_conditions.outputs.velocity
            )

    lpc_res = Data(
                angular_velocity    =  lpc_conditions.omega, 
            )    

    noise_conditions.fan_nozzle             = None 
    noise_conditions.core_nozzle            = core_nozzle_res
    noise_conditions.fan                    = lpc_res   
    
    stored_results_flag            = True
    stored_propulsor_tag           = turbojet.tag  

    turbojet_conditions.outputs.thrust                 = thrust_vector
    turbojet_conditions.outputs.moment                 = moment
    turbojet_conditions.outputs.power.propulsive       = power
    turbojet_conditions.inputs.power.chemical          = mdot_fuel * combustor.fuel_data.lower_heating_value # negative because it is consumed power

    return turbojet_conditions.inputs ,turbojet_conditions.outputs, stored_results_flag, stored_propulsor_tag 
    
def reuse_stored_turbojet_data(turbojet,state,network,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one turbojet for identical propulsors
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    turbojet            - turbojet data structure                [-] 
    state               - operating conditions data structure   [-]  
    fuel_line            - fuelline                              [-] 
    total_thrust         - thrust of turbojet group              [N]
    total_power          - power of turbojet group               [W] 

    Outputs:  
    total_thrust         - thrust of turbojet group              [N]
    total_power          - power of turbojet group               [W] 
    
    Properties Used: 
    N.A.        
    '''
    # unpack
    conditions                  = state.conditions 
    ram                         = turbojet.ram
    inlet_nozzle                = turbojet.inlet_nozzle 
    low_pressure_compressor     = turbojet.low_pressure_compressor
    high_pressure_compressor    = turbojet.high_pressure_compressor
    combustor                   = turbojet.combustor
    high_pressure_turbine       = turbojet.high_pressure_turbine
    low_pressure_turbine        = turbojet.low_pressure_turbine
    core_nozzle                 = turbojet.core_nozzle
    ram_0                       = network.propulsors[stored_propulsor_tag].ram
    inlet_nozzle_0              = network.propulsors[stored_propulsor_tag].inlet_nozzle 
    low_pressure_compressor_0   = network.propulsors[stored_propulsor_tag].low_pressure_compressor
    high_pressure_compressor_0  = network.propulsors[stored_propulsor_tag].high_pressure_compressor
    combustor_0                 = network.propulsors[stored_propulsor_tag].combustor
    high_pressure_turbine_0     = network.propulsors[stored_propulsor_tag].high_pressure_turbine
    low_pressure_turbine_0      = network.propulsors[stored_propulsor_tag].low_pressure_turbine
    core_nozzle_0               = network.propulsors[stored_propulsor_tag].core_nozzle

    # copy the stored propulsor's results, reusing memory
    conditions.energy.propulsors[turbojet.tag].copy_values(conditions.energy.propulsors[stored_propulsor_tag])
    conditions.aeroacoustics.propulsors[turbojet.tag].copy_values(conditions.aeroacoustics.propulsors[stored_propulsor_tag])
    conditions.energy.converters[ram.tag].copy_values(conditions.energy.converters[ram_0.tag])
    conditions.energy.converters[inlet_nozzle.tag].copy_values(conditions.energy.converters[inlet_nozzle_0.tag])
    conditions.energy.converters[low_pressure_compressor.tag].copy_values(conditions.energy.converters[low_pressure_compressor_0.tag])
    conditions.energy.converters[high_pressure_compressor.tag].copy_values(conditions.energy.converters[high_pressure_compressor_0.tag])
    conditions.energy.converters[combustor.tag].copy_values(conditions.energy.converters[combustor_0.tag])
    conditions.energy.converters[low_pressure_turbine.tag].copy_values(conditions.energy.converters[low_pressure_turbine_0.tag])
    conditions.energy.converters[high_pressure_turbine.tag].copy_values(conditions.energy.converters[high_pressure_turbine_0.tag])
    conditions.energy.converters[core_nozzle.tag].copy_values(conditions.energy.converters[core_nozzle_0.tag])
    # compute moment  
    thrust             = conditions.energy.propulsors[turbojet.tag].thrust[:,0]
    xp                 = array_namespace(thrust)
    zero               = 0 * thrust
    thrust_vector      = xp.stack([thrust, zero, zero], axis=1)
    moment_vector      = xp.stack([zero + (turbojet.origin[0][0] - center_of_gravity[0][0]),
                                   zero + (turbojet.origin[0][1] - center_of_gravity[0][1]),
                                   zero + (turbojet.origin[0][2] - center_of_gravity[0][2])], axis=1)
    moment             = xp.cross(moment_vector, thrust_vector)
  
    power                                             = conditions.energy.propulsors[turbojet.tag].power 
    conditions.energy.propulsors[turbojet.tag].moment = moment

    idm   = turbojet.integrated_drive_motor
    idm_0 = network.propulsors[stored_propulsor_tag].integrated_drive_motor
    if idm != None and  len(state.numerics.time.differentiate) > 0:
        conditions.energy.converters[idm.tag].copy_values(conditions.energy.converters[idm_0.tag])
        conditions.energy.propulsors[turbojet.tag].inputs.power.electrical = conditions.energy.converters[idm.tag].inputs.power.electrical

    idg   = turbojet.integrated_drive_generator
    idg_0 = network.propulsors[stored_propulsor_tag].integrated_drive_generator
    if idg != None and len(state.numerics.time.differentiate) > 0:
        conditions.energy.converters[idg.tag].copy_values(conditions.energy.converters[idg_0.tag])
        conditions.energy.propulsors[turbojet.tag].outputs.power.electrical = conditions.energy.converters[idg.tag].outputs.power.electrical

    conditions.energy.propulsors[turbojet.tag].outputs.thrust           = thrust_vector
    conditions.energy.propulsors[turbojet.tag].outputs.moment           = moment
    conditions.energy.propulsors[turbojet.tag].outputs.power.propulsive = power

    return conditions.energy.propulsors[turbojet.tag].inputs, conditions.energy.propulsors[turbojet.tag].outputs


# ----------------------------------------------------------------------------------------------------------------------
#  compute_turbojet_performance_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def compute_turbojet_performance_offdesign(turbojet, state, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes turbojet thrust and fuel flow by live off-design component
    matching (`Turbojet_OffDesign_Matching.solve_turbojet_offdesign_robust`
    -- a dedicated turbojet solver, not the turbofan one: see that module's
    docstring for why a turbojet's fully-expanded convergent-divergent core
    nozzle needs different equations from a turbofan's plain convergent
    one). Dispatched from `compute_turbojet_performance` when
    `turbojet.offdesign_matching` is not None.

    Parameters
    ----------
    turbojet : RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet
        Must have `turbojet.offdesign_matching` set to
        `Data(design_constants=..., reference_point=...)`, from
        `design_turbojet_offdesign_matching`.
    state : RCAIDE.Framework.Mission.Common.State
        Must have `conditions.freestream.mach_number/temperature/pressure`,
        and `conditions.energy.propulsors[turbojet.tag].throttle`.

    Returns
    -------
    inputs, outputs, stored_results_flag, stored_propulsor_tag
        Same return signature as `compute_turbojet_performance`, for a
        common call site in `Turbojet.compute_performance`.

    Raises
    ------
    OffDesignMatchingError
        If the matching solver fails to converge at a control point AND
        `turbojet.offdesign_matching.idle_fallback` is not set. With no
        fallback configured, this fails loudly rather than silently
        substituting an approximation -- unchanged default behavior. If
        `idle_fallback` (a built `Turbofan_Surrogate` -- see
        `generate_turbojet_deck`; the interpolator itself has no turbofan-
        specific coupling, so it's reused as-is) is set, a point that fails
        to converge is routed to it instead of raising -- same mechanism as
        `compute_turbofan_performance_offdesign`'s own `idle_fallback`.

    Notes
    -----
    Throttle is consumed the same way `compute_turbofan_performance_
    offdesign` does: `Tt4 = reference_point.Tt4 * rated_takeoff_temperature_ratio * throttle` (see that
    module's own docstring Notes for the caveats).

    There is no fan nozzle at all for a turbojet (all flow exits through the
    single core nozzle) -- `noise_conditions.fan_nozzle` is left `None`,
    matching `compute_turbojet_performance`'s (the analytical model's) own
    schema, not the turbofan dispatcher's.

    Below the matching floor the cycle and fuel flow stay at the floor and the thrust decays from the floor
    thrust toward the ram drag of the airflow (extend_thrust_below_floor), so static thrust and fuel flow never
    turn negative and the mission solver keeps a gradient.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.Turbojet_OffDesign_Matching.solve_turbojet_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.design_turbojet_offdesign_matching
    """
    conditions           = state.conditions
    turbojet_conditions  = conditions.energy.propulsors[turbojet.tag]
    noise_conditions     = conditions.aeroacoustics.propulsors[turbojet.tag]
    xp                   = array_namespace(turbojet_conditions.throttle, conditions.freestream.mach_number)

    # freestream quantities may be column vectors or 1-D arrays, and may hold a single value for all
    # control points -- broadcast each onto the throttle rows (see compute_turbofan_performance_offdesign)
    throttle            = xp.ravel(turbojet_conditions.throttle) * turbojet.rated_takeoff_temperature_ratio  # design-point Tt4 fraction
    n                   = throttle.shape[0]
    altitude            = xp.ravel(conditions.freestream.altitude)    * xp.ones(n)
    mach_number         = xp.ravel(conditions.freestream.mach_number) * xp.ones(n)
    static_temperature  = xp.ravel(conditions.freestream.temperature) * xp.ones(n)
    static_pressure     = xp.ravel(conditions.freestream.pressure)    * xp.ones(n)
    velocity            = xp.ravel(conditions.freestream.velocity)    * xp.ones(n)

    matching_floor   = getattr(turbojet.offdesign_matching, 'matching_floor', None)

    # lowest Tt4 fraction the matching solver represents; below it thrust decays from the floor (extend_thrust_below_floor)
    if matching_floor is not None:
        floor_interpolator = Linear_Interpolator([matching_floor.altitude, matching_floor.mach_number],
                                                 matching_floor.combustor_exit_temperature_fraction)
        floor_fraction     = floor_interpolator(xp.stack([altitude, mach_number], axis=1))
    else:
        floor_fraction     = -np.inf * xp.ones(n)
    below_floor = throttle < floor_fraction

    # gas-generator shaft power to/from the integrated drive motor or generator at the mission's electrical
    # power; the design-point offtake outside a mission
    lpc_conditions       = conditions.energy.converters[turbojet.low_pressure_compressor.tag]
    lpc_conditions.omega = turbojet.low_pressure_compressor.design_angular_velocity * xp.reshape(throttle, (-1, 1))
    external_shaft_power, _, _, in_mission = compute_gas_generator_shaft_power_offtake(turbojet, state, lpc_conditions.omega)
    shaft_power_offtake  = xp.ravel(external_shaft_power) * xp.ones(n) if in_mission else None

    flight = (altitude, mach_number, static_temperature, static_pressure, velocity, throttle, floor_fraction, below_floor, shaft_power_offtake)
    if xp is np:
        result = solve_turbojet_points_numpy(turbojet, *flight)
    else:
        result = solve_turbojet_points_jax(turbojet, *flight)

    thrust_N            = result['thrust']
    fuel_mass_flow_rate = result['fuel_mass_flow_rate']
    zero                = 0 * thrust_N
    thrust_vector       = xp.stack([thrust_N, zero, zero], axis=1)

    positive       = thrust_N > 0
    gravity        = xp.ravel(conditions.freestream.gravity) * xp.ones(n) if hasattr(conditions.freestream, 'gravity') \
                     else 9.80665 * xp.ones(n)
    TSFC           = xp.where(positive, fuel_mass_flow_rate * gravity / xp.where(positive, thrust_N, 1.0), 0.0)

    power_propulsive = thrust_N * velocity

    # Compute forces and moments
    moment_vector  = xp.stack([zero + (turbojet.origin[0][0] - center_of_gravity[0][0]),
                               zero + (turbojet.origin[0][1] - center_of_gravity[0][1]),
                               zero + (turbojet.origin[0][2] - center_of_gravity[0][2])], axis=1)
    moment         = xp.cross(moment_vector, thrust_vector)

    turbojet_conditions.thrust                            = thrust_vector
    turbojet_conditions.fuel_mass_flow_rate               = xp.reshape(fuel_mass_flow_rate, (-1,1))
    turbojet_conditions.thrust_specific_fuel_consumption  = xp.reshape(TSFC, (-1,1))
    turbojet_conditions.moment                            = moment
    turbojet_conditions.outputs.thrust                    = thrust_vector
    turbojet_conditions.outputs.moment                    = moment
    turbojet_conditions.outputs.power.propulsive          = xp.reshape(power_propulsive, (-1,1))
    turbojet_conditions.power                             = xp.reshape(power_propulsive, (-1,1))
    turbojet_conditions.below_matching_floor              = xp.reshape(below_floor.astype(float), (-1,1))

    # same fields the analytical path sets; NaN at idle_fallback points
    turbojet_conditions.flow_through_core           = 1.0
    turbojet_conditions.fuel_to_air_ratio           = xp.reshape(result['fuel_to_air_ratio'], (-1,1))
    turbojet_conditions.total_temperature_reference = xp.reshape(result['tau_r']*result['tau_c']*static_temperature, (-1,1))
    turbojet_conditions.total_pressure_reference    = xp.reshape(result['pi_r']*result['pi_d']*result['pi_c']*static_pressure, (-1,1))

    if turbojet.combustor is not None and turbojet.combustor.fuel_data is not None:
        turbojet_conditions.inputs.power.chemical = \
            xp.reshape(fuel_mass_flow_rate * turbojet.combustor.fuel_data.lower_heating_value, (-1,1))
        # uses specific_energy, not lower_heating_value used just above -- different fields
        turbojet_conditions.overall_efficiency = \
            xp.reshape(thrust_N * velocity / (fuel_mass_flow_rate * turbojet.combustor.fuel_data.specific_energy), (-1,1))
    # thermal_efficiency not set: needs internal enthalpies the offdesign solver doesn't return

    noise_conditions.core_nozzle = Data(
        exit_static_temperature      = xp.reshape(result['core_nozzle_exit_static_temperature'], (-1,1)),
        exit_static_pressure         = xp.reshape(result['core_nozzle_exit_static_pressure'], (-1,1)),
        exit_stagnation_temperature  = xp.reshape(result['core_nozzle_exit_stagnation_temperature'], (-1,1)),
        exit_stagnation_pressure     = xp.reshape(result['core_nozzle_exit_stagnation_pressure'], (-1,1)),
        exit_velocity                = xp.reshape(result['core_nozzle_exit_velocity'], (-1,1)),
    )
    noise_conditions.fan_nozzle = None
    noise_conditions.fan        = Data(angular_velocity = np.nan * xp.ones((n,1)))

    # core nozzle exit state, in the same conditions the analytical cycle populates
    nozzle_outputs                        = conditions.energy.converters[turbojet.core_nozzle.tag].outputs
    nozzle_outputs.velocity               = xp.reshape(result['core_nozzle_exit_velocity'], (-1,1))
    nozzle_outputs.static_temperature     = xp.reshape(result['core_nozzle_exit_static_temperature'], (-1,1))
    nozzle_outputs.static_pressure        = xp.reshape(result['core_nozzle_exit_static_pressure'], (-1,1))
    nozzle_outputs.stagnation_temperature = xp.reshape(result['core_nozzle_exit_stagnation_temperature'], (-1,1))
    nozzle_outputs.stagnation_pressure    = xp.reshape(result['core_nozzle_exit_stagnation_pressure'], (-1,1))

    stored_results_flag   = True
    stored_propulsor_tag  = turbojet.tag

    return turbojet_conditions.inputs, turbojet_conditions.outputs, stored_results_flag, stored_propulsor_tag

# fields of the matched point that compute_turbojet_performance_offdesign reads
OFFDESIGN_POINT_FIELDS = ['thrust', 'fuel_mass_flow_rate', 'tau_r', 'tau_c', 'pi_r', 'pi_d', 'pi_c', 'fuel_to_air_ratio',
                          'core_nozzle_exit_velocity', 'core_nozzle_exit_static_temperature', 'core_nozzle_exit_static_pressure',
                          'core_nozzle_exit_stagnation_temperature', 'core_nozzle_exit_stagnation_pressure']

# Tt4 fraction step for the floor thrust slope; small because thrust is strongly concave above the floor
FLOOR_STEP = 0.005

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbojet_points_numpy
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbojet_points_numpy(turbojet, altitude, mach_number, static_temperature, static_pressure, velocity, throttle,
                                floor_fraction, below_floor, shaft_power_offtake):
    """
    Matched off-design point at each control point on NumPy: the numba kernel through solve_turbojet_offdesign_robust,
    one point at a time, with the afterburner when lit, the thrust below the matching floor extended from the floor
    toward the ram drag, and the idle fallback deck at points the matching cannot solve.

    Inputs:
       turbojet            turbojet with offdesign_matching
       altitude, mach_number, static_temperature, static_pressure, velocity, throttle (Tt4 fraction), floor_fraction,
       below_floor         1-D arrays, one entry per control point
       shaft_power_offtake 1-D array, or None for the design-point offtake

    Outputs:
       result              dict of 1-D arrays named as OFFDESIGN_POINT_FIELDS; NaN station data at idle fallback points
    """
    design_constants = turbojet.offdesign_matching.design_constants
    reference_point  = turbojet.offdesign_matching.reference_point
    idle_fallback    = getattr(turbojet.offdesign_matching, 'idle_fallback', None)
    n                = len(throttle)

    result                        = {field: np.full(n, np.nan) for field in OFFDESIGN_POINT_FIELDS}
    result['thrust']              = np.zeros(n)
    result['fuel_mass_flow_rate'] = np.zeros(n)

    packed_design_constants = pack_turbojet_design_constants(design_constants)
    packed_reference_point  = pack_turbojet_reference_point(reference_point)
    offtake = [None] * n if shaft_power_offtake is None else shaft_power_offtake
    def evaluate(i, fraction):
        point = solve_turbojet_offdesign_robust(
            design_constants, reference_point, mach_number[i], static_temperature[i], static_pressure[i],
            reference_point.Tt4 * fraction, packed_design_constants=packed_design_constants,
            packed_reference_point=packed_reference_point, shaft_power_offtake=offtake[i])
        if turbojet.afterburner_active and turbojet.afterburner is not None:
            point = apply_turbojet_afterburner(point, design_constants, turbojet.afterburner, turbojet.working_fluid,
                                               static_temperature[i], static_pressure[i])
        return point

    for i in range(n):
        try:
            point  = evaluate(i, max(throttle[i], floor_fraction[i]))
            thrust = point.thrust
            if below_floor[i]:
                # station data and fuel flow stay at the floor
                floor_slope = (evaluate(i, floor_fraction[i] + FLOOR_STEP).thrust - point.thrust) / FLOOR_STEP
                # in flight the thrust approaches the ram drag of the airflow with no gross thrust
                thrust      = extend_thrust_below_floor(point.thrust, floor_slope, throttle[i], floor_fraction[i],
                                                        lower_bound = -point.mass_flow_rate * velocity[i])
        except OffDesignMatchingError:
            if idle_fallback is None:
                raise
            # the deck is queried at the floor, not extrapolated below its data; fuel flow stays at the floor
            query_fraction = max(throttle[i], floor_fraction[i])
            F, FF = idle_fallback.query(np.array([altitude[i]]), np.array([mach_number[i]]),
                                         throttle=np.array([query_fraction]))
            if below_floor[i]:
                F_above, _ = idle_fallback.query(np.array([altitude[i]]), np.array([mach_number[i]]),
                                                 throttle=np.array([floor_fraction[i] + FLOOR_STEP]))
                F = np.array([extend_thrust_below_floor(F[0], (F_above[0] - F[0]) / FLOOR_STEP, throttle[i], floor_fraction[i])])
            result['thrust'][i]              = F[0]
            result['fuel_mass_flow_rate'][i] = FF[0]
            continue
        for field in OFFDESIGN_POINT_FIELDS:
            result[field][i] = point[field]
        result['thrust'][i] = thrust
    return result

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbojet_points_jax
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbojet_points_jax(turbojet, altitude, mach_number, static_temperature, static_pressure, velocity, throttle,
                              floor_fraction, below_floor, shaft_power_offtake):
    """
    Matched off-design point at every control point at once on JAX (solve_turbojet_offdesign_jax), differentiable,
    with the afterburner and the thrust below the matching floor as on NumPy. Points no solver matches are NaN: the
    idle fallback deck is not available on JAX.

    Inputs and outputs as solve_turbojet_points_numpy.
    """
    import jax.numpy as jnp
    design_constants = turbojet.offdesign_matching.design_constants
    reference_point  = turbojet.offdesign_matching.reference_point
    if shaft_power_offtake is None:
        shaft_power_offtake = getattr(design_constants, 'shaft_work_specific_design', 0.0) * reference_point.m0 * jnp.ones_like(throttle)

    def evaluate(fraction):
        point, matched = solve_turbojet_offdesign_jax(design_constants, reference_point, mach_number, static_temperature,
                                                      static_pressure, reference_point.Tt4 * fraction, shaft_power_offtake)
        if turbojet.afterburner_active and turbojet.afterburner is not None:
            point = apply_turbojet_afterburner(point, design_constants, turbojet.afterburner, turbojet.working_fluid,
                                               static_temperature, static_pressure)
        return point

    point  = evaluate(jnp.maximum(throttle, floor_fraction))
    result = {field: point[field] for field in OFFDESIGN_POINT_FIELDS}
    if getattr(turbojet.offdesign_matching, 'matching_floor', None) is not None:
        # station data and fuel flow stay at the floor; thrust decays from it toward the ram drag
        floor_slope = (evaluate(floor_fraction + FLOOR_STEP).thrust - point.thrust) / FLOOR_STEP
        extended    = extend_thrust_below_floor(point.thrust, floor_slope, throttle, floor_fraction,
                                                lower_bound = -point.mass_flow_rate * velocity)
        result['thrust'] = jnp.where(below_floor, extended, point.thrust)
    return result
