# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/compute_turbofan_performance.py
# 
# 
# Created:  Jul 2024, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Data, array_namespace
from RCAIDE.Library.Methods.Utilities import Linear_Interpolator
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                  import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor            import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor           import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Fan                  import compute_fan_performance, compute_fan_angular_velocity
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine              import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle     import compute_expansion_nozzle_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle   import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan             import compute_thrust
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common               import compute_gas_generator_shaft_power_offtake, extend_thrust_below_floor
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.compute_turbofan_performance_surrogate import compute_turbofan_performance_surrogate
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import (
    pack_design_constants, pack_reference_point,
    solve_turbofan_offdesign_robust, OffDesignMatchingError)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_jax import solve_turbofan_offdesign_jax

import  numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
# compute_performance
# ----------------------------------------------------------------------------------------------------------------------   
def compute_turbofan_performance(turbofan,state,network=None,center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes the performance of a turbofan engine by analyzing the thermodynamic cycle.
    
    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Propulsors.Turbofan
        Turbofan engine component with the following attributes:
            - tag : str
                Identifier for the turbofan
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
            - fan : Data
                Fan component
                    - tag : str
                        Identifier for the fan
                    - design_angular_velocity : float
                        Design angular velocity [rad/s]
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
            - core_nozzle : Data
                Core nozzle component
                    - tag : str
                        Identifier for the core nozzle
            - fan_nozzle : Data
                Fan nozzle component
                    - tag : str
                        Identifier for the fan nozzle
            - bypass_ratio : float
                Bypass ratio of the turbofan
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
            - ones_row : function
                Function to create array of ones with specified length
    center_of_gravity : list of lists, optional
        Center of gravity coordinates [[x, y, z]] [m]
        Default: [[0.0, 0.0, 0.0]]
    
    Returns
    -------
    inputs : Data
        Turbofan input conditions (power.electrical/mechanical/etc.)
    outputs : Data
        Turbofan output conditions (thrust, moment, power.propulsive, etc.)
    stored_results_flag : bool
        Flag indicating if results are stored
    stored_propulsor_tag : str
        Tag of the turbofan with stored results
    
    Notes
    -----
    This function computes the performance of a turbofan engine by sequentially analyzing
    each component in the engine's thermodynamic cycle. It links the output conditions of
    each component to the input conditions of the next component in the flow path.
    
    The function follows this sequence:
        1. Set working fluid properties
        2. Compute ram performance
        3. Compute inlet nozzle performance
        4. Compute fan performance
        5. Compute low pressure compressor performance
        6. Compute high pressure compressor performance
        7. Compute combustor performance
        8. Compute high pressure turbine performance
        9. Compute low pressure turbine performance
        10. Compute core nozzle performance
        11. Compute fan nozzle performance
        12. Compute thrust and power output
        13. Calculate efficiencies
        14. Handle electrical power generation/consumption if applicable
    
    **Major Assumptions**
        * Steady state operation
        * One-dimensional flow through components
        * Adiabatic components except for the combustor
        * Perfect gas behavior with variable properties
    
    References
    ----------
    [1] Mattingly, J.D., "Elements of Gas Turbine Propulsion", 2nd Edition, AIAA Education Series, 2005. https://soaneemrana.org/onewebmedia/ELEMENTS%20OF%20GAS%20TURBINE%20PROPULTION2.pdf
    [2] Cantwell, B., "AA283 Course Notes", Stanford University. https://web.stanford.edu/~cantwell/AA283_Course_Material/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.compute_thurst
    """ 

    if turbofan.offdesign_matching is not None:
        return compute_turbofan_performance_offdesign(turbofan, state, network, center_of_gravity)

    if turbofan.surrogate is not None:
        return compute_turbofan_performance_surrogate(turbofan, state, network, center_of_gravity)

    # else: analytical cycle model (below)

    conditions                = state.conditions
    noise_conditions          = conditions.aeroacoustics.propulsors[turbofan.tag] 
    turbofan_conditions       = conditions.energy.propulsors[turbofan.tag] 
    U0                        = conditions.freestream.velocity
    T                         = conditions.freestream.temperature
    P                         = conditions.freestream.pressure
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
    fuel                      = combustor.fuel_data
    
    # unpack component conditions 
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
 
    # ----------------------------------------------------------------------------
    # Compute Externally Supplied/Delivered Shaft Power from Electric Motors or Generators
    # ----------------------------------------------------------------------------
    # throttle is a fraction of the takeoff rating; the cycle runs on the fraction of the design point
    design_point_throttle   = turbofan_conditions.throttle * turbofan.rated_takeoff_temperature_ratio

    # absolute power [W], converted to specific work with the core mass flow before the HP turbine below
    lpc_conditions.omega    = low_pressure_compressor.design_angular_velocity * design_point_throttle
    external_shaft_work, _, _, _ = compute_gas_generator_shaft_power_offtake(turbofan, state, lpc_conditions.omega)

    # ----------------------------------------------------------------------------
    # Compute Turbofan Performance
    # ----------------------------------------------------------------------------

    # Set the working fluid to determine the fluid properties
    ram.working_fluid = turbofan.working_fluid

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
    
    # Link the fan to the inlet nozzle
    fan_conditions.inputs.stagnation_temperature                      = inlet_nozzle_conditions.outputs.stagnation_temperature
    fan_conditions.inputs.stagnation_pressure                         = inlet_nozzle_conditions.outputs.stagnation_pressure
    fan_conditions.inputs.static_temperature                          = inlet_nozzle_conditions.outputs.static_temperature
    fan_conditions.inputs.static_pressure                             = inlet_nozzle_conditions.outputs.static_pressure
    fan_conditions.inputs.mach_number                                 = inlet_nozzle_conditions.outputs.mach_number  
    fan.working_fluid                                                 = turbofan.working_fluid
    
    # Flow through the fan
    compute_fan_performance(fan,conditions)    

    # Link low pressure compressor to the inlet nozzle
    lpc_conditions.inputs.stagnation_temperature               = fan_conditions.outputs.stagnation_temperature
    lpc_conditions.inputs.stagnation_pressure                  = fan_conditions.outputs.stagnation_pressure
    lpc_conditions.inputs.static_temperature                   = fan_conditions.outputs.static_temperature
    lpc_conditions.inputs.static_pressure                      = fan_conditions.outputs.static_pressure
    lpc_conditions.inputs.mach_number                          = fan_conditions.outputs.mach_number  
    low_pressure_compressor.working_fluid                      = turbofan.working_fluid
    low_pressure_compressor.nondimensional_massflow            = turbofan.compressor_nondimensional_massflow
    low_pressure_compressor.reference_temperature              = turbofan.reference_temperature
    low_pressure_compressor.reference_pressure                 = turbofan.reference_pressure
        
    # Flow through the low pressure compressor
    compute_compressor_performance(low_pressure_compressor,conditions)

    # Link the high pressure compressor to the low pressure compressor
    hpc_conditions.inputs.stagnation_temperature                = lpc_conditions.outputs.stagnation_temperature
    hpc_conditions.inputs.stagnation_pressure                   = lpc_conditions.outputs.stagnation_pressure
    hpc_conditions.inputs.static_temperature                    = lpc_conditions.outputs.static_temperature
    hpc_conditions.inputs.static_pressure                       = lpc_conditions.outputs.static_pressure
    hpc_conditions.inputs.mach_number                           = lpc_conditions.outputs.mach_number  
    high_pressure_compressor.working_fluid                      = low_pressure_compressor.working_fluid    
    high_pressure_compressor.nondimensional_massflow            = turbofan.compressor_nondimensional_massflow
    high_pressure_compressor.reference_temperature              = turbofan.reference_temperature
    high_pressure_compressor.reference_pressure                 = turbofan.reference_pressure
        
    # Flow through the high pressure compressor
    compute_compressor_performance(high_pressure_compressor,conditions)

    # Link the combustor to the high pressure compressor
    combustor_conditions.inputs.stagnation_temperature                = hpc_conditions.outputs.stagnation_temperature
    combustor_conditions.inputs.stagnation_pressure                   = hpc_conditions.outputs.stagnation_pressure
    combustor_conditions.inputs.static_temperature                    = hpc_conditions.outputs.static_temperature
    combustor_conditions.inputs.static_pressure                       = hpc_conditions.outputs.static_pressure
    combustor_conditions.inputs.mach_number                           = hpc_conditions.outputs.mach_number  
    combustor.working_fluid                                           = high_pressure_compressor.working_fluid     
        
    # Flow through the high pressor compressor 
    compute_combustor_performance(combustor,conditions) 

    # Link the high pressure turbine to the combustor
    hpt_conditions.inputs.stagnation_temperature    = combustor_conditions.outputs.stagnation_temperature
    hpt_conditions.inputs.stagnation_pressure       = combustor_conditions.outputs.stagnation_pressure
    hpt_conditions.inputs.fuel_to_air_ratio         = combustor_conditions.outputs.fuel_to_air_ratio
    hpt_conditions.inputs.static_temperature        = combustor_conditions.outputs.static_temperature
    hpt_conditions.inputs.static_pressure           = combustor_conditions.outputs.static_pressure
    hpt_conditions.inputs.mach_number               = combustor_conditions.outputs.mach_number  
    hpt_conditions.inputs.compressor                = hpc_conditions.outputs
    hpt_conditions.inputs.bypass_ratio              = 0.0 #set to zero to ensure that fan not linked here
    # external_shaft_work above is an absolute power [W] (electrical/mechanical power
    # to/from the IDG or motor), but compute_turbine_performance's energy balance
    # (compressor_work + shaft_work + alpha*fan_work) is entirely in *specific* work
    # [J/kg] -- compressor_work/fan_work both come from ht_out-ht_in. Converting here
    # using the same mdot_core relation compute_thurst.py uses later (mdhc corrected by
    # the current LPC exit stagnation conditions, already computed above) -- this was
    # previously fed in unconverted, silently overstating the offtake's effect on the HP
    # turbine (a 300 kW offtake read as 300,000 J/kg is order-of-magnitude too large for
    # any real core mass flow).
    mdot_core_for_shaft_work                        = turbofan.compressor_nondimensional_massflow * \
        np.sqrt(turbofan.reference_temperature / lpc_conditions.outputs.stagnation_temperature) * \
        (lpc_conditions.outputs.stagnation_pressure / turbofan.reference_pressure)
    hpt_conditions.inputs.external_shaft.work_done  = external_shaft_work / mdot_core_for_shaft_work
    high_pressure_turbine.working_fluid             = combustor.working_fluid
        
    # Flow through the high pressure turbine
    compute_turbine_performance(high_pressure_turbine,conditions)

    # Link the low pressure turbine to the high pressure turbine
    lpt_conditions.inputs.stagnation_temperature     = hpt_conditions.outputs.stagnation_temperature
    lpt_conditions.inputs.stagnation_pressure        = hpt_conditions.outputs.stagnation_pressure
    lpt_conditions.inputs.static_temperature         = hpt_conditions.outputs.static_temperature
    lpt_conditions.inputs.static_pressure            = hpt_conditions.outputs.static_pressure
    lpt_conditions.inputs.mach_number                = hpt_conditions.outputs.mach_number
    lpt_conditions.inputs.velocity                   = hpt_conditions.outputs.velocity
    lpt_conditions.inputs.compressor                 = lpc_conditions.outputs
    lpt_conditions.inputs.fuel_to_air_ratio          = combustor_conditions.outputs.fuel_to_air_ratio
    lpt_conditions.inputs.fan                        = fan_conditions.outputs
    lpt_conditions.inputs.bypass_ratio               = bypass_ratio
    low_pressure_turbine.working_fluid               = high_pressure_turbine.working_fluid

    # Flow through the low pressure turbine
    compute_turbine_performance(low_pressure_turbine,conditions)

    # Link the core nozzle to the low pressure turbine
    core_nozzle_conditions.inputs.stagnation_temperature     = lpt_conditions.outputs.stagnation_temperature
    core_nozzle_conditions.inputs.stagnation_pressure        = lpt_conditions.outputs.stagnation_pressure
    core_nozzle_conditions.inputs.static_temperature         = lpt_conditions.outputs.static_temperature
    core_nozzle_conditions.inputs.static_pressure            = lpt_conditions.outputs.static_pressure 
    core_nozzle_conditions.inputs.velocity                   = lpt_conditions.outputs.velocity                
    core_nozzle_conditions.inputs.mach_number                = lpt_conditions.outputs.mach_number   
    core_nozzle.working_fluid                                = turbofan.working_fluid 
        
    # Flow through the core nozzle
    compute_expansion_nozzle_performance(core_nozzle,conditions)

    # Link the dan nozzle to the fan
    fan_nozzle_conditions.inputs.stagnation_temperature     = fan_conditions.outputs.stagnation_temperature
    fan_nozzle_conditions.inputs.stagnation_pressure        = fan_conditions.outputs.stagnation_pressure
    fan_nozzle_conditions.inputs.static_temperature         = fan_conditions.outputs.static_temperature
    fan_nozzle_conditions.inputs.static_pressure            = fan_conditions.outputs.static_pressure  
    fan_nozzle_conditions.inputs.mach_number                = fan_conditions.outputs.mach_number   
    fan_nozzle.working_fluid                                = turbofan.working_fluid
        
    # Flow through the fan nozzle
    compute_expansion_nozzle_performance(fan_nozzle,conditions)
 
    # Link the thrust component to the fan nozzle
    turbofan_conditions.fan_nozzle_exit_velocity                        = fan_nozzle_conditions.outputs.velocity
    turbofan_conditions.fan_nozzle_area_ratio                           = fan_nozzle_conditions.outputs.area_ratio  
    turbofan_conditions.fan_nozzle_static_pressure                      = fan_nozzle_conditions.outputs.static_pressure
    turbofan_conditions.core_nozzle_area_ratio                          = core_nozzle_conditions.outputs.area_ratio 
    turbofan_conditions.core_nozzle_static_pressure                     = core_nozzle_conditions.outputs.static_pressure
    turbofan_conditions.core_nozzle_exit_velocity                       = core_nozzle_conditions.outputs.velocity  

    # Link the thrust component to the combustor
    turbofan_conditions.fuel_to_air_ratio                        = combustor_conditions.outputs.fuel_to_air_ratio

    # Link the thrust component to the low pressure compressor 
    turbofan_conditions.total_temperature_reference              = lpc_conditions.outputs.stagnation_temperature
    turbofan_conditions.total_pressure_reference                 = lpc_conditions.outputs.stagnation_pressure 
    turbofan_conditions.bypass_ratio                             = bypass_ratio
    turbofan_conditions.flow_through_core                        = 1./(1.+bypass_ratio) # scaled constant to turn on core thrust computation
    turbofan_conditions.flow_through_fan                         = bypass_ratio/(1.+bypass_ratio) #scaled constant to turn on fan thrust computation        

    # Compute the thrust
    compute_thrust(turbofan,conditions)

    # Compute forces and moments
    moment_vector              = 0*state.ones_row(3)
    thrust_vector              = 0*state.ones_row(3)
    thrust_vector[:,0]         =  turbofan_conditions.thrust[:,0]
    center_of_gravity = [[0.0, 0.0,0.0]] 
    moment_vector[:,0]         =  turbofan.origin[0][0] -   center_of_gravity[0][0]
    moment_vector[:,1]         =  turbofan.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]         =  turbofan.origin[0][2]  -  center_of_gravity[0][2]
    M                          =  np.cross(moment_vector, thrust_vector)   
    moment                     = M 
    power                      = turbofan_conditions.outputs.power.propulsive
    turbofan_conditions.moment = moment 

    # compute efficiencies 
    mdot_air_core                                  = turbofan_conditions.core_mass_flow_rate
    mdot_air_fan                                   = bypass_ratio *  mdot_air_core  
    fuel_enthalpy                                  = combustor.fuel_data.specific_energy 
    mdot_fuel                                      = turbofan_conditions.fuel_mass_flow_rate  
    h_e_f                                          = fan_nozzle_conditions.outputs.static_enthalpy
    h_e_c                                          = core_nozzle_conditions.outputs.static_enthalpy
    h_0                                            = turbofan.working_fluid.compute_cp(T,P) * T 
    h_t4                                           = combustor_conditions.outputs.stagnation_enthalpy
    h_t3                                           = hpc_conditions.outputs.stagnation_enthalpy 
    turbofan_conditions.overall_efficiency         = turbofan_conditions.thrust[:, 0]* U0 / (mdot_fuel * fuel_enthalpy)  
    turbofan_conditions.thermal_efficiency         = 1 - ((mdot_air_core +  mdot_fuel)*(h_e_c -  h_0) + mdot_air_fan*(h_e_f - h_0) + mdot_fuel *h_0)/((mdot_air_core +  mdot_fuel)*h_t4 - mdot_air_core *h_t3)  
     
    # volumetric flow rate
    Q  = mdot_fuel /fuel.density 

    # haydraulic power o fuel flow
    power_hydraulic  = Q * hpc_conditions.outputs.stagnation_pressure  
        
    # compute shaft RPMs 
    fan_conditions.omega        = compute_fan_angular_velocity(fan, fan_conditions.outputs.stagnation_temperature - fan_conditions.inputs.stagnation_temperature)
    lpc_conditions.omega        = low_pressure_compressor.design_angular_velocity * design_point_throttle
    hpc_conditions.omega        = high_pressure_compressor.design_angular_velocity * design_point_throttle
  
    # store data
    fan_res         = Data(
                angular_velocity   = fan_conditions.omega
    )

    core_nozzle_res = Data(
                exit_static_temperature             = core_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = core_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = core_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = core_nozzle_conditions.outputs.stagnation_pressure,
                exit_velocity                       = core_nozzle_conditions.outputs.velocity
            )

    fan_nozzle_res = Data(
                exit_static_temperature             = fan_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = fan_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = fan_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = fan_nozzle_conditions.outputs.stagnation_pressure,
                exit_velocity                       = fan_nozzle_conditions.outputs.velocity
                )
                
    noise_conditions.copy_values({'fan_nozzle': fan_nozzle_res, 'core_nozzle': core_nozzle_res, 'fan': fan_res})
    stored_results_flag                              = True
    stored_propulsor_tag                             = turbofan.tag  
    
    turbofan_conditions.outputs.thrust               = thrust_vector
    turbofan_conditions.outputs.moment               = moment
    turbofan_conditions.outputs.power.propulsive     = power 
    turbofan_conditions.inputs.power.hydraulic       = power_hydraulic  
    turbofan_conditions.inputs.power.chemical        = mdot_fuel * combustor.fuel_data.lower_heating_value 
    turbofan_conditions.fuel_mass_flow_rate          = mdot_fuel  

    return turbofan_conditions.inputs ,turbofan_conditions.outputs, stored_results_flag, stored_propulsor_tag 
    
def reuse_stored_turbofan_data(turbofan,state,network,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one turbofan for identical turbofans
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    turbofan             - turbofan data structure                [-]
    state                - operating conditions data structure   [-]  
    fuel_line            - fuelline                              [-]  
    total_thrust         - thrust of turbofan group              [N]
    total_power          - power of turbofan group               [W] 

    Outputs:  
    total_thrust         - thrust of turbofan group              [N]
    total_power          - power of turbofan group               [W] 
    
    Properties Used: 
    N.A.        
    '''
    # unpack
    conditions                  = state.conditions 
    ram                         = turbofan.ram
    inlet_nozzle                = turbofan.inlet_nozzle
    fan                         = turbofan.fan
    low_pressure_compressor     = turbofan.low_pressure_compressor
    high_pressure_compressor    = turbofan.high_pressure_compressor
    combustor                   = turbofan.combustor
    high_pressure_turbine       = turbofan.high_pressure_turbine
    low_pressure_turbine        = turbofan.low_pressure_turbine
    core_nozzle                 = turbofan.core_nozzle
    fan_nozzle                  = turbofan.fan_nozzle  
    ram_0                       = network.propulsors[stored_propulsor_tag].ram
    inlet_nozzle_0              = network.propulsors[stored_propulsor_tag].inlet_nozzle
    fan_0                       = network.propulsors[stored_propulsor_tag].fan
    low_pressure_compressor_0   = network.propulsors[stored_propulsor_tag].low_pressure_compressor
    high_pressure_compressor_0  = network.propulsors[stored_propulsor_tag].high_pressure_compressor
    combustor_0                 = network.propulsors[stored_propulsor_tag].combustor
    high_pressure_turbine_0     = network.propulsors[stored_propulsor_tag].high_pressure_turbine
    low_pressure_turbine_0      = network.propulsors[stored_propulsor_tag].low_pressure_turbine
    core_nozzle_0               = network.propulsors[stored_propulsor_tag].core_nozzle
    fan_nozzle_0                = network.propulsors[stored_propulsor_tag].fan_nozzle 
    
    # copy the stored turbofan's results into this one's, reusing its memory
    conditions.energy.propulsors[turbofan.tag].copy_values(conditions.energy.propulsors[stored_propulsor_tag])
    conditions.aeroacoustics.propulsors[turbofan.tag].copy_values(conditions.aeroacoustics.propulsors[stored_propulsor_tag])
    for component, component_0 in ((ram, ram_0), (inlet_nozzle, inlet_nozzle_0), (fan, fan_0),
                                    (low_pressure_compressor, low_pressure_compressor_0),
                                    (high_pressure_compressor, high_pressure_compressor_0), (combustor, combustor_0),
                                    (low_pressure_turbine, low_pressure_turbine_0),
                                    (high_pressure_turbine, high_pressure_turbine_0), (core_nozzle, core_nozzle_0),
                                    (fan_nozzle, fan_nozzle_0)):
        conditions.energy.converters[component.tag].copy_values(conditions.energy.converters[component_0.tag])
    
    # compute moment  
    thrust             = conditions.energy.propulsors[turbofan.tag].outputs.thrust[:,0]
    xp                 = array_namespace(thrust)
    zero               = 0 * thrust
    thrust_vector      = xp.stack([thrust, zero, zero], axis=1)
    moment_vector      = xp.stack([zero + (turbofan.origin[0][0] - center_of_gravity[0][0]),
                                   zero + (turbofan.origin[0][1] - center_of_gravity[0][1]),
                                   zero + (turbofan.origin[0][2] - center_of_gravity[0][2])], axis=1)
    moment             = xp.cross(moment_vector, thrust_vector)
   
    conditions.energy.propulsors[turbofan.tag].outputs.moment = moment
    
    
    idm   = turbofan.integrated_drive_motor
    idm_0 = network.propulsors[stored_propulsor_tag].integrated_drive_motor
    if idm != None and  len(state.numerics.time.differentiate) > 0:
        conditions.energy.converters[idm.tag].copy_values(conditions.energy.converters[idm_0.tag])
        conditions.energy.propulsors[turbofan.tag].inputs.power.electrical = conditions.energy.converters[idm.tag].inputs.power.electrical

    idg   = turbofan.integrated_drive_generator
    idg_0 = network.propulsors[stored_propulsor_tag].integrated_drive_generator
    if idg != None and len(state.numerics.time.differentiate) > 0:
        conditions.energy.converters[idg.tag].copy_values(conditions.energy.converters[idg_0.tag])
        conditions.energy.propulsors[turbofan.tag].outputs.power.electrical = conditions.energy.converters[idg.tag].outputs.power.electrical


    return conditions.energy.propulsors[turbofan.tag].inputs, conditions.energy.propulsors[turbofan.tag].outputs


# ----------------------------------------------------------------------------------------------------------------------
#  compute_turbofan_performance_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def compute_turbofan_performance_offdesign(turbofan, state, network=None, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes turbofan thrust and fuel flow by live off-design component
    matching (Mattingly Ch. 8 -- see `solve_turbofan_offdesign_robust`)
    instead of the fixed-pressure-ratio analytical cycle model or the table-
    driven surrogate. Dispatched from `compute_turbofan_performance` when
    `turbofan.offdesign_matching` is not None, checked before `turbofan.surrogate`.

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
        Must have `turbofan.offdesign_matching` set to
        `Data(design_constants=..., reference_point=...)`, from
        `design_turbofan_offdesign_matching`.
    state : RCAIDE.Framework.Mission.Common.State
        Must have `conditions.freestream.mach_number/temperature/pressure`,
        and `conditions.energy.propulsors[turbofan.tag].throttle`.

    Returns
    -------
    inputs, outputs, stored_results_flag, stored_propulsor_tag
        Same return signature as `compute_turbofan_performance`, for a common
        call site in `Turbofan.compute_performance`.

    Raises
    ------
    OffDesignMatchingError
        If the matching solver fails to converge at a control point AND
        `turbofan.offdesign_matching.idle_fallback` is not set. With no
        fallback configured, this fails loudly and stops the run rather than
        silently substituting an unflagged approximation for the failed point
        -- unchanged default behavior. If `idle_fallback` (a built
        `Turbofan_Surrogate`, queried with `rating_code='FID'`) is set, a
        point that fails to converge is routed to it instead of raising --
        see Notes. There is no solver-only fix for this: the matching
        equations are a closed-form power-balance model of a driven cycle,
        and deep part-power/idle is closer to windmilling (ram drag exceeds
        gross thrust -- confirmed against real Flight Idle deck data, not
        assumed), a different regime the equations don't describe at all.

    Notes
    -----
    Throttle is consumed differently here than by the analytical cycle model:
    the analytical model holds `combustor.turbine_inlet_temperature` fixed
    regardless of throttle and applies throttle as a post-hoc thrust
    multiplier downstream (`compute_thrust`). This solver instead needs an
    actual combustor exit temperature to match against, so throttle is used
    as a fraction of the takeoff rating:
    `Tt4 = reference_point.Tt4 * rated_takeoff_temperature_ratio * throttle`. This is a simplification (a real
    engine's throttle-to-Tt4 relationship is not perfectly linear), not a
    validated correlation -- consistent with how this solver was stress-
    tested throughout its development (as a Tt4-fraction sweep), but worth
    revisiting if a specific engine's real throttle schedule is known.

    Unlike the surrogate (which has no component-level station data to
    report and fills `noise_conditions` with NaN), this path computes real
    core/fan nozzle exit conditions as part of the matching solve -- except
    at a point routed to `idle_fallback`, which (being a surrogate query
    itself) has none either, so `noise_conditions` is NaN for exactly those
    points, real everywhere else.

    Below the matching floor the fuel flow stays at the floor and the thrust
    decays from the floor thrust toward zero (extend_thrust_below_floor), so
    it is never negative and keeps a gradient for the mission solver.

    On JAX every control point is matched at once by `solve_turbofan_offdesign_jax`, differentiable through the
    implicit function theorem. Points no solver matches are NaN there: the idle fallback deck is not available on JAX.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan_offdesign_matching
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_jax
    """
    conditions           = state.conditions
    turbofan_conditions  = conditions.energy.propulsors[turbofan.tag]
    noise_conditions     = conditions.aeroacoustics.propulsors[turbofan.tag]
    xp                   = array_namespace(turbofan_conditions.throttle, conditions.freestream.mach_number)

    # freestream quantities may be column vectors or 1-D arrays (e.g. estimate_take_off_field_length),
    # and may hold a single value for all control points -- broadcast each onto the throttle rows
    # throttle is a fraction of the takeoff rating; the solver runs on the fraction of the design-point Tt4
    throttle            = xp.ravel(turbofan_conditions.throttle) * turbofan.rated_takeoff_temperature_ratio
    n                   = throttle.shape[0]
    altitude            = xp.ravel(conditions.freestream.altitude)    * xp.ones(n)
    mach_number         = xp.ravel(conditions.freestream.mach_number) * xp.ones(n)
    static_temperature  = xp.ravel(conditions.freestream.temperature) * xp.ones(n)
    static_pressure     = xp.ravel(conditions.freestream.pressure)    * xp.ones(n)
    velocity            = xp.ravel(conditions.freestream.velocity)    * xp.ones(n)

    design_constants = turbofan.offdesign_matching.design_constants
    reference_point  = turbofan.offdesign_matching.reference_point
    matching_floor   = getattr(turbofan.offdesign_matching, 'matching_floor', None)

    # lowest Tt4 fraction the matching solver represents; below it thrust decays from the floor (extend_thrust_below_floor)
    if matching_floor is not None:
        floor_interpolator = Linear_Interpolator([matching_floor.altitude, matching_floor.mach_number],
                                                 matching_floor.combustor_exit_temperature_fraction)
        floor_fraction     = floor_interpolator(xp.stack([altitude, mach_number], axis=1))
    else:
        floor_fraction     = -np.inf * xp.ones(n)
    below_floor = throttle < floor_fraction

    # HP-spool shaft power to/from the integrated drive motor or generator at the mission's electrical
    # power; the design-point offtake outside a mission
    lpc_conditions       = conditions.energy.converters[turbofan.low_pressure_compressor.tag]
    lpc_conditions.omega = turbofan.low_pressure_compressor.design_angular_velocity * xp.reshape(throttle, (-1, 1))
    external_shaft_power, _, _, in_mission = compute_gas_generator_shaft_power_offtake(turbofan, state, lpc_conditions.omega)
    shaft_power_offtake  = xp.ravel(external_shaft_power) * xp.ones(n) if in_mission else None

    flight = (altitude, mach_number, static_temperature, static_pressure, throttle, floor_fraction, below_floor, shaft_power_offtake)
    if xp is np:
        result = solve_offdesign_points_numpy(turbofan, *flight)
    else:
        result = solve_offdesign_points_jax(turbofan, *flight)

    thrust_N            = result['thrust']
    fuel_mass_flow_rate = result['fuel_mass_flow_rate']
    alpha               = result['alpha']
    zero                = 0 * thrust_N
    thrust_vector       = xp.stack([thrust_N, zero, zero], axis=1)

    positive       = thrust_N > 0
    gravity        = xp.ravel(conditions.freestream.gravity) * xp.ones(n) if hasattr(conditions.freestream, 'gravity') \
                     else 9.80665 * xp.ones(n)
    TSFC           = xp.where(positive, fuel_mass_flow_rate * gravity / xp.where(positive, thrust_N, 1.0), 0.0)

    power_propulsive = thrust_N * velocity

    # Compute forces and moments
    moment_vector  = xp.stack([zero + (turbofan.origin[0][0] - center_of_gravity[0][0]),
                               zero + (turbofan.origin[0][1] - center_of_gravity[0][1]),
                               zero + (turbofan.origin[0][2] - center_of_gravity[0][2])], axis=1)
    moment         = xp.cross(moment_vector, thrust_vector)

    # Pack turbofan outputs -- same fields as compute_turbofan_performance/_surrogate
    turbofan_conditions.thrust                            = thrust_vector
    turbofan_conditions.fuel_mass_flow_rate               = xp.reshape(fuel_mass_flow_rate, (-1,1))
    turbofan_conditions.thrust_specific_fuel_consumption  = xp.reshape(TSFC, (-1,1))
    turbofan_conditions.moment                            = moment
    turbofan_conditions.outputs.thrust                    = thrust_vector
    turbofan_conditions.outputs.moment                    = moment
    turbofan_conditions.outputs.power.propulsive          = xp.reshape(power_propulsive, (-1,1))
    turbofan_conditions.below_matching_floor              = xp.reshape(below_floor.astype(float), (-1,1))

    # same fields the analytical path sets; NaN at idle_fallback points
    turbofan_conditions.bypass_ratio                      = xp.reshape(alpha, (-1,1))
    turbofan_conditions.flow_through_core                 = xp.reshape(1./(1.+alpha), (-1,1))
    turbofan_conditions.flow_through_fan                  = xp.reshape(alpha/(1.+alpha), (-1,1))
    turbofan_conditions.fuel_to_air_ratio                 = xp.reshape(result['fuel_to_air_ratio'], (-1,1))
    turbofan_conditions.core_mass_flow_rate               = xp.reshape(result['mass_flow_rate'] / (1 + alpha), (-1,1))
    turbofan_conditions.total_temperature_reference       = xp.reshape(result['tau_r']*result['tau_f']*static_temperature, (-1,1))
    turbofan_conditions.total_pressure_reference          = xp.reshape(result['pi_r']*result['pi_d']*result['pi_f']*static_pressure, (-1,1))

    if turbofan.combustor is not None and turbofan.combustor.fuel_data is not None:
        turbofan_conditions.inputs.power.chemical = \
            xp.reshape(fuel_mass_flow_rate * turbofan.combustor.fuel_data.lower_heating_value, (-1,1))
        # uses specific_energy, not lower_heating_value used just above -- different fields
        turbofan_conditions.overall_efficiency = \
            xp.reshape(thrust_N * velocity / (fuel_mass_flow_rate * turbofan.combustor.fuel_data.specific_energy), (-1,1))
    # thermal_efficiency not set: needs internal enthalpies the offdesign solver doesn't return

    # noise_conditions schema match -- real values at points the live solver handled,
    # NaN at any point routed to idle_fallback (no station data there either)
    for stream in ('core', 'fan'):
        noise_conditions.copy_values({stream + '_nozzle': Data(
            exit_static_temperature      = xp.reshape(result[stream + '_nozzle_exit_static_temperature'], (-1,1)),
            exit_static_pressure         = xp.reshape(result[stream + '_nozzle_exit_static_pressure'], (-1,1)),
            exit_stagnation_temperature  = xp.reshape(result[stream + '_nozzle_exit_stagnation_temperature'], (-1,1)),
            exit_stagnation_pressure     = xp.reshape(result[stream + '_nozzle_exit_stagnation_pressure'], (-1,1)),
            exit_velocity                = xp.reshape(result[stream + '_nozzle_exit_velocity'], (-1,1)),
        )})
    # fan total temperature rise (fan inlet Tt2 = T0*tau_r) sets the fan angular velocity
    fan_inlet_stagnation_temperature  = result['tau_r'] * static_temperature
    fan_exit_stagnation_temperature   = fan_inlet_stagnation_temperature * result['tau_f_alone']
    fan_angular_velocity              = compute_fan_angular_velocity(turbofan.fan, fan_exit_stagnation_temperature - fan_inlet_stagnation_temperature)
    noise_conditions.copy_values({'fan': Data(angular_velocity = xp.reshape(fan_angular_velocity, (-1,1)))})

    # component station states, in the same conditions the analytical cycle populates, so that
    # downstream consumers (e.g. the aeroacoustic engine noise models) read one location
    converters = conditions.energy.converters
    for nozzle, stream in ((turbofan.core_nozzle, 'core'), (turbofan.fan_nozzle, 'fan')):
        nozzle_outputs                        = converters[nozzle.tag].outputs
        nozzle_outputs.velocity               = xp.reshape(result[stream + '_nozzle_exit_velocity'], (-1,1))
        nozzle_outputs.static_temperature     = xp.reshape(result[stream + '_nozzle_exit_static_temperature'], (-1,1))
        nozzle_outputs.static_pressure        = xp.reshape(result[stream + '_nozzle_exit_static_pressure'], (-1,1))
        nozzle_outputs.stagnation_temperature = xp.reshape(result[stream + '_nozzle_exit_stagnation_temperature'], (-1,1))
        nozzle_outputs.stagnation_pressure    = xp.reshape(result[stream + '_nozzle_exit_stagnation_pressure'], (-1,1))

    # combustor inlet (station 3) state, used by the emissions models
    combustor_conditions                                = converters[turbofan.combustor.tag]
    combustor_conditions.inputs.stagnation_temperature  = xp.reshape(static_temperature * result['tau_r'] * result['tau_f'] * result['tau_cH'], (-1,1))
    combustor_conditions.inputs.stagnation_pressure     = xp.reshape(static_pressure * result['pi_r'] * result['pi_d'] * result['pi_f'] * result['pi_cH'], (-1,1))
    combustor_conditions.outputs.fuel_to_air_ratio      = xp.reshape(result['fuel_to_air_ratio'], (-1,1))

    fan_conditions                                = converters[turbofan.fan.tag]
    fan_conditions.inputs.stagnation_temperature  = xp.reshape(fan_inlet_stagnation_temperature, (-1,1))
    fan_conditions.outputs.stagnation_temperature = xp.reshape(fan_exit_stagnation_temperature, (-1,1))
    fan_conditions.omega                          = xp.reshape(fan_angular_velocity, (-1,1))

    stored_results_flag   = True
    stored_propulsor_tag  = turbofan.tag

    return turbofan_conditions.inputs, turbofan_conditions.outputs, stored_results_flag, stored_propulsor_tag

# fields of the matched point that compute_turbofan_performance_offdesign reads
OFFDESIGN_POINT_FIELDS = ['thrust', 'fuel_mass_flow_rate', 'alpha', 'tau_r', 'tau_f', 'tau_f_alone', 'tau_cH', 'pi_r', 'pi_d',
                          'pi_f', 'pi_cH', 'fuel_to_air_ratio', 'mass_flow_rate',
                          'core_nozzle_exit_velocity', 'fan_nozzle_exit_velocity',
                          'core_nozzle_exit_static_temperature', 'fan_nozzle_exit_static_temperature',
                          'core_nozzle_exit_static_pressure', 'fan_nozzle_exit_static_pressure',
                          'core_nozzle_exit_stagnation_temperature', 'fan_nozzle_exit_stagnation_temperature',
                          'core_nozzle_exit_stagnation_pressure', 'fan_nozzle_exit_stagnation_pressure']

# Tt4 fraction step for the floor thrust slope
FLOOR_STEP = 0.02

# ----------------------------------------------------------------------------------------------------------------------
#  solve_offdesign_points_numpy
# ----------------------------------------------------------------------------------------------------------------------
def solve_offdesign_points_numpy(turbofan, altitude, mach_number, static_temperature, static_pressure, throttle,
                                 floor_fraction, below_floor, shaft_power_offtake):
    """
    Matched off-design point at each control point on NumPy: the numba kernel through solve_turbofan_offdesign_robust,
    one point at a time, with the thrust below the matching floor extended from the floor and the idle fallback deck
    at points the matching cannot solve.

    Inputs:
       turbofan            turbofan with offdesign_matching
       altitude, mach_number, static_temperature, static_pressure, throttle (Tt4 fraction), floor_fraction,
       below_floor         1-D arrays, one entry per control point
       shaft_power_offtake 1-D array, or None for the design-point offtake

    Outputs:
       result              dict of 1-D arrays named as OFFDESIGN_POINT_FIELDS; NaN station data at idle fallback points
    """
    design_constants = turbofan.offdesign_matching.design_constants
    reference_point  = turbofan.offdesign_matching.reference_point
    idle_fallback    = getattr(turbofan.offdesign_matching, 'idle_fallback', None)
    n                = len(throttle)

    # NaN, not zero: a point routed to idle_fallback has no real station data (see Notes),
    # and NaN correctly signals that rather than a silently-wrong 0
    result                        = {field: np.full(n, np.nan) for field in OFFDESIGN_POINT_FIELDS}
    result['thrust']              = np.zeros(n)
    result['fuel_mass_flow_rate'] = np.zeros(n)

    packed_design_constants = pack_design_constants(design_constants)
    packed_reference_point  = pack_reference_point(reference_point)
    offtake = [None] * n if shaft_power_offtake is None else shaft_power_offtake
    for i in range(n):
        combustor_exit_temperature = reference_point.Tt4 * max(throttle[i], floor_fraction[i])
        try:
            point  = solve_turbofan_offdesign_robust(
                design_constants, reference_point, mach_number[i], static_temperature[i], static_pressure[i],
                combustor_exit_temperature, packed_design_constants=packed_design_constants,
                packed_reference_point=packed_reference_point, shaft_power_offtake=offtake[i])
            thrust = point.thrust
            if below_floor[i]:
                # station data and fuel flow stay at the floor
                above_floor = solve_turbofan_offdesign_robust(
                    design_constants, reference_point, mach_number[i], static_temperature[i], static_pressure[i],
                    reference_point.Tt4 * (floor_fraction[i] + FLOOR_STEP), packed_design_constants=packed_design_constants,
                    packed_reference_point=packed_reference_point, shaft_power_offtake=offtake[i])
                floor_slope = (above_floor.thrust - point.thrust) / FLOOR_STEP
                thrust      = extend_thrust_below_floor(point.thrust, floor_slope, throttle[i], floor_fraction[i])
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
#  solve_offdesign_points_jax
# ----------------------------------------------------------------------------------------------------------------------
def solve_offdesign_points_jax(turbofan, altitude, mach_number, static_temperature, static_pressure, throttle,
                               floor_fraction, below_floor, shaft_power_offtake):
    """
    Matched off-design point at every control point at once on JAX (solve_turbofan_offdesign_jax), differentiable,
    with the thrust below the matching floor extended from the floor as on NumPy. Points no solver matches are NaN:
    the idle fallback deck is not available on JAX.

    Inputs and outputs as solve_offdesign_points_numpy.
    """
    import jax.numpy as jnp
    design_constants = turbofan.offdesign_matching.design_constants
    reference_point  = turbofan.offdesign_matching.reference_point
    if shaft_power_offtake is None:
        offtake_design      = getattr(design_constants, 'shaft_work_specific_design', 0.0) * reference_point.m0 / (1 + reference_point.alpha)
        shaft_power_offtake = offtake_design * jnp.ones_like(throttle)
    freestream = (mach_number, static_temperature, static_pressure)

    result, matched = solve_turbofan_offdesign_jax(design_constants, reference_point, *freestream,
                                                   reference_point.Tt4 * jnp.maximum(throttle, floor_fraction), shaft_power_offtake)
    result = dict(result)
    if getattr(turbofan.offdesign_matching, 'matching_floor', None) is not None:
        # station data and fuel flow stay at the floor; thrust decays from it with the floor's slope
        above, _    = solve_turbofan_offdesign_jax(design_constants, reference_point, *freestream,
                                                   reference_point.Tt4 * (floor_fraction + FLOOR_STEP), shaft_power_offtake)
        floor_slope = (above['thrust'] - result['thrust']) / FLOOR_STEP
        extended    = extend_thrust_below_floor(result['thrust'], floor_slope, throttle, floor_fraction)
        result['thrust'] = jnp.where(below_floor, extended, result['thrust'])
    return result
