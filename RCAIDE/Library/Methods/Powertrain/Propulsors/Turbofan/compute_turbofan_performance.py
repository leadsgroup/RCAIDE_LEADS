# RCAIDE/Methods/Energy/Propulsors/Turbofan/compute_turbofan_performance.py
# 
# 
# Created:  Jul 2024, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core import Data   
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                  import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor            import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor           import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Fan                  import compute_fan_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine              import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle     import compute_expansion_nozzle_performance 
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle   import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan             import compute_thrust

import  numpy as  np
from copy import  deepcopy

# ----------------------------------------------------------------------------------------------------------------------
# compute_performance
# ----------------------------------------------------------------------------------------------------------------------   
def compute_turbofan_performance(turbofan, state, center_of_gravity=[[0.0, 0.0, 0.0]]):
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
                - motor : Data, optional
                    Electric motor component
                - generator : Data, optional
                    Electric generator component
                - design_angular_velocity : float
                    Design angular velocity [rad/s]
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
    thrust_vector : numpy.ndarray
        Thrust force vector [N]
    moment : numpy.ndarray
        Moment vector [N·m]
    power : numpy.ndarray
        Shaft power output [W]
    power_elec : numpy.ndarray
        Electrical power input/output [W]
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
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.compute_thrust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.reuse_stored_turbofan_data
    """ 
    conditions                = state.conditions   
    noise_conditions          = conditions.noise.propulsors[turbofan.tag] 
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
    hpt_conditions.inputs.fan                       = fan_conditions.outputs
    hpt_conditions.inputs.bypass_ratio              = 0.0 #set to zero to ensure that fan not linked here 
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
    turbofan_conditions.flow_through_core                        = 1./(1.+bypass_ratio) #scaled constant to turn on core thrust computation
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
    power                      = turbofan_conditions.power 
    turbofan_conditions.moment = moment 
        
    # compute efficiencies 
    mdot_air_core                                  = turbofan_conditions.core_mass_flow_rate
    mdot_air_fan                                   = bypass_ratio *  mdot_air_core  
    fuel_enthalpy                                  = combustor.fuel_data.specific_energy 
    mdot_fuel                                      = turbofan_conditions.fuel_flow_rate  
    h_e_f                                          = fan_nozzle_conditions.outputs.static_enthalpy
    h_e_c                                          = core_nozzle_conditions.outputs.static_enthalpy
    h_0                                            = turbofan.working_fluid.compute_cp(T,P) * T 
    h_t4                                           = combustor_conditions.outputs.stagnation_enthalpy
    h_t3                                           = hpc_conditions.outputs.stagnation_enthalpy 
    turbofan_conditions.overall_efficiency         = thrust_vector* U0 / (mdot_fuel * fuel_enthalpy)  
    turbofan_conditions.thermal_efficiency         = 1 - ((mdot_air_core +  mdot_fuel)*(h_e_c -  h_0) + mdot_air_fan*(h_e_f - h_0) + mdot_fuel *h_0)/((mdot_air_core +  mdot_fuel)*h_t4 - mdot_air_core *h_t3)  
     
    # compute shaft RPMs 
    fan_conditions.omega        = fan.design_angular_velocity * turbofan_conditions.throttle
    lpc_conditions.omega        = low_pressure_compressor.design_angular_velocity * turbofan_conditions.throttle
    hpc_conditions.omega        = high_pressure_compressor.design_angular_velocity * turbofan_conditions.throttle
    
    # compute electrical power if generated/supplied  
    power_elec = 0*state.ones_row(1)
    if low_pressure_compressor.motor != None and  len(state.numerics.time.differentiate) > 0: 
        compressor_motor_conditions                 = conditions.energy.converters[low_pressure_compressor.motor.tag] 
        compressor_motor_conditions.outputs.power   = power *conditions.energy.hybrid_power_split_ratio   
        compressor_motor_conditions.outputs.omega   = lpc_conditions.omega
        compressor_motor_conditions.outputs.torque  = compressor_motor_conditions.outputs.power / compressor_motor_conditions.outputs.omega   
        power_elec =  compressor_motor_conditions.outputs.power  
    
    if low_pressure_compressor.generator != None and len(state.numerics.time.differentiate) > 0: 
        compressor_generator_conditions                = conditions.energy.converters[low_pressure_compressor.generator.tag] 
        compressor_generator_conditions.inputs.power   = power *conditions.energy.hybrid_power_split_ratio  
        compressor_generator_conditions.inputs.omega   = lpc_conditions.omega
        compressor_generator_conditions.outputs.torque = compressor_generator_conditions.outputs.power / compressor_generator_conditions.outputs.omega  
        power_elec =  compressor_generator_conditions.inputs.power  
    
  
    # store data
    core_nozzle_res = Data(
                exit_static_temperature             = core_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = core_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = core_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = core_nozzle_conditions.outputs.static_pressure,
                exit_velocity                       = core_nozzle_conditions.outputs.velocity
            )

    fan_nozzle_res = Data(
                exit_static_temperature             = fan_nozzle_conditions.outputs.static_temperature,
                exit_static_pressure                = fan_nozzle_conditions.outputs.static_pressure,
                exit_stagnation_temperature         = fan_nozzle_conditions.outputs.stagnation_temperature,
                exit_stagnation_pressure            = fan_nozzle_conditions.outputs.static_pressure,
                exit_velocity                       = fan_nozzle_conditions.outputs.velocity
                )
                
    lpc_res = Data(
                angular_velocity    = fan_conditions.omega, 
            )

    noise_conditions.fan_nozzle             = fan_nozzle_res
    noise_conditions.core_nozzle            = core_nozzle_res  
    noise_conditions.low_pressure_spool     = lpc_res
    stored_results_flag                     = True
    stored_propulsor_tag                    = turbofan.tag 
    
    return thrust_vector,moment,power,power_elec,stored_results_flag,stored_propulsor_tag 
    
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
    
    # deep copy results 
    conditions.energy.propulsors[turbofan.tag]                 = deepcopy(conditions.energy.propulsors[stored_propulsor_tag])
    conditions.noise.propulsors[turbofan.tag]                  = deepcopy(conditions.noise.propulsors[stored_propulsor_tag]) 
    conditions.energy.converters[ram.tag]                      = deepcopy(conditions.energy.converters[ram_0.tag]                     )
    conditions.energy.converters[inlet_nozzle.tag]             = deepcopy(conditions.energy.converters[inlet_nozzle_0.tag]            )
    conditions.energy.converters[fan.tag]                      = deepcopy(conditions.energy.converters[fan_0.tag]                     )
    conditions.energy.converters[low_pressure_compressor.tag]  = deepcopy(conditions.energy.converters[low_pressure_compressor_0.tag] )
    conditions.energy.converters[high_pressure_compressor.tag] = deepcopy(conditions.energy.converters[high_pressure_compressor_0.tag])
    conditions.energy.converters[combustor.tag]                = deepcopy(conditions.energy.converters[combustor_0.tag]               )
    conditions.energy.converters[low_pressure_turbine.tag]     = deepcopy(conditions.energy.converters[low_pressure_turbine_0.tag]    )
    conditions.energy.converters[high_pressure_turbine.tag]    = deepcopy(conditions.energy.converters[high_pressure_turbine_0.tag]   )
    conditions.energy.converters[core_nozzle.tag]              = deepcopy(conditions.energy.converters[core_nozzle_0.tag]             )
    conditions.energy.converters[fan_nozzle.tag]               = deepcopy(conditions.energy.converters[fan_nozzle_0.tag]              )
    
    # compute moment  
    moment_vector      = 0*state.ones_row(3)
    thrust_vector      = 0*state.ones_row(3)
    thrust_vector[:,0] = conditions.energy.propulsors[turbofan.tag].thrust[:,0] 
    moment_vector[:,0] = turbofan.origin[0][0] -   center_of_gravity[0][0] 
    moment_vector[:,1] = turbofan.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2] = turbofan.origin[0][2]  -  center_of_gravity[0][2]
    moment             = np.cross(moment_vector,thrust_vector)    
  
    power                                             = conditions.energy.propulsors[turbofan.tag].power 
    conditions.energy.propulsors[turbofan.tag].moment = moment
    
    power_elec = 0*state.ones_row(1)
    if low_pressure_compressor.motor != None and  len(state.numerics.time.differentiate) > 0: 
        conditions.energy.converters[low_pressure_compressor.motor.tag]  = deepcopy(conditions.energy.converters[low_pressure_compressor_0.motor.tag]) 
        power_elec =  conditions.energy.converters[low_pressure_compressor.motor.tag].outputs.power  
    
    if low_pressure_compressor.generator != None and len(state.numerics.time.differentiate) > 0:  
        conditions.energy.converters[low_pressure_compressor.generator.tag]  = deepcopy(conditions.energy.converters[low_pressure_compressor_0.generator.tag]) 
        power_elec =  conditions.energy.converters[low_pressure_compressor.generator.tag].inputs.power
        
    return thrust_vector,moment,power, power_elec