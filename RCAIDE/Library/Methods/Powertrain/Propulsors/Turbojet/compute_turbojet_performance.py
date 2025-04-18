# RCAIDE/Methods/Energy/Propulsors/Networks/Turbojet/compute_turbojet_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core import Data    
from RCAIDE.Library.Methods.Powertrain.Converters.Ram                import compute_ram_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor          import compute_combustor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor         import compute_compressor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine            import compute_turbine_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Supersonic_Nozzle  import compute_supersonic_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle import compute_compression_nozzle_performance
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet           import compute_thrust

# python imports 
import  numpy as  np 
from copy import  deepcopy

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
    conditions                = state.conditions
    noise_conditions          = conditions.noise.propulsors[turbojet.tag]  
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
    mdot_fuel                                      = turbojet_conditions.fuel_flow_rate   
    h_e_c                                          = core_nozzle_conditions.outputs.static_enthalpy
    h_0                                            = turbojet.working_fluid.compute_cp(T,P) * T 
    h_t4                                           = combustor_conditions.outputs.stagnation_enthalpy
    h_t3                                           = hpc_conditions.outputs.stagnation_enthalpy 
    turbojet_conditions.overall_efficiency         = thrust_vector* U0 / (mdot_fuel * fuel_enthalpy)  
    turbojet_conditions.thermal_efficiency         = 1 - ((mdot_air_core +  mdot_fuel)*(h_e_c -  h_0) + mdot_fuel *h_0)/((mdot_air_core +  mdot_fuel)*h_t4 - mdot_air_core *h_t3)  
 

    # compute shaft RPMs   
    lpc_conditions.omega        = low_pressure_compressor.design_angular_velocity * turbojet_conditions.throttle
    hpc_conditions.omega        = high_pressure_compressor.design_angular_velocity * turbojet_conditions.throttle
   
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

    lpc_res = Data(
                angular_velocity    =  lpc_conditions.omega, 
            )    

    noise_conditions.fan_nozzle             = None 
    noise_conditions.core_nozzle            = core_nozzle_res
    noise_conditions.fan                    = lpc_res   
    stored_results_flag                     = True
    stored_propulsor_tag                    = turbojet.tag
    
    power_elec =  0*state.ones_row(1)
    
    return thrust_vector,moment,power,power_elec,stored_results_flag,stored_propulsor_tag 

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
    afterburner                 = turbojet.afterburner
    core_nozzle                 = turbojet.core_nozzle
    ram_0                       = network.propulsors[stored_propulsor_tag].ram
    inlet_nozzle_0              = network.propulsors[stored_propulsor_tag].inlet_nozzle 
    low_pressure_compressor_0   = network.propulsors[stored_propulsor_tag].low_pressure_compressor
    high_pressure_compressor_0  = network.propulsors[stored_propulsor_tag].high_pressure_compressor
    combustor_0                 = network.propulsors[stored_propulsor_tag].combustor
    high_pressure_turbine_0     = network.propulsors[stored_propulsor_tag].high_pressure_turbine
    low_pressure_turbine_0      = network.propulsors[stored_propulsor_tag].low_pressure_turbine
    core_nozzle_0               = network.propulsors[stored_propulsor_tag].core_nozzle
    afterburner_0               = network.propulsors[stored_propulsor_tag].afterburner
    # deep copy results 
    conditions.energy.propulsors[turbojet.tag]                 = deepcopy(conditions.energy.propulsors[stored_propulsor_tag])
    conditions.noise.propulsors[turbojet.tag]                  = deepcopy(conditions.noise.propulsors[stored_propulsor_tag]) 
    conditions.energy.converters[ram.tag]                      = deepcopy(conditions.energy.converters[ram_0.tag]                     )
    conditions.energy.converters[inlet_nozzle.tag]             = deepcopy(conditions.energy.converters[inlet_nozzle_0.tag]            ) 
    conditions.energy.converters[low_pressure_compressor.tag]  = deepcopy(conditions.energy.converters[low_pressure_compressor_0.tag] )
    conditions.energy.converters[high_pressure_compressor.tag] = deepcopy(conditions.energy.converters[high_pressure_compressor_0.tag])
    conditions.energy.converters[combustor.tag]                = deepcopy(conditions.energy.converters[combustor_0.tag]               )
    conditions.energy.converters[low_pressure_turbine.tag]     = deepcopy(conditions.energy.converters[low_pressure_turbine_0.tag]    )
    conditions.energy.converters[high_pressure_turbine.tag]    = deepcopy(conditions.energy.converters[high_pressure_turbine_0.tag]   )
    conditions.energy.converters[core_nozzle.tag]              = deepcopy(conditions.energy.converters[core_nozzle_0.tag]             )
    conditions.energy.converters[afterburner.tag]              = deepcopy(conditions.energy.converters[afterburner_0.tag]             )
    # compute moment  
    moment_vector      = 0*state.ones_row(3)
    thrust_vector      = 0*state.ones_row(3)
    thrust_vector[:,0] = conditions.energy.propulsors[turbojet.tag].thrust[:,0] 
    moment_vector[:,0] = turbojet.origin[0][0] -   center_of_gravity[0][0] 
    moment_vector[:,1] = turbojet.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2] = turbojet.origin[0][2]  -  center_of_gravity[0][2]
    moment             = np.cross(moment_vector,thrust_vector)    
  
    power                                             = conditions.energy.propulsors[turbojet.tag].power 
    conditions.energy.propulsors[turbojet.tag].moment = moment

    power_elec = 0*state.ones_row(1)
    if low_pressure_compressor.motor != None and  len(state.numerics.time.differentiate) > 0: 
        conditions.energy.converters[low_pressure_compressor.motor.tag]  = deepcopy(conditions.energy.converters[low_pressure_compressor_0.motor.tag]) 
        power_elec =  conditions.energy.converters[low_pressure_compressor.motor.tag].outputs.power  
    
    if low_pressure_compressor.generator != None and len(state.numerics.time.differentiate) > 0:  
        conditions.energy.converters[low_pressure_compressor.generator.tag]  = deepcopy(conditions.energy.converters[low_pressure_compressor_0.generator.tag]) 
        power_elec =  conditions.energy.converters[low_pressure_compressor.generator.tag].inputs.power
        
    return thrust_vector,moment,power, power_elec
 