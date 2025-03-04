# RCAIDE/Library/Methods/Powertrain/Converters/Ram/compute_ram_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke    

# ----------------------------------------------------------------------------------------------------------------------
# compute_ram_performance
# ----------------------------------------------------------------------------------------------------------------------     
def compute_ram_performance(ram,ram_conditions,conditions):
    """ This computes the output values from the input values according T0
    equations from the source. The following properties are determined 
        conditions.freestream.
          stagnation_temperature              (numpy.ndarray): freestream stagnation_temperature             [K]
          stagnation_pressure                 (numpy.ndarray): freestream stagnation_pressure                [Pa]
          isentropic_expansion_factor         (numpy.ndarray): freestream isentropic_expansion_factor        [unitless]
          specific_heat_at_constant_pressure  (numpy.ndarray): freestream specific_heat_at_constant_pressure [J/(kg K)]
          gas_specific_constant               (numpy.ndarray): freestream gas_specific_constant              [J/(kg K)]
          speed_of_sound                      (numpy.ndarray): freestream speed_of_sound                     [m/s]
        ram.outputs.
          stagnation_temperature              (numpy.ndarray): exiting stagnation temperature     [K]
          stagnation_pressure                 (numpy.ndarray): exiting stagnation pressure        [Pa]
          isentropic_expansion_factor         (numpy.ndarray): isentropic expansion factor        [unitless]
          specific_heat_at_constant_pressure  (numpy.ndarray): specific_heat_at_constant_pressure [J/(kg K)]
          gas_specific_constant               (numpy.ndarray): gas_specific_constant              [J/(kg K)]   

    Assumptions:
        None

    Source:
        https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/

    Args:
        conditions.freestream.
           pressure               (numpy.ndarray): freestream pressure      [Pa]
           temperature            (numpy.ndarray): freestream temperature   [K]
           mach_number            (numpy.ndarray): freestream Mach number   [unitless]
        ram.inputs.working_fluid           (dict): working fluid properties [-]

    Returns:
        None 
  
    """
    # Unpack flight conditions 
    M0 = conditions.freestream.mach_number
    P0 = conditions.freestream.pressure
    T0 = conditions.freestream.temperature

    # Unpack ram inputs
    working_fluid  = ram.working_fluid
 
    # Compute the working fluid properties
    R        = working_fluid.gas_specific_constant
    gamma    = working_fluid.compute_gamma(T0,P0) 
    Cp       = working_fluid.compute_cp(T0,P0)
    a        = working_fluid.compute_speed_of_sound(T0,P0)
    V0       = a*M0 

    # Compute the stagnation quantities from the input static quantities
    stagnation_pressure    = P0*((1.+(gamma-1.)/2.*M0*M0 )**(gamma/(gamma-1.))) 
    stagnation_temperature = T0*(1.+((gamma-1.)/2.*M0*M0))

    # Store values into flight conditions data structure  
    conditions.freestream.isentropic_expansion_factor          = gamma
    conditions.freestream.specific_heat_at_constant_pressure   = Cp
    conditions.freestream.gas_specific_constant                = R
    conditions.freestream.stagnation_temperature               = stagnation_temperature
    conditions.freestream.stagnation_pressure                  = stagnation_pressure

    # Store values into compoment outputs  
    ram_conditions.outputs.isentropic_expansion_factor         = gamma
    ram_conditions.outputs.specific_heat_at_constant_pressure  = Cp
    ram_conditions.outputs.gas_specific_constant               = R
    ram_conditions.outputs.stagnation_temperature              = stagnation_temperature
    ram_conditions.outputs.stagnation_pressure                 = stagnation_pressure 
    ram_conditions.outputs.static_temperature                  = T0
    ram_conditions.outputs.static_pressure                     = P0
    ram_conditions.outputs.mach_number                         = M0
    ram_conditions.outputs.velocity                            = V0
    ram_conditions.outputs.speed_of_sound                      = a    
    
    return 