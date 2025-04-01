# RCAIDE/Library/Methods/Powertrain/Converters/Ram/compute_ram_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke    

# ----------------------------------------------------------------------------------------------------------------------
# compute_ram_performance
# ----------------------------------------------------------------------------------------------------------------------     
def compute_ram_performance(ram, ram_conditions, conditions):
    """
    Computes the stagnation properties of air due to ram compression.
    
    Parameters
    ----------
    ram : Ram
        The ram component with the following attributes:
            - working_fluid : FluidModel
                Object containing methods to compute fluid properties
    ram_conditions : Conditions
        Object to store ram output conditions
    conditions : Conditions
        Object containing freestream conditions with the following attributes:
            - freestream.pressure : numpy.ndarray
                Freestream static pressure [Pa]
            - freestream.temperature : numpy.ndarray
                Freestream static temperature [K]
            - freestream.mach_number : numpy.ndarray
                Freestream Mach number [unitless]
    
    Returns
    -------
    None
        This function modifies both the ram_conditions.outputs and conditions.freestream
        objects in-place with the following attributes:
            - stagnation_temperature : numpy.ndarray
                Stagnation temperature [K]
            - stagnation_pressure : numpy.ndarray
                Stagnation pressure [Pa]
            - isentropic_expansion_factor : numpy.ndarray
                Ratio of specific heats (gamma) [unitless]
            - specific_heat_at_constant_pressure : numpy.ndarray
                Specific heat capacity [J/(kg·K)]
            - gas_specific_constant : numpy.ndarray
                Gas constant [J/(kg·K)]
            - speed_of_sound : numpy.ndarray
                Speed of sound [m/s]
            - static_temperature : numpy.ndarray (ram_conditions.outputs only)
                Static temperature [K]
            - static_pressure : numpy.ndarray (ram_conditions.outputs only)
                Static pressure [Pa]
            - mach_number : numpy.ndarray (ram_conditions.outputs only)
                Mach number [unitless]
            - velocity : numpy.ndarray (ram_conditions.outputs only)
                Velocity [m/s]
    
    Notes
    -----
    This function computes the stagnation properties of air due to ram compression
    using isentropic flow relations. The stagnation properties represent the state
    that would be achieved if the flow were brought to rest isentropically.
    
    **Theory**
    
    The stagnation temperature is calculated using:
    
    .. math::
        T_t = T_0 \\left(1 + \\frac{\\gamma-1}{2}M_0^2\\right)
    
    The stagnation pressure is calculated using:
    
    .. math::
        P_t = P_0 \\left(1 + \\frac{\\gamma-1}{2}M_0^2\\right)^{\\frac{\\gamma}{\\gamma-1}}
    
    where:
        - :math:`T_0` is the freestream static temperature
        - :math:`P_0` is the freestream static pressure
        - :math:`M_0` is the freestream Mach number
        - :math:`\\gamma` is the ratio of specific heats
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_BOOK/AA283_Aircraft_and_Rocket_Propulsion_BOOK_Brian_J_Cantwell_May_28_2024.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Ram.append_ram_conditions
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