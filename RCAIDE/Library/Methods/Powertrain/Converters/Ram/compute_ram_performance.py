# RCAIDE/Library/Methods/Powertrain/Converters/Ram/compute_ram_performance.py
# 
# Created:  Jun 2024, M. Clarke    

# ----------------------------------------------------------------------------------------------------------------------
# compute_ram_performance
# ----------------------------------------------------------------------------------------------------------------------     
def compute_ram_performance(ram, conditions):
    """
    Computes the thermodynamic properties of air at the inlet of a propulsion system.
    
    Parameters
    ----------
    ram : RCAIDE.Library.Components.Converters.Ram
        Ram air converter component with the following attributes:
            - tag : str
                Identifier for the ram air converter
            - working_fluid : Data
                Working fluid properties object
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream : Data
                Freestream properties
                - pressure : numpy.ndarray
                    Freestream static pressure [Pa]
                - temperature : numpy.ndarray
                    Freestream static temperature [K]
                - mach_number : numpy.ndarray
                    Freestream Mach number
    
    Returns
    -------
    None
        Results are stored in:
        
        conditions.freestream:
            - stagnation_temperature : numpy.ndarray
                Freestream stagnation temperature [K]
            - stagnation_pressure : numpy.ndarray
                Freestream stagnation pressure [Pa]
            - isentropic_expansion_factor : numpy.ndarray
                Ratio of specific heats (gamma) [unitless]
            - specific_heat_at_constant_pressure : numpy.ndarray
                Specific heat at constant pressure [J/(kg·K)]
            - gas_specific_constant : numpy.ndarray
                Gas specific constant [J/(kg·K)]
            - speed_of_sound : numpy.ndarray
                Speed of sound [m/s]
        
        conditions.energy.converters[ram.tag].outputs:
            - stagnation_temperature : numpy.ndarray
                Stagnation temperature [K]
            - stagnation_pressure : numpy.ndarray
                Stagnation pressure [Pa]
            - isentropic_expansion_factor : numpy.ndarray
                Ratio of specific heats (gamma) [unitless]
            - specific_heat_at_constant_pressure : numpy.ndarray
                Specific heat at constant pressure [J/(kg·K)]
            - gas_specific_constant : numpy.ndarray
                Gas specific constant [J/(kg·K)]
            - static_temperature : numpy.ndarray
                Static temperature [K]
            - static_pressure : numpy.ndarray
                Static pressure [Pa]
            - mach_number : numpy.ndarray
                Mach number
            - velocity : numpy.ndarray
                Velocity [m/s]
            - speed_of_sound : numpy.ndarray
                Speed of sound [m/s]
    
    Notes
    -----
    This function computes the stagnation (total) properties of the air at the inlet
    of a propulsion system based on the freestream conditions. It calculates the
    stagnation temperature and pressure using isentropic flow relations, and also
    computes various thermodynamic properties of the working fluid.
    
    The computation follows these steps:
        1. Extract freestream conditions (pressure, temperature, Mach number)
        2. Compute working fluid properties (gamma, Cp, R, speed of sound)
        3. Calculate freestream velocity from Mach number and speed of sound
        4. Compute stagnation temperature and pressure using isentropic flow relations
        5. Store results in both freestream and ram output conditions
    
    **Major Assumptions**
        * Isentropic flow from freestream to inlet
        * No losses in the inlet
        * Working fluid properties are computed at freestream conditions
    
    **Theory**
    The stagnation temperature and pressure are calculated using the following isentropic relations:
    
    .. math::
        T_0 = T \\left(1 + \\frac{\\gamma - 1}{2} M^2 \\right)
    
    .. math::
        P_0 = P \\left(1 + \\frac{\\gamma - 1}{2} M^2 \\right)^{\\gamma/(\\gamma-1)}
    
    where:
    - T_0 is the stagnation temperature
    - T is the static temperature
    - P_0 is the stagnation pressure
    - P is the static pressure
    - γ is the ratio of specific heats
    - M is the Mach number
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Compressor.compute_compressor_performance
    """
    # Unpack flight conditions 
    M0 = conditions.freestream.mach_number
    P0 = conditions.freestream.pressure
    T0 = conditions.freestream.temperature

    # Unpack ram inputs
    working_fluid  = ram.working_fluid
    ram_conditions = conditions.energy.converters[ram.tag]
 
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