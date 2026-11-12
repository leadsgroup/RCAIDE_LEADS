# RCAIDE/Library/Methods/Powertrain/Converters/Compression_Nozzle/compute_compression_nozzle_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------     

# package imports
import numpy as np
from RCAIDE.Framework.Core import array_namespace
from warnings import warn

# ---------------------------------------------------------------------------------------------------------------------- 
# compute_compression_nozzle_performance
# ----------------------------------------------------------------------------------------------------------------------    
def compute_compression_nozzle_performance(compression_nozzle, conditions):
    """
    Computes the performance of a compression nozzle based on its polytropic efficiency.
    
    Parameters
    ----------
    compression_nozzle : Data
        Data dictionary with compression nozzle properties
            - tag : str
                Identifier for the compression nozzle
            - pressure_ratio : float
                Pressure ratio across the nozzle [unitless]
            - polytropic_efficiency : float
                Polytropic efficiency of the nozzle [unitless]
            - pressure_recovery : float
                Pressure recovery factor [unitless]
            - compressibility_effects : bool
                Flag to include compressibility effects
            - working_fluid : Data
                Working fluid object with methods to compute properties
    conditions : Data
        Data dictionary with flow conditions
            - freestream : Data
                Freestream flow properties
                - pressure : ndarray
                    Freestream pressure [Pa]
                - mach_number : ndarray
                    Freestream Mach number [unitless]
            - energy : Data
                Energy conditions
                    - converters : dict
                        Dictionary of converter conditions indexed by tag
                        - inputs : Data
                            Input conditions
                            - stagnation_temperature : ndarray
                                Entering stagnation temperature [K]
                            - stagnation_pressure : ndarray
                                Entering stagnation pressure [Pa]
                            - static_temperature : ndarray
                                Entering static temperature [K]
                            - static_pressure : ndarray
                                Entering static pressure [Pa]
                            - mach_number : ndarray
                                Entering Mach number [unitless]
    
    Returns
    -------
    None
        Results are stored in conditions.energy.converters[compression_nozzle.tag].outputs:
            - stagnation_temperature : ndarray
                Exit stagnation temperature [K]
            - stagnation_pressure : ndarray
                Exit stagnation pressure [Pa]
            - stagnation_enthalpy : ndarray
                Exit stagnation enthalpy [J/kg]
            - mach_number : ndarray
                Exit Mach number [unitless]
            - static_temperature : ndarray
                Exit static temperature [K]
            - static_enthalpy : ndarray
                Exit static enthalpy [J/kg]
            - velocity : ndarray
                Exit nozzle velocity [m/s]
            - static_pressure : ndarray
                Exit static pressure [Pa]
    
    Notes
    -----
    This function computes the thermodynamic properties at the exit of a compression nozzle
    based on the inlet conditions and nozzle characteristics. It handles both subsonic and
    supersonic flows with appropriate relations.
    
    **Major Assumptions**
        * Pressure ratio and polytropic efficiency do not change with varying conditions
        * Adiabatic process
        * Subsonic or choked output
    
    **Theory**
    The compression nozzle performance is calculated using gas dynamics relations for
    compressible flow. For subsonic flow, isentropic relations are used. For supersonic flow,
    normal shock relations are applied. The stagnation properties are transformed to static
    properties based on the exit Mach number.
    
    References
    ----------
    [1] Stanford University, "AA283 Course Notes", https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle.compute_expansion_nozzle_performance
    """

    # Unpack conditions 
    P0                = conditions.freestream.pressure
    M0                = conditions.freestream.mach_number
    nozzle_conditions = conditions.energy.converters[compression_nozzle.tag]

    # Unpack inpust
    Tt_in                   = nozzle_conditions.inputs.stagnation_temperature
    Pt_in                   = nozzle_conditions.inputs.stagnation_pressure
    PR                      = compression_nozzle.pressure_ratio
    eta_p_old               = compression_nozzle.polytropic_efficiency
    eta_rec                 = compression_nozzle.pressure_recovery
    compressibility_effects = compression_nozzle.compressibility_effects
    
    # Unpack ram inputs
    working_fluid           = compression_nozzle.working_fluid
 
    # Compute the working fluid properties  
    T0     = nozzle_conditions.inputs.static_temperature
    P0     = nozzle_conditions.inputs.static_pressure   
    M0     = nozzle_conditions.inputs.mach_number 
    gamma  = working_fluid.compute_gamma(T0,P0) 
    Cp     = working_fluid.compute_cp(T0,P0)  
    a      = working_fluid.compute_speed_of_sound(T0,P0)    
 
    # Compute output stagnation quantities
    Pt_out  = Pt_in*PR*eta_rec
    Tt_out  = Tt_in*(PR*eta_rec)**((gamma-1)/(gamma*eta_p_old))
    ht_out  = Tt_out*Cp 

    xp = array_namespace(Tt_in, Pt_in, P0, M0)
    if compressibility_effects:
        # isentropic relations where the inflow is subsonic, a normal shock where it is supersonic (Anderson, Modern
        # Compressible Flow, Sec. 3.6); each branch is kept finite where it is not used
        subsonic = M0 <= 1.0
        Pt_low   = Pt_in*PR
        M_low    = xp.sqrt(xp.maximum(((Pt_low/P0)**((gamma-1.)/gamma))-1., 0.) *2./(gamma-1.))
        T_low    = Tt_out/(1.+(gamma-1.)/2.*M_low*M_low)
        P_low    = Pt_low/((1.+(gamma-1.)/2.*M_low*M_low)**(gamma/(gamma-1.)))

        M1       = xp.where(subsonic, 2.0, M0)
        M_high   = xp.sqrt((1.+(gamma-1.)/2.*M1**2.)/(gamma*M1**2-(gamma-1.)/2.))
        T_high   = Tt_out/(1.+(gamma-1.)/2*M_high*M_high)
        Pt_high  = PR*Pt_in*((((gamma+1.)*(M1**2.))/((gamma-1.)*M1**2.+2.))**(gamma/(gamma-1.)))*\
                   ((gamma+1.)/(2.*gamma*M1**2.-(gamma-1.)))**(1./(gamma-1.))
        P_high   = Pt_high/(1.+(gamma-1.)/2.*M_high**2.)**(gamma/(gamma-1.))

        Pt_out   = xp.where(subsonic, Pt_low, Pt_high)
        M_out    = xp.where(subsonic, M_low, M_high)
        T_out    = xp.where(subsonic, T_low, T_high)
        P_out    = xp.where(subsonic, P_low, P_high)
    else:
        Pt_out  = Pt_in*PR*eta_rec 
        too_low = Pt_out < P0
        if xp is np and np.any(too_low): # in case pressures go too low
            warn('Pt_out goes too low',RuntimeWarning)
        Pt_out  = xp.where(too_low, P0, Pt_out)
        # zero where the pressure was capped, where the square root's derivative is infinite
        M_squared = (((Pt_out/P0)**((gamma-1.)/gamma))-1.) *2./(gamma-1.)
        M_out   = xp.where(too_low, 0., xp.sqrt(xp.where(too_low, 1., M_squared)))
        T_out  = Tt_out/(1.+(gamma-1.)/2.*M_out*M_out)
        P_out  = Pt_out/(1.+(gamma-1.)/2.*M_out*M_out)**(gamma/(gamma-1.))
        
    # Compute exit ethalpy and velocity  
    h_out   = Cp*T_out
    u_out   = xp.sqrt(2.*(ht_out-h_out))

    # Pack computed quantities into outputs
    nozzle_conditions.outputs.mach_number             = M_out
    nozzle_conditions.outputs.velocity                = u_out
    nozzle_conditions.outputs.static_enthalpy         = h_out
    nozzle_conditions.outputs.static_temperature      = T_out
    nozzle_conditions.outputs.static_pressure         = P_out
    nozzle_conditions.outputs.stagnation_enthalpy     = ht_out
    nozzle_conditions.outputs.stagnation_temperature  = Tt_out
    nozzle_conditions.outputs.stagnation_pressure     = Pt_out
    
    return 