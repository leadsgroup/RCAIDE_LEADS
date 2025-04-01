# RCAIDE/Library/Methods/Powertrain/Converters/Fan/compute_fan_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports
import numpy as np 
from RCAIDE.Library.Methods.Gas_Dynamics.fm_id import fm_id

# exceptions/warnings
from warnings import warn

# ----------------------------------------------------------------------------------------------------------------------
#  compute_expansion_nozzle_performance
# ----------------------------------------------------------------------------------------------------------------------        
def compute_expansion_nozzle_performance(expansion_nozzle, nozzle_conditions, conditions):
    """
    Computes the thermodynamic properties at the exit of an expansion nozzle.
    
    Parameters
    ----------
    expansion_nozzle : ExpansionNozzle
        The expansion nozzle component with the following attributes:
            - pressure_ratio : float or numpy.ndarray
                Ratio of exit to inlet pressure
            - polytropic_efficiency : float or numpy.ndarray
                Efficiency of the expansion process
            - working_fluid : FluidModel
                Object containing methods to compute fluid properties
    nozzle_conditions : Conditions
        Object containing nozzle inlet conditions with the following attributes:
            - inputs.stagnation_temperature : numpy.ndarray
                Inlet stagnation temperature [K]
            - inputs.stagnation_pressure : numpy.ndarray
                Inlet stagnation pressure [Pa]
            - inputs.static_temperature : numpy.ndarray
                Inlet static temperature [K]
            - inputs.static_pressure : numpy.ndarray
                Inlet static pressure [Pa]
    conditions : Conditions
        Object containing freestream conditions with the following attributes:
            - freestream.mach_number : numpy.ndarray
                Freestream Mach number [unitless]
            - freestream.pressure : numpy.ndarray
                Ambient pressure [Pa]
            - freestream.stagnation_pressure : numpy.ndarray
                Freestream stagnation pressure [Pa]
            - freestream.stagnation_temperature : numpy.ndarray
                Freestream stagnation temperature [K]
            - freestream.specific_heat_at_constant_pressure : numpy.ndarray
                Specific heat capacity [J/(kg·K)]
            - freestream.isentropic_expansion_factor : numpy.ndarray
                Ratio of specific heats (gamma) [unitless]
            - freestream.specific_gas_constant : numpy.ndarray
                Gas constant [J/(kg·K)]
    
    Returns
    -------
    None
        This function modifies the nozzle_conditions.outputs object in-place with the following attributes:
            - stagnation_temperature : numpy.ndarray
                Exit stagnation temperature [K]
            - stagnation_pressure : numpy.ndarray
                Exit stagnation pressure [Pa]
            - stagnation_enthalpy : numpy.ndarray
                Exit stagnation enthalpy [J/kg]
            - mach_number : numpy.ndarray
                Exit Mach number [unitless]
            - static_temperature : numpy.ndarray
                Exit static temperature [K]
            - static_pressure : numpy.ndarray
                Exit static pressure [Pa]
            - static_enthalpy : numpy.ndarray
                Exit static enthalpy [J/kg]
            - velocity : numpy.ndarray
                Exit nozzle velocity [m/s]
            - area_ratio : numpy.ndarray
                Ratio of exit to throat area [unitless]
    
    Notes
    -----
    This function computes the thermodynamic properties at the exit of an expansion nozzle
    using gas dynamic relations. It handles both subsonic and choked flow conditions.
    
    **Major Assumptions**
        * Constant polytropic efficiency and pressure ratio
        * If pressures make the Mach number go negative, these values are corrected
        * Adiabatic process
    
    **Theory**
    
    For subsonic flow, isentropic relations are used:
    
    .. math::
        M_{out} = \\sqrt{\\frac{2}{(\\gamma-1)}\\left[\\left(\\frac{P_{t,out}}{P_0}\\right)^{\\frac{\\gamma-1}{\\gamma}}-1\\right]}
    
    For choked flow, the exit pressure is calculated from:
    
    .. math::
        P_{out} = \\frac{P_{t,out}}{\\left(1+\\frac{\\gamma-1}{2}M_{out}^2\\right)^{\\frac{\\gamma}{\\gamma-1}}}
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_BOOK/AA283_Aircraft_and_Rocket_Propulsion_BOOK_Brian_J_Cantwell_May_28_2024.pdf

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle.append_expansion_nozzle_conditions
    RCAIDE.Library.Methods.Gas_Dynamics.fm_id
    """                 
    # Unpack flight conditions     
    M0       = conditions.freestream.mach_number
    P0       = conditions.freestream.pressure
    Pt0      = conditions.freestream.stagnation_pressure
    Tt0      = conditions.freestream.stagnation_temperature
    
    # Unpack exansion nozzle inputs
    Tt_in    = nozzle_conditions.inputs.stagnation_temperature
    Pt_in    = nozzle_conditions.inputs.stagnation_pressure 
    PR       = expansion_nozzle.pressure_ratio
    etapold  = expansion_nozzle.polytropic_efficiency
    
    P_in  = nozzle_conditions.inputs.static_pressure    
    T_in  = nozzle_conditions.inputs.static_temperature
    
    # Unpack ram inputs       
    working_fluid  = expansion_nozzle.working_fluid 
    gamma          = working_fluid.compute_gamma(T_in,P_in) 
    Cp             = working_fluid.compute_cp(T_in,P_in)    
     
    # Compute output stagnation quantities
    Pt_out   = Pt_in*PR
    Tt_out   = Tt_in*PR**((gamma-1)/(gamma)*etapold)
    ht_out   = Cp*Tt_out
    
    # A cap so pressure doesn't go negative
    Pt_out[Pt_out<P0] = P0[Pt_out<P0]
    
    # Compute the output Mach number, static quantities and the output velocity
    Mach          = np.sqrt((((Pt_out/P0)**((gamma-1)/gamma))-1)*2/(gamma-1)) 
    
    #initializing the Pout array
    P_out         = np.ones_like(Mach)
    
    # Computing output pressure and Mach number for the case Mach <1.0
    i_low         = Mach < 1.0
    P_out[i_low]  = P0[i_low]
    Mach[i_low]   = np.sqrt((((Pt_out[i_low]/P0[i_low])**((gamma[i_low]-1.)/gamma[i_low]))-1.)*2./(gamma[i_low]-1.))
    
    # Computing output pressure and Mach number for the case Mach >=1.0     
    i_high        = Mach >=1.0   
    Mach[i_high]  = Mach[i_high]/Mach[i_high]
    P_out[i_high] = Pt_out[i_high]/(1.+(gamma[i_high]-1.)/2.*Mach[i_high]*Mach[i_high])**(gamma[i_high]/(gamma[i_high]-1.))
    
    # A cap to make sure Mach doesn't go to zero:
    if np.any(Mach<=0.0):
        warn('Pressures Result in Negative Mach Number, making positive',RuntimeWarning)
        Mach[Mach<=0.0] = 0.001
    
    # Compute the output temperature,enthalpy,velocity and density
    T_out         = Tt_out/(1+(gamma-1)/2*Mach*Mach)
    h_out         = T_out * Cp
    u_out         = np.sqrt(2*(ht_out-h_out))
    #rho_out       = P_out/(R*T_out)
    
    # Compute the freestream to nozzle area ratio  
    area_ratio    = (fm_id(M0,gamma)/fm_id(Mach,gamma)*(1/(Pt_out/Pt0))*(np.sqrt(Tt_out/Tt0)))
    
    #pack computed quantities into outputs
    nozzle_conditions.outputs.area_ratio              = area_ratio
    nozzle_conditions.outputs.mach_number             = Mach
    #nozzle_conditions.outputs.density                 = rho_out
    nozzle_conditions.outputs.velocity                = u_out
    nozzle_conditions.outputs.static_pressure         = P_out
    nozzle_conditions.outputs.static_temperature      = T_out
    nozzle_conditions.outputs.static_enthalpy         = h_out
    nozzle_conditions.outputs.stagnation_temperature  = Tt_out
    nozzle_conditions.outputs.stagnation_pressure     = Pt_out
    nozzle_conditions.outputs.stagnation_enthalpy     = ht_out
    
    return 
