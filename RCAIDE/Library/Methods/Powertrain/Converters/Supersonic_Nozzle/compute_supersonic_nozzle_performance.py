# RCAIDE/Library/Methods/Powertrain/Converters/Compression_Nozzle/compute_supersonic_nozzle_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------     

# package imports
import numpy as np   
from RCAIDE.Library.Methods.Gas_Dynamics.fm_id import fm_id

# ---------------------------------------------------------------------------------------------------------------------- 
# compute_compression_nozzle_performance
# ----------------------------------------------------------------------------------------------------------------------    
def compute_supersonic_nozzle_performance(supersonic_nozzle, s_nozzle_conditions, conditions):
    """
    Computes the thermodynamic properties at the exit of a supersonic nozzle.
    
    Parameters
    ----------
    supersonic_nozzle : SupersonicNozzle
        The supersonic nozzle component with the following attributes:
            - pressure_ratio : float or numpy.ndarray
                Ratio of exit to inlet pressure
            - polytropic_efficiency : float or numpy.ndarray
                Efficiency of the expansion process
            - pressure_recovery : float or numpy.ndarray
                Factor accounting for pressure losses
    s_nozzle_conditions : Conditions
        Object containing nozzle inlet conditions with the following attributes:
            - inputs.stagnation_temperature : numpy.ndarray
                Inlet stagnation temperature [K]
            - inputs.stagnation_pressure : numpy.ndarray
                Inlet stagnation pressure [Pa]
    conditions : Conditions
        Object containing freestream conditions with the following attributes:
            - freestream.isentropic_expansion_factor : numpy.ndarray
                Ratio of specific heats (gamma) [unitless]
            - freestream.specific_heat_at_constant_pressure : numpy.ndarray
                Specific heat capacity [J/(kg·K)]
            - freestream.pressure : numpy.ndarray
                Ambient pressure [Pa]
            - freestream.stagnation_pressure : numpy.ndarray
                Freestream stagnation pressure [Pa]
            - freestream.stagnation_temperature : numpy.ndarray
                Freestream stagnation temperature [K]
            - freestream.gas_specific_constant : numpy.ndarray
                Gas constant [J/(kg·K)]
            - freestream.mach_number : numpy.ndarray
                Freestream Mach number [unitless]
    
    Returns
    -------
    None
        This function modifies the s_nozzle_conditions.outputs object in-place with the following attributes:
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
            - density : numpy.ndarray
                Exit density [kg/m³]
            - static_enthalpy : numpy.ndarray
                Exit static enthalpy [J/kg]
            - velocity : numpy.ndarray
                Exit nozzle velocity [m/s]
            - static_pressure : numpy.ndarray
                Exit static pressure [Pa]
            - area_ratio : numpy.ndarray
                Ratio of exit to throat area [unitless]
    
    Notes
    -----
    This function computes the thermodynamic properties at the exit of a supersonic nozzle
    using gas dynamic relations. It handles both supersonic and subsonic flow conditions.
    
    **Major Assumptions**
        * Constant polytropic efficiency and pressure ratio
        * Adiabatic process
        * Mach number is limited to a maximum of 10.0
    
    **Theory**
    
    For supersonic flow, the exit Mach number is calculated using:
    
    .. math::
        M_{out} = \\sqrt{\\frac{2}{(\\gamma-1)}\\left[\\left(\\frac{P_{t,out}}{P_0}\\right)^{\\frac{\\gamma-1}{\\gamma}}-1\\right]}
    
    The area ratio is calculated using the mass flow parameter:
    
    .. math::
        \\frac{A_{out}}{A_{throat}} = \\frac{fm(M_0, \\gamma)}{fm(M_{out}, \\gamma)} \\cdot \\frac{1}{P_{t,out}/P_{t,0}} \\cdot \\sqrt{\\frac{T_{t,out}}{T_{t,0}}}
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_BOOK/AA283_Aircraft_and_Rocket_Propulsion_BOOK_Brian_J_Cantwell_May_28_2024.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Supersonic_Nozzle.append_supersonic_nozzle_conditions
    RCAIDE.Library.Methods.Gas_Dynamics.fm_id
    """           
    
    #unpack the values
    
    #unpack from conditions
    gamma    = conditions.freestream.isentropic_expansion_factor
    Cp       = conditions.freestream.specific_heat_at_constant_pressure
    Po       = conditions.freestream.pressure
    Pto      = conditions.freestream.stagnation_pressure
    Tto      = conditions.freestream.stagnation_temperature
    R        = conditions.freestream.gas_specific_constant
    Mo       = conditions.freestream.mach_number
    
    #unpack from inputs
    Tt_in    = s_nozzle_conditions.inputs.stagnation_temperature
    Pt_in    = s_nozzle_conditions.inputs.stagnation_pressure
     
    pid      = supersonic_nozzle.pressure_ratio
    etapold  = supersonic_nozzle.polytropic_efficiency
    eta_rec  = supersonic_nozzle.pressure_recovery
    
    #Method for computing the nozzle properties
    
    #--Getting the output stagnation quantities
    Pt_out   = Pt_in*pid*eta_rec
    Tt_out   = Tt_in*(pid*eta_rec)**((gamma-1)/(gamma)*etapold)
    ht_out   = Cp*Tt_out
    
    
    #compute the output Mach number, static quantities and the output velocity
    Mach          = np.sqrt((((Pt_out/Po)**((gamma-1)/gamma))-1)*2/(gamma-1))
    
    #Remove check on mach numbers fromn expansion nozzle
    i_low         = Mach < 1.0
    
    #initializing the Pout array
    P_out         = 1.0 *Mach/Mach
    
    #Computing output pressure and Mach number for the case Mach <1.0
    P_out[i_low]  = Po[i_low]
    Mach[i_low]   = np.sqrt((((Pt_out[i_low]/Po[i_low])**((gamma[i_low]-1.)/gamma[i_low]))-1.)*2./(gamma[i_low]-1.))
    
    #Computing the output temperature,enthalpy, velocity and density
    T_out         = Tt_out/(1.+(gamma-1.)/2.*Mach*Mach)
    h_out         = Cp*T_out
    u_out         = np.sqrt(2.*(ht_out-h_out))
    rho_out       = P_out/(R*T_out)
    
    #Computing the freestream to nozzle area ratio (mainly from thrust computation)
    area_ratio    = (fm_id(Mo,gamma)/fm_id(Mach,gamma)*(1/(Pt_out/Pto))*(np.sqrt(Tt_out/Tto)))
    
    #pack computed quantities into outputs
    s_nozzle_conditions.outputs.stagnation_temperature  = Tt_out
    s_nozzle_conditions.outputs.stagnation_pressure     = Pt_out
    s_nozzle_conditions.outputs.stagnation_enthalpy     = ht_out
    s_nozzle_conditions.outputs.mach_number             = Mach
    s_nozzle_conditions.outputs.static_temperature      = T_out
    s_nozzle_conditions.outputs.density                 = rho_out
    s_nozzle_conditions.outputs.static_enthalpy         = h_out
    s_nozzle_conditions.outputs.velocity                = u_out
    s_nozzle_conditions.outputs.static_pressure         = P_out
    s_nozzle_conditions.outputs.area_ratio              = area_ratio
        
    return 