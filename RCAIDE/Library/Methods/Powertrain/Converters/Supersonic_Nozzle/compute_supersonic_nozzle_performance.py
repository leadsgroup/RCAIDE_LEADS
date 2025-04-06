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
def compute_supersonic_nozzle_performance(supersonic_nozzle,conditions): 
    """
    Computes the performance parameters of a supersonic nozzle based on input conditions.
    
    Parameters
    ----------
    supersonic_nozzle : RCAIDE.Components.Energy.Converters.Supersonic_Nozzle
        The supersonic nozzle component for which performance is being computed
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Container for flight conditions and energy system states
        
    Returns
    -------
    None
        This function modifies the conditions object in-place
    
    Notes
    -----
    This function calculates output values from input conditions according to 
    gas dynamics equations for supersonic nozzles.
    
    **Major Assumptions**
        * Constant polytropic efficiency and pressure ratio
        * Isentropic flow except for losses accounted by efficiency terms
        * Perfect gas behavior
    
    **Theory**
    
    The nozzle performance is calculated using isentropic flow relations. For supersonic flow,
    the Mach number is calculated from the pressure ratio:
    
    .. math::
        M = \\sqrt{\\frac{2}{\\gamma-1}\\left[\\left(\\frac{P_t}{P_0}\\right)^{\\frac{\\gamma-1}{\\gamma}}-1\\right]}
    
    The static temperature is related to stagnation temperature by:
    
    .. math::
        T = \\frac{T_t}{1+\\frac{\\gamma-1}{2}M^2}
    
    **Definitions**
    
    'Pressure ratio'
        Ratio of exit pressure to inlet pressure in the nozzle
    
    'Polytropic efficiency'
        Measure of the nozzle's thermodynamic efficiency accounting for irreversibilities
    
    References
    ----------
    [1] Cantwell, B. "AA283 Course Material: Course Notes." Stanford University. https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/
    
    See Also
    --------
    RCAIDE.Library.Methods.Gas_Dynamics.fm_id
    """           
    
    #unpack the values
    
    #unpack from conditions
    gamma               = conditions.freestream.isentropic_expansion_factor
    Cp                  = conditions.freestream.specific_heat_at_constant_pressure
    Po                  = conditions.freestream.pressure
    Pto                 = conditions.freestream.stagnation_pressure
    Tto                 = conditions.freestream.stagnation_temperature
    R                   = conditions.freestream.gas_specific_constant
    Mo                  = conditions.freestream.mach_number
    s_nozzle_conditions = conditions.energy.converters[supersonic_nozzle.tag]
    
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