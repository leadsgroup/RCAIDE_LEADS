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
def compute_expansion_nozzle_performance(expansion_nozzle, conditions):
    """
    Computes the thermodynamic performance of an expansion nozzle in a propulsion system.
    
    Parameters
    ----------
    expansion_nozzle : RCAIDE.Library.Components.Converters.Expansion_Nozzle
        Expansion nozzle component with the following attributes:
            - tag : str
                Identifier for the nozzle
            - working_fluid : Data
                Working fluid properties object
            - pressure_ratio : float
                Pressure ratio across the nozzle
            - polytropic_efficiency : float
                Polytropic efficiency of the expansion process
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream : Data
                Freestream properties
                - isentropic_expansion_factor : numpy.ndarray
                    Ratio of specific heats (gamma)
                - specific_heat_at_constant_pressure : numpy.ndarray
                    Specific heat at constant pressure [J/(kg·K)]
                - pressure : numpy.ndarray
                    Freestream pressure [Pa]
                - stagnation_pressure : numpy.ndarray
                    Freestream stagnation pressure [Pa]
                - stagnation_temperature : numpy.ndarray
                    Freestream stagnation temperature [K]
                - specific_gas_constant : numpy.ndarray
                    Specific gas constant [J/(kg·K)]
                - mach_number : numpy.ndarray
                    Freestream Mach number
            - energy : Data
                Energy conditions
                    - converters : dict
                        Converter energy conditions indexed by tag
    
    Returns
    -------
    None
        Results are stored in conditions.energy.converters[expansion_nozzle.tag].outputs:
            - stagnation_temperature : numpy.ndarray
                Stagnation temperature at nozzle exit [K]
            - stagnation_pressure : numpy.ndarray
                Stagnation pressure at nozzle exit [Pa]
            - stagnation_enthalpy : numpy.ndarray
                Stagnation enthalpy at nozzle exit [J/kg]
            - mach_number : numpy.ndarray
                Mach number at nozzle exit
            - static_temperature : numpy.ndarray
                Static temperature at nozzle exit [K]
            - static_enthalpy : numpy.ndarray
                Static enthalpy at nozzle exit [J/kg]
            - velocity : numpy.ndarray
                Exit velocity [m/s]
            - static_pressure : numpy.ndarray
                Static pressure at nozzle exit [Pa]
            - area_ratio : numpy.ndarray
                Exit to freestream area ratio
    
    Notes
    -----
    This function computes the thermodynamic properties at the expansion nozzle exit based on
    the inlet conditions and nozzle characteristics. It calculates the exit velocity, pressure,
    temperature, and other properties for both subsonic and supersonic flow regimes.
    
    The computation follows these steps:
        1. Extract freestream and inlet conditions
        2. Compute working fluid properties (gamma, Cp)
        3. Calculate stagnation conditions at exit using pressure ratio
        4. Compute exit Mach number based on pressure ratio
        5. Handle subsonic (M < 1.0) and supersonic (M ≥ 1.0) cases separately
        6. Calculate static conditions (temperature, pressure) at exit
        7. Compute exit velocity from energy conservation
        8. Calculate area ratio between freestream and nozzle exit
        9. Store all results in the conditions data structure
    
    **Major Assumptions**
        * Constant polytropic efficiency and pressure ratio
        * If pressures make the Mach number go negative, these values are corrected
    
    **Theory**
    For subsonic flow (M < 1.0), the exit pressure equals the ambient pressure, and the
    Mach number is calculated from the pressure ratio. For supersonic flow (M ≥ 1.0),
    the nozzle is choked, and the exit pressure is calculated from the Mach number.
    
    The exit velocity is calculated from the conservation of energy:
    
    .. math::
        u_{out} = \\sqrt{2(h_{t,out} - h_{out})}
    
    where h_{t,out} is the exit stagnation enthalpy and h_{out} is the exit static enthalpy.
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/
    
    See Also
    --------
    RCAIDE.Library.Methods.Gas_Dynamics.fm_id
    """                 
    # Unpack flight conditions     
    M0                = conditions.freestream.mach_number
    P0                = conditions.freestream.pressure
    Pt0               = conditions.freestream.stagnation_pressure
    Tt0               = conditions.freestream.stagnation_temperature
    nozzle_conditions = conditions.energy.converters[expansion_nozzle.tag]
    
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
    sup =  Pt_out<P0
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
    Mach[sup]  = np.sqrt((((Pt_out[sup]/P0[sup])**((gamma[sup]-1)/gamma[sup]))-1)*2/(gamma[sup]-1)) 
    Mach[sup]  = Mach[sup]/Mach[sup]
    P_out[sup] = Pt_out[sup]/(1.+((gamma[sup]-1.)/2.)*Mach[sup]**2)**(gamma[sup]/(gamma[sup]-1.)) 
    
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
