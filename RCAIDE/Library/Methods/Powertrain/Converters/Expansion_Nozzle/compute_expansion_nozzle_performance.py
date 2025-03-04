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
def compute_expansion_nozzle_performance(expansion_nozzle,nozzle_conditions,conditions):
    """ This computes the output values from the input values according to
    equations from the source. The following properties are computed: 
    expansion_nozzle.outputs.
      stagnation_temperature  (numpy.ndarray): [K]  
      stagnation_pressure     (numpy.ndarray): [Pa]
      stagnation_enthalpy     (numpy.ndarray): [J/kg]
      mach_number             (numpy.ndarray): [-]
      static_temperature      (numpy.ndarray): [K]
      static_enthalpy         (numpy.ndarray): [J/kg]
      velocity                (numpy.ndarray): [m/s]
      static_pressure         (numpy.ndarray): [Pa]
      area_ratio              (numpy.ndarray): [-]
      denisty                 (numpy.ndarray): [kg/m^3] 

    Assumptions:
        1. Constant polytropic efficiency and pressure ratio
        2. If pressures make the Mach number go negative, these values are corrected

    Source:
        https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/

    Args:
        conditions.freestream.
          isentropic_expansion_factor         (numpy.ndarray): [-]
          specific_heat_at_constant_pressure  (numpy.ndarray): [J/(kg K)]
          pressure                            (numpy.ndarray): [Pa]
          stagnation_pressure                 (numpy.ndarray): [Pa]
          stagnation_temperature              (numpy.ndarray): [K]
          specific_gas_constant               (numpy.ndarray): [J/(kg K)] 
          mach_number                         (numpy.ndarray): [-]
        expansion_nozzle
          .inputs.stagnation_temperature       (numpy.ndarray): [K]
          .inputs.stagnation_pressure          (numpy.ndarray): [Pa] 
          .pressure_ratio                      (numpy.ndarray): [-]
          .polytropic_efficiency               (numpy.ndarray): [-]    

    Returns:
        None 

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
