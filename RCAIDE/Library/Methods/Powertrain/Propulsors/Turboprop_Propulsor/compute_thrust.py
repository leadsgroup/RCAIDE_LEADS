# RCAIDE/Methods/Energy/Propulsors/Turboprop_Propulsor/compute_thrust.py
# 
# 
# Created:  Sep 2024, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports  
from RCAIDE.Framework.Core      import Units 

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_thrust
# ----------------------------------------------------------------------------------------------------------------------
def compute_thrust(turboprop,turboprop_conditions,conditions):
    
    """Computes thrust and other properties as below.

    Assumptions:
    Perfect gas

    Source:
    Mattingly, Jack D.. “Elements of Gas Turbine Propulsion.” (1996).

    Inputs:
    conditions.freestream.gravity 
    conditions.freestream.temperature
    conditions.freestream.pressure      
    conditions.freestream.mach_number
    conditions.freestream.velocity
    conditions.freestream.speed_of_sound
    turboprop.reference_temperature
    turboprop.reference_pressure 
    turboprop.combustor.turbine_inlet_temperature
    turboprop.design_propeller_efficiency
    turboprop.design_gearbox_efficiency
    turboprop.low_pressure_turbine.mechanical_efficiency
    turboprop.combustor.fuel_data.lower_heating_value
    turboprop_conditions.stag_temp_hpt_out
    turboprop_conditions.stag_temp_hpt_in
    turboprop_conditions.stag_temp_lpt_out
    turboprop_conditions.stag_temp_lpt_in
    turboprop_conditions.total_temperature_reference
    turboprop_conditions.total_pressure_reference     
    turboprop_conditions.fuel_to_air_ratio 
    turboprop_conditions.cpt
    turboprop_conditions.cpc
    turboprop_conditions.R_t
    turboprop_conditions.R_c
    turboprop_conditions.T9
    turboprop_conditions.P9  
    turboprop_conditions.gamma_c
    turboprop_conditions.core_exit_velocity
    
    Outputs:
    turboprop_conditions.thrust                           
    turboprop_conditions.thrust_specific_fuel_consumption 
    turboprop_conditions.non_dimensional_thrust           
    turboprop_conditions.core_mass_flow_rate              
    turboprop_conditions.fuel_flow_rate                   
    turboprop_conditions.power                            
    turboprop_conditions.specific_power                   
    turboprop_conditions.power_specific_fuel_consumption  
    turboprop_conditions.thermal_efficiency               
    turboprop_conditions.propulsive_efficiency               
    """       
    
    #compute dimensional mass flow rates 
    g                                              = conditions.freestream.gravity                                                                             # [m/s**2]
    Tref                                           = turboprop.reference_temperature                                                                           # [K]
    Pref                                           = turboprop.reference_pressure                                                                              # [Pa]
    total_temperature_reference                    = turboprop_conditions.total_temperature_reference                                                          # [K]
    total_pressure_reference                       = turboprop_conditions.total_pressure_reference                                                             # [Pa]
                                                                                                                                                                          
    #unpack from turboprop                                                                                                                                     
    f                                              = turboprop_conditions.fuel_to_air_ratio                                                                    # [-]
    cp_t                                           = turboprop_conditions.cpt                                                                                  # [J/kg*K]
    cp_c                                           = turboprop_conditions.cpc                                                                                  # [J/kg*K]   
    R_t                                            = turboprop_conditions.R_t                                                                                  # [J/mol*K]    
    R_c                                            = turboprop_conditions.R_c                                                                                  # [J/mol*K]
    T9                                             = turboprop_conditions.T9                                                                                   # [K]
    P9                                             = turboprop_conditions.P9                                                                                   # [Pa]     
    gamma_c                                        = turboprop_conditions.gamma_c                                                                              # [-]
    V9                                             = turboprop_conditions.core_exit_velocity                                                                   # [m/s]
    T0                                             = conditions.freestream.temperature                                                                         # [K]
    P0                                             = conditions.freestream.pressure                                                                            # [Pa]   
    M0                                             = conditions.freestream.mach_number                                                                         # [-]    
    V0                                             = conditions.freestream.velocity                                                                            # [m/s]
    a0                                             = conditions.freestream.speed_of_sound                                                                      # [m/s]
    Tt4                                            = turboprop.combustor.turbine_inlet_temperature                                                             # [K]     
    eta_prop                                       = turboprop.design_propeller_efficiency                                                                     # [-]
    eta_g                                          = turboprop.design_gearbox_efficiency                                                                       # [-]
    eta_mL                                         = turboprop.low_pressure_turbine.mechanical_efficiency                                                      # [-]
    h_PR                                           = turboprop.combustor.fuel_data.lower_heating_value                                                         # [J/kg]
    tau_tH                                         = (turboprop_conditions.stag_temp_hpt_out/turboprop_conditions.stag_temp_hpt_in)                            # [-]
    tau_tL                                         = (turboprop_conditions.stag_temp_lpt_out/turboprop_conditions.stag_temp_lpt_in)                            # [-]

    C_prop                                         = eta_prop*eta_g*eta_mL*(1 + f)*(cp_t*Tt4)/(cp_c*T0)*tau_tH*(1 - tau_tL)                                    # [-]
    Cc                                             = (gamma_c - 1)*M0*((1 + f)*(V9/a0) - M0 + (1 + f)*(R_t/R_c)*((T9/T0)/((V9/a0)))*((1 - (P0/P9))/gamma_c))   # [-]
    C_tot                                          = Cc + C_prop                                                                                               # [-]
    Fsp                                            = (C_tot*cp_c*T0)/(V0)                                                                                      # [(N*s)/kg] 
    TSFC                                           = (f/(Fsp)) * Units.hour                                                                                    # [kg/(N*hr)] 
    W_dot_mdot0                                    = C_tot*cp_c*T0                                                                                             # [(W*s)/kg] 
    PSFC                                           = (f/(C_tot*cp_c*T0)) * Units.hour                                                                          # [kg/(W*hr)]
    eta_T                                          = C_tot/((f*h_PR)/(cp_c*T0))                                                                                # [-]
    eta_P                                          = C_tot/((C_prop/eta_prop) + ((gamma_c - 1)/2)*((1 + f)*((V9/a0))**2 - M0**2))                              # [-]   
    
    mdot_core                                      = turboprop.design_thrust*turboprop_conditions.throttle/(Fsp)                                               # [kg/s]
    mdhc                                           = mdot_core/ (np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref))                    # [kg/s]
    
    #computing the dimensional thrust
    FD2                                            = Fsp*mdot_core                                                                                             # [N]  

    #fuel flow rate
    a                                              = np.array([0.])                                                                                            # [-]     
    fuel_flow_rate                                 = np.fmax(FD2*TSFC/g,a)*1./Units.hour                                                                       # [kg/s]  

    #computing the power 
    power                                          = FD2*V0

    # pack outputs 
    turboprop_conditions.thrust                            = FD2 
    turboprop_conditions.thrust_specific_fuel_consumption  = TSFC
    turboprop_conditions.non_dimensional_thrust            = Fsp 
    turboprop_conditions.core_mass_flow_rate               = mdot_core
    turboprop_conditions.fuel_flow_rate                    = fuel_flow_rate    
    turboprop_conditions.power                             = power  
    turboprop_conditions.specific_power                    = W_dot_mdot0  
    turboprop_conditions.power_specific_fuel_consumption   = PSFC 
    turboprop_conditions.thermal_efficiency                = eta_T
    turboprop_conditions.propulsive_efficiency             = eta_P

    return 