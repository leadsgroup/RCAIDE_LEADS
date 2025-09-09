# RCAIDE/Library/Methods/Powertrain/Converters/Motor/compute_motor_performance.py

# 
# Created:  Sep. 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE
from RCAIDE.Framework.Core import  Units

# package imports 
import numpy as np    

def compute_pump_performance(pump, line, conditions):

    Q1 = n * ID.N_p * ID.A_p * s             # total pump flow
    A_line = np.pi * (ID.D/2.0)**2
    P1 = ID.P_T - 0.5 * ID.rho * ID.f_T1 * (ID.L_T1 / ID.D) * (1.0 / A_line)**2 * np.abs(Q1) * Q1

    # Pump outlet boundary (tank + load)
    P2 = ID.P_T + ID.DeltaP_load

   
def compute_power_consumed(pressure_differential, density, mass_flow_rate, efficiency):
        
    """
    Calculates the power consumed by the pump.

    Parameters
    ----------
    pressure_differential : float
        Pressure rise across the pump
        
    density : float
        Coolant density
        
    mass_flow_rate : float
        Mass flow rate through the pump
        
    efficiency : float
        Overall pump efficiency

    Returns
    -------
    float
        Power consumed by the pump

    Notes
    -----
    Uses the standard pump power equation:
    Power = (mass_flow_rate * pressure_differential) / (density * efficiency)
    """
    return mass_flow_rate * pressure_differential / (density * efficiency)