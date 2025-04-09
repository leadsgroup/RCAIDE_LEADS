# RCAIDE/Methods/Performance/estimate_stall_speed.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
 
# Pacakge imports 
import numpy as np  

#------------------------------------------------------------------------------
# Stall Speed Estimation
#------------------------------------------------------------------------------ 
def estimate_stall_speed(vehicle_mass,reference_area,altitude,maximum_lift_coefficient): 
    """
    Calculates the stall speed of an aircraft at a given altitude and maximum lift coefficient.

    Parameters
    ----------
    vehicle_mass : float
        Total mass of the vehicle [kg]
    reference_area : float
        Wing reference area [m²]
    altitude : float
        Flight altitude [m]
    maximum_lift_coefficient : float
        Maximum lift coefficient of the aircraft [unitless]

    Returns
    -------
    V_stall : float
        Stall speed [m/s]

    Notes
    -----
    The stall speed is calculated using the standard lift equation solved for velocity:
    
    .. math::
        V_{stall} = \sqrt{\\frac{2W}{\\rho S C_{L_{max}}}}

    where:
        * W = mg (vehicle weight)
        * ρ = air density at altitude
        * S = reference area
        * CL_max = maximum lift coefficient

    **Major Assumptions**
        * Steady, level flight
        * Incompressible flow
        * Standard atmospheric conditions
        * No wind or atmospheric disturbances
        * Rigid aircraft structure

    See Also
    --------
    RCAIDE.Library.Attributes.Atmospheres
    RCAIDE.Library.Methods.Performance.estimate_take_off_field_length
    """
      
    g       = 9.81 
    atmo    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    rho     = atmo.compute_values(altitude,0.).density 
    V_stall = float(np.sqrt((2.*vehicle_mass*g)/(rho*reference_area*maximum_lift_coefficient)))  
    
    return V_stall