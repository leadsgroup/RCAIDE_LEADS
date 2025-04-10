# estimate_landing_field_length.py
#
# Created:  Jun 2014, T. Orra, C. Ilario, Celso, 
# Modified: Apr 2015, M. Vegh 
#           Jan 2016, E. Botero 
#           Mar 2020, M. Clarke
#           Jul 2020, E. Botero 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import  RCAIDE
from   RCAIDE.Framework.Core import Data, Units
from   RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_max_lift_coeff import compute_max_lift_coeff

import numpy as np

# ----------------------------------------------------------------------
#  Compute field length required for landing
# ----------------------------------------------------------------------
def estimate_landing_field_length(vehicle,analyses, altitude=0, delta_isa=0):
    """
    Computes the landing field length required for a given vehicle configuration at specified airport conditions.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - mass_properties.landing : float
                Landing weight [kg]
            - reference_area : float
                Wing reference area [m^2]
            - maximum_lift_coefficient : float, optional
                Maximum lift coefficient if pre-computed
            - Vref_VS_ratio : float, optional
                Ratio of approach to stall speed, default 1.23
    analyses : Analyses
        Container with aerodynamic analyses for computing maximum lift coefficient
    altitude : float, optional
        Airport altitude [ft], default 0
    delta_isa : float, optional
        Temperature offset from ISA conditions [K], default 0

    Returns
    -------
    landing_field_length : float
      Required landing field length [m]

    Notes
    -----
    The landing distance is computed using a semi-empirical approach:

    .. math::
        LFL = k_1 + k_2 V_{ref}^2

    where:
      * k₁ = 250 (constant)
      * k₂ = 2.485/g for two-wheel trucks
      * Vref = 1.23 * Vstall (default)

    **Major Assumptions**
      * Two-wheel truck landing gear configuration
      * Sea level standard atmospheric conditions unless specified
      * Standard approach speed ratio (1.23 × stall speed)
      * No wind conditions

    **Theory**
    The stall speed is computed as:

    .. math::
        V_{stall} = \sqrt{\\frac{2W}{\\rho S C_{L_{max}}}}

    References
    ----------
    [1] Torenbeek, E. (2013). Advanced Aircraft Design: Conceptual Design, Analysis and Optimization of Subsonic Civil Airplanes. Equation 9.25.

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_max_lift_coeff
    """            
   
    # ==============================================
    # Unpack
    # ============================================== 
    altitude        = altitude * Units.ft
    delta_isa       = delta_isa
    weight          = vehicle.mass_properties.landing
    reference_area  = vehicle.reference_area
    try:
        Vref_VS_ratio = vehicle.Vref_VS_ratio
    except:
        Vref_VS_ratio = 1.23
        
    # ==============================================
    # Computing atmospheric conditions
    # ==============================================
    atmo            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_values     = atmo.compute_values(altitude,delta_isa)
    
    p                 = atmo_values.pressure
    T                 = atmo_values.temperature
    rho               = atmo_values.density
    a                 = atmo_values.speed_of_sound
    mu                = atmo_values.dynamic_viscosity
    sea_level_gravity = atmo.planet.sea_level_gravity
   
    # ==============================================
    # Determining vehicle maximum lift coefficient
    # ==============================================
    # Condition to CLmax calculation: 90KTAS @ airport
    state = Data()
    state.conditions =  RCAIDE.Framework.Mission.Common.Results() 
    state.conditions.freestream = Data()
    state.conditions.freestream.density           = rho
    state.conditions.freestream.velocity          = 90. * Units.knots
    state.conditions.freestream.dynamic_viscosity = mu
    
    settings = analyses.aerodynamics.settings

    maximum_lift_coefficient, induced_drag_high_lift = compute_max_lift_coeff(state,settings,vehicle)

    # ==============================================
    # Computing speeds (Vs, Vref)
    # ==============================================
    stall_speed  = (2 * weight * sea_level_gravity / (rho * reference_area * maximum_lift_coefficient)) ** 0.5
    Vref         = stall_speed * Vref_VS_ratio
    
    # ========================================================================================
    # Computing landing distance, according to Torenbeek equation
    #     Landing Field Length = k1 + k2 * Vref**2
    # ========================================================================================

    # Defining landing distance equation coefficients 
    landing_constants    = np.zeros(3)
    landing_constants[0] = 250.
    landing_constants[1] =   0.
    landing_constants[2] =  2.485  / sea_level_gravity  # Two-wheels truck : [ (1.56 / 0.40 + 1.07) / (2*sea_level_gravity) ]
    
    # Calculating landing field length
    landing_field_length = 0.
    for idx,constant in enumerate(landing_constants):
        landing_field_length += constant * Vref**idx
    
    # return
    return landing_field_length
