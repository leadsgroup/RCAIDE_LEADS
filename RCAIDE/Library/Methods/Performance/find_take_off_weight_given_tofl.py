# find_take_off_weight_given_tofl.py
#
# Created:  Sep 2014, C. Ilario, T. Orra 
# Modified: Jan 2016, E. Botero


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from RCAIDE.Library.Methods.Performance.estimate_take_off_field_length import estimate_take_off_field_length

import numpy as np

# ----------------------------------------------------------------------
#  Find Takeoff Weight Given TOFL
# ----------------------------------------------------------------------
def find_take_off_weight_given_tofl(vehicle,analyses,target_tofl,altitude = 0, delta_isa = 0,):
    """
    Estimates the maximum allowable takeoff weight for a given takeoff field length requirement.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - mass_properties.operating_empty : float
                Operating empty weight [kg]
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
    analyses : Analyses
        Container with atmosphere and aerodynamic analyses
    target_tofl : float or ndarray
        Target takeoff field length(s) [m]
    altitude : float, optional
        Airport altitude [m], default 0
    delta_isa : float, optional
        Temperature offset from ISA conditions [K], default 0

    Returns
    -------
    max_tow : ndarray
        Maximum allowable takeoff weight(s) for given field length(s) [kg]

    Notes
    -----
    Uses an interpolation approach by:
        1. Creating array of possible takeoff weights between OEW and 110% MTOW
        2. Computing TOFL for each weight
        3. Interpolating to find weight that gives target TOFL

    **Major Assumptions**
        * Linear interpolation between computed points is valid
        * Target TOFL is within achievable range
        * Meets assumptions from estimate_take_off_field_length

    **Theory**
    Takeoff field length varies approximately with W/T where:
        * W = aircraft weight
        * T = available thrust

    .. math::
        TOFL \propto \\frac{W}{T}

    See Also
    --------
    RCAIDE.Library.Methods.Performance.estimate_take_off_field_length
    """       

    #unpack
    tow_lower = vehicle.mass_properties.operating_empty
    tow_upper = 1.10 * vehicle.mass_properties.max_takeoff

    #saving initial reference takeoff weight
    tow_ref = vehicle.mass_properties.max_takeoff

    tow_vec = np.linspace(tow_lower,tow_upper,50)
    tofl    = np.zeros_like(tow_vec)

    for id,tow in enumerate(tow_vec):
        vehicle.mass_properties.takeoff = tow
        tofl[id], _ = estimate_take_off_field_length(vehicle,analyses,altitude = 0, delta_isa = 0)

    target_tofl = np.atleast_1d(target_tofl)
    max_tow     = np.zeros_like(target_tofl)

    for id,toflid in enumerate(target_tofl):
        max_tow[id] = np.interp(toflid,tofl,tow_vec)

    #reset the initial takeoff weight
    vehicle.mass_properties.max_takeoff = tow_ref

    return max_tow