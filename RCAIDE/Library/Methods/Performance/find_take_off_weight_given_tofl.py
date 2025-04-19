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
    """Estimates the takeoff weight given a certain takeoff field length.

    Assumptions:
    assumptions per estimate_take_off_field_length()

    Source:
    N/A

    Inputs:
    vehicle.mass_properties.
      operating_empty         [kg]
      max_takeoff             [kg]
      analyses                [RCAIDE data structure]
      airport                 [RCAIDE data structure]
      target_tofl             [m]
      
    Outputs:
    max_tow                   [kg]

    Properties Used:
    N/A
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