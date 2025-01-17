 # compute_flap_lift.py
#
# Created:  Dec 2013, A. Varyar
# Modified: Feb 2014, T. Orra
#           Jan 2016, E. Botero         

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
from   RCAIDE import  * 
from   RCAIDE.Framework.Core import Units
import numpy  as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  compute_flap_lift
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_flap_lift(t_c,flap_type,flap_chord,flap_angle,sweep,wing_Sref,wing_affected_area):
    """Computes the increase of lift due to trailing edge flap deployment

    Assumptions:
    None

    Source:
    Unknown

    Inputs:
    t_c                 (wing thickness ratio)                 [Unitless]
    flap_type                                                  [string]
    flap_c_chord        (flap chord as fraction of wing chord) [Unitless]
    flap_angle          (flap deflection)                      [radians]
    sweep               (Wing sweep angle)                     [radians]
    wing_Sref           (Wing reference area)                  [m^2]
    wing_affected_area  (Wing area affected by flaps)          [m^2]

    Outputs:
    dcl_max_flaps       (Lift coefficient increase)            [Unitless]

    Properties Used:
    N/A
    """          

    #unpack
    tc_r  = t_c
    fc    = flap_chord * 100.
    fa    = flap_angle / Units.deg
    Swf   = wing_affected_area
    sweep = sweep

    # Basic increase in CL due to flap
    dmax_ref= -4E-05*tc_r**4 + 0.0014*tc_r**3 - 0.0093*tc_r**2 + 0.0436*tc_r + 0.9734

    # Corrections for flap type
    if flap_type == None:
        dmax_ref = 0.
    elif flap_type.upper() == 'single_slotted'.upper():
        dmax_ref = dmax_ref * 0.93
    elif flap_type.upper() == 'triple_slotted'.upper():
        dmax_ref = dmax_ref * 1.08

    # Chord correction
    Kc =  0.0395*fc    + 0.0057

    # Deflection correction
    Kd = -1.7857E-04*fa**2 + 2.9214E-02*fa - 1.4000E-02

    # Sweep correction
    Ksw = (1 - 0.08 * (np.cos(sweep))**2) * (np.cos(sweep)) ** 0.75

    # Applying corrections
    dmax_flaps = Kc * Kd * Ksw * dmax_ref

    # Final CL increment due to flap
    dcl_max_flaps = dmax_flaps  *  Swf / wing_Sref

    return dcl_max_flaps