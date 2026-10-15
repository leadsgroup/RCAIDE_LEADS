# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/compute_flap_profile_drag.py
#
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core import Units

# Python package imports
import numpy as np
import warnings

# ----------------------------------------------------------------------------------------------------------------------
#  Roskam two-dimensional flap profile drag increments
# ----------------------------------------------------------------------------------------------------------------------
# Roskam, Airplane Design Part VI (DARcorporation, 2004 printing), Figs. 4.46-4.48, valid for zero quarter-chord
# sweep. Digitized from the scanned figures by gridline-calibrated pixel tracking of each curve and local linear
# regression at the listed deflections (fit residual below 0.002 in delta c_dp). Each entry is
# flap chord ratio : (deflections [deg], delta c_dp [-]).
FLAP_PROFILE_DRAG_DATA = {
    'single_slotted': {  # Fig. 4.46
        0.10: ([11.5, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 58.5],
               [0.0017, 0.0031, 0.0051, 0.0070, 0.0095, 0.0123, 0.0157, 0.0199, 0.0247, 0.0299, 0.0338]),
        0.20: ([11.5, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 58.5],
               [0.0042, 0.0065, 0.0101, 0.0143, 0.0198, 0.0273, 0.0364, 0.0474, 0.0599, 0.0741, 0.0847]),
        0.30: ([11.5, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 58.5],
               [0.0077, 0.0115, 0.0181, 0.0270, 0.0383, 0.0515, 0.0663, 0.0823, 0.0998, 0.1185, 0.1331]),
    },
    'double_slotted': {  # Fig. 4.47
        0.20: ([11.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0],
               [0.0110, 0.0161, 0.0229, 0.0303, 0.0382, 0.0457, 0.0530, 0.0604, 0.0678, 0.0755]),
        0.30: ([11.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 52.5],
               [0.0190, 0.0282, 0.0404, 0.0528, 0.0652, 0.0776, 0.0901, 0.1024, 0.1150, 0.1212]),
        0.40: ([11.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0],
               [0.0308, 0.0454, 0.0655, 0.0857, 0.1070, 0.1272, 0.1475, 0.1678]),
    },
    'fowler': {  # Fig. 4.48
        0.10: ([12.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 47.5],
               [0.0021, 0.0025, 0.0037, 0.0055, 0.0084, 0.0126, 0.0183, 0.0254, 0.0305]),
        0.20: ([12.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 47.0],
               [0.0037, 0.0049, 0.0079, 0.0124, 0.0186, 0.0269, 0.0386, 0.0541, 0.0612]),
        0.30: ([12.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 47.0],
               [0.0063, 0.0091, 0.0151, 0.0236, 0.0346, 0.0486, 0.0644, 0.0814, 0.0882]),
        0.40: ([12.0, 15.0, 20.0, 25.0, 30.0, 35.0, 38.5],
               [0.0110, 0.0166, 0.0294, 0.0457, 0.0630, 0.0805, 0.0923]),
    },
}

# Roskam Part VI Figs. 4.44 (plain) and 4.45 (split) give the increment against flap chord ratio at the takeoff
# (15 deg) and landing (60 deg) deflections only. Plain: the two drawn straight lines, fit to the digitized curve
# with outlier rejection (within 3% of FAST-GA's independent digitization of the same figure). Split: the plotted
# points, per airfoil thickness ratio. Each entry is deflection [deg] : (flap chord ratios, delta c_dp).
PLAIN_FLAP_PROFILE_DRAG_DATA = {    # Fig. 4.44, drawn for c_f/c 0.07-0.29
    15.0: ([0.075, 0.10, 0.15, 0.20, 0.25, 0.30], [0.0064, 0.0086, 0.0130, 0.0173, 0.0217, 0.0261]),
    60.0: ([0.075, 0.10, 0.15, 0.20, 0.25, 0.30], [0.0433, 0.0624, 0.1006, 0.1388, 0.1771, 0.2153]),
}
SPLIT_FLAP_PROFILE_DRAG_DATA = {    # Fig. 4.45, thickness ratio : deflection [deg] : (flap chord ratios, delta c_dp)
    0.10: {15.0: ([0.10, 0.20, 0.30], [0.0135, 0.0276, 0.0431]), 60.0: ([0.10, 0.20, 0.30], [0.0797, 0.1737, 0.2804])},
    0.20: {15.0: ([0.10, 0.20, 0.30], [0.0101, 0.0196, 0.0297]), 60.0: ([0.10, 0.20, 0.30], [0.0701, 0.1530, 0.2461])},
    0.30: {15.0: ([0.10, 0.20, 0.30], [0.0059, 0.0112, 0.0194]), 60.0: ([0.10, 0.20, 0.30], [0.0570, 0.1263, 0.2053])},
}

# Interference drag factor K_int of Roskam Part VI Eq. 4.75
FLAP_INTERFERENCE_FACTOR = {'single_slotted': 0.40, 'double_slotted': 0.40, 'fowler': 0.25, 'plain': 0.0, 'split': -0.15}

# Flap types without a Roskam chart, mapped to the closest charted type
FLAP_TYPE_FALLBACK = {'triple_slotted': 'double_slotted'}

# ----------------------------------------------------------------------------------------------------------------------
#  compute_flap_profile_drag
# ----------------------------------------------------------------------------------------------------------------------
def compute_flap_profile_drag(wing, flap, reference_area):
    """
    Computes the zero-lift drag increment of a deflected trailing-edge flap, including its interference drag.

    Parameters
    ----------
    wing : RCAIDE.Library.Components.Wings.Wing
        Wing carrying the flap
            - sweeps.quarter_chord : float
                Quarter-chord sweep [rad]
            - thickness_to_chord : float
                Airfoil thickness ratio, used for split flaps [-]
            - areas.affected : float
                Flapped wing area, the wing area spanned by the flap on both sides [m^2]
            - areas.reference : float
                Wing reference area [m^2]
    flap : RCAIDE.Library.Components.Wings.Control_Surfaces.Flap
        Flap
            - configuration_type : str
                'plain', 'split', 'single_slotted', 'double_slotted', 'triple_slotted' or 'fowler'
            - chord_fraction : float
                Flap chord over wing chord [-]
            - deflection : float
                Flap deflection [rad]
    reference_area : float
        Vehicle reference area [m^2]

    Returns
    -------
    delta_CD : float
        Profile plus interference drag increment of the flap on the vehicle reference area [-]

    Notes
    -----
    Roskam, Airplane Design Part VI (DARcorporation, 2004 printing), Sec. 4.6:

    :math:`\\Delta C_{D} = (1 + K_{int}) \\Delta c_{d_p} \\cos\\Lambda_{c/4} S_{wf}/S`  (Eqs. 4.71 and 4.75)

    with the two-dimensional increment from Figs. 4.46-4.48 and K_int from Eq. 4.75. The flap's induced drag
    increment (Eq. 4.74) is captured by the vortex lattice surrogate. Between the charted flap chord ratios
    the increment is interpolated linearly, and below the smallest charted ratio toward zero at zero flap chord;
    below the first charted deflection it is interpolated toward zero at zero deflection. Beyond the charted
    ranges the nearest charted value is held. Plain and split flaps are charted at 15 and 60 deg only; between
    them the increment follows the quadratic in deflection through zero and the two charted values, the rule
    FAST-GA applies to the same Roskam figure (figure_digitization.delta_cd_plain_flap). Split flaps are
    interpolated in the wing thickness ratio. Triple slotted flaps use the double slotted chart, and an
    unrecognized flap type the single slotted chart; both raise a warning.
    """
    flap_type = flap.configuration_type
    if flap_type in FLAP_TYPE_FALLBACK:
        warnings.warn(f"No Roskam flap profile drag chart for '{flap_type}' flaps; using the {FLAP_TYPE_FALLBACK[flap_type]} chart.")
        flap_type = FLAP_TYPE_FALLBACK[flap_type]
    elif flap_type not in FLAP_INTERFERENCE_FACTOR:
        warnings.warn(f"Unrecognized flap type '{flap_type}'; using the single_slotted flap profile drag chart.")
        flap_type = 'single_slotted'

    delta_deg = np.abs(flap.deflection) / Units.degrees
    if flap_type == 'plain':
        cd_2D = takeoff_landing_increment(PLAIN_FLAP_PROFILE_DRAG_DATA, flap.chord_fraction, delta_deg)
    elif flap_type == 'split':
        ratios = sorted(SPLIT_FLAP_PROFILE_DRAG_DATA.keys())
        values = [takeoff_landing_increment(SPLIT_FLAP_PROFILE_DRAG_DATA[t_c], flap.chord_fraction, delta_deg) for t_c in ratios]
        cd_2D  = np.interp(wing.thickness_to_chord, ratios, values)
    else:
        curves = FLAP_PROFILE_DRAG_DATA[flap_type]
        ratios = np.array([0.0] + sorted(curves.keys()))
        values = [0.0]
        for ratio in sorted(curves.keys()):
            deflections, increments = curves[ratio]
            values.append(np.interp(delta_deg, [0.0] + deflections, [0.0] + increments))
        cd_2D = np.interp(flap.chord_fraction, ratios, values)

    sweep        = wing.sweeps.quarter_chord if wing.sweeps.quarter_chord is not None else 0.0
    # flapped wing area; the flap's spanwise share of the wing area when it is not set
    flapped_area = wing.areas.affected if wing.areas.affected > 0 else wing.areas.reference * (flap.span_fraction_end - flap.span_fraction_start)
    return (1 + FLAP_INTERFERENCE_FACTOR[flap_type]) * cd_2D * np.cos(sweep) * flapped_area / reference_area


def takeoff_landing_increment(chart, chord_fraction, delta_deg):
    """
    Increment from a chart given at 15 and 60 deg: each deflection is interpolated in flap chord ratio (toward zero
    at zero chord), then the quadratic in deflection through zero and the two values is evaluated, held at 60 deg.
    """
    at = {}
    for deflection, (ratios, increments) in chart.items():
        at[deflection] = np.interp(chord_fraction, [0.0] + ratios, [0.0] + increments)
    d    = min(delta_deg, 60.0)
    a, b = np.linalg.solve([[15.0, 15.0**2], [60.0, 60.0**2]], [at[15.0], at[60.0]])
    return max(0.0, a * d + b * d**2)
