# RCAIDE/Library/Methods/Stability/Vortex_Lattice_Method/compute_neutral_point.py
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM   import VLM
from copy import deepcopy

# package imports
import numpy  as np

# Fuselage and nacelle moment factor K_f [per radian] against the wing root quarter-chord position in percent of
# body length, digitized from Gilruth and White, NACA Report 711 (1941), Figure 8
FUSELAGE_MOMENT_FACTOR_POSITION = np.array([15., 20., 25., 30., 35., 40., 45., 50., 55., 60.])
FUSELAGE_MOMENT_FACTOR          = np.array([0.176, 0.283, 0.456, 0.592, 0.768, 0.989, 1.260, 1.643, 2.018, 2.585])

# ----------------------------------------------------------------------------------------------------------------------
#  compute_neutral_point
# ----------------------------------------------------------------------------------------------------------------------
def compute_neutral_point(stability, vehicle):
    """
    Computes the neutral point of an aircraft from the Vortex Lattice Method for the wings and tails, with the
    destabilizing moment of each fuselage from NACA Report 711.

    Parameters
    ----------
    stability : RCAIDE.Framework.Analyses.Stability
        Stability analysis object containing:
            - settings : RCAIDE.Framework.Analyses.Stability.Settings
                Stability analysis settings
            - training : Data
                Training data for analysis
                - angle_of_attack : numpy.ndarray
                    Array of angle of attack values [radians]
                - Mach : numpy.ndarray
                    Array of Mach numbers [unitless]
    vehicle : RCAIDE.Vehicle
        Vehicle with geometry and mass properties

    Returns
    -------
    None
        The neutral point is stored in vehicle.neutral_point [m]

    Notes
    -----
    With settings.fuselage_correction (default True) the wings and tails are panelled without the fuselage, because
    fuselage panels close to the tail panels move the neutral point by tens of percent MAC for small tail changes.
    With it False the fuselage panels are kept (settings.model_fuselage) and no fuselage moment is added.
    dCM/dalpha is linear in the moment reference location, so the wing-and-tail neutral point is the root of that
    line, found from two VLM evaluations. Each fuselage then moves the neutral point forward by its destabilizing
    moment K_f w_f^2 L_f (NACA Report 711, Eq. 1) divided by S_ref CL_alpha.

    **Major Assumptions**
        * Linear aerodynamics within the analysis range
        * Control surfaces are removed for clean configuration analysis
        * K_f is held at the end values of Figure 8 outside 15 to 60% body length
        * Pylon-mounted engine nacelles are not included: the nacelle term of Report 711 is for nacelles the wing
          passes through

    **Theory**

    .. math::
        x_{np} = x_{np,wt} - \\sum \\frac{K_f w_f^2 L_f}{S_{ref} C_{L_\\alpha}}

    References
    ----------
    [1] Gilruth, R. R., and White, M. D., "Analysis and Prediction of Longitudinal Stability of Airplanes,"
        NACA Report No. 711, 1941, Eq. (1) and Figure 8.
    """
    settings       = deepcopy(stability.settings)
    if settings.fuselage_correction:
        settings.model_fuselage = False
    AoA            = np.array([stability.training.angle_of_attack[2],stability.training.angle_of_attack[3]])
    Mach           = np.array([stability.training.Mach[0]])

    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.mach_number               = np.atleast_2d(np.repeat(Mach,len(AoA))).T
    conditions.freestream.velocity                  = np.ones_like(conditions.freestream.mach_number) * 1e-6
    conditions.aerodynamics.angles.alpha            = np.atleast_2d(AoA).T

    clean_wing_vehicle_np = deepcopy(vehicle)
    for wing in clean_wing_vehicle_np.wings:
        wing.control_surfaces = []

    # wing-and-tail neutral point: root of dCM/dalpha, which is linear in the moment reference location
    x_cg = vehicle.mass_properties.center_of_gravity[0][0]
    x_1, x_2 = x_cg - 1.0, x_cg + 1.0
    slope_1, CL_alpha = pitching_moment_slope(x_1, conditions, settings, clean_wing_vehicle_np, AoA)
    slope_2, _        = pitching_moment_slope(x_2, conditions, settings, clean_wing_vehicle_np, AoA)
    neutral_point     = x_1 - slope_1 * (x_2 - x_1) / (slope_2 - slope_1)

    # fuselage destabilizing moments [1], unless the fuselage panels are kept in the VLM
    fuselages = vehicle.fuselages if settings.fuselage_correction else []
    if len(fuselages) > 0:
        # main wing: the largest Main_Wing, else the largest wing
        main_wings = [wing for wing in vehicle.wings if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing)]
        main_wing  = max(main_wings if len(main_wings) > 0 else list(vehicle.wings), key=lambda wing: wing.areas.reference)
        x_wing_c4  = main_wing.origin[0][0] + 0.25 * main_wing.chords.root
    for fuselage in fuselages:
        L_f      = fuselage.lengths.total
        w_f      = fuselage.width
        position = 100. * (x_wing_c4 - fuselage.origin[0][0]) / L_f
        K_f      = np.interp(position, FUSELAGE_MOMENT_FACTOR_POSITION, FUSELAGE_MOMENT_FACTOR)
        neutral_point -= K_f * w_f**2 * L_f / (vehicle.reference_area * CL_alpha)

    vehicle.neutral_point = neutral_point
    return


def pitching_moment_slope(x_reference, conditions, settings, vehicle, AoA):
    """dCM/dalpha about x_reference, and dCL/dalpha, from two VLM angles of attack."""
    vehicle.mass_properties.center_of_gravity[0][0] = x_reference
    VLM_results = VLM(conditions,settings,vehicle)
    CM          = np.ravel(VLM_results.CM)
    CL          = np.ravel(VLM_results.CLift)
    return (CM[1] - CM[0]) / (AoA[1] - AoA[0]), (CL[1] - CL[0]) / (AoA[1] - AoA[0])
