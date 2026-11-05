# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/compute_longitudinal_derivatives.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_longitudinal_derivatives
# ----------------------------------------------------------------------------------------------------------------------
def compute_longitudinal_derivatives(state, settings, vehicle):
    """
    Stability-axis force derivatives at the flight condition, from the aerodynamic slopes the VLM evaluation stored
    (Clift_alpha, Cdrag_alpha, Clift_u, Cdrag_u) and the drag and thrust of this evaluation.

    Parameters
    ----------
    state : Segment or State
        conditions and analyses.aerodynamics (for user overrides in stability_derivatives)
    settings : Data
        viscous_lift_dependent_drag_factor
    vehicle : Vehicle

    Returns
    -------
    None
        Writes CX_alpha, CZ_alpha, CX_u, CZ_u in conditions.static_stability.derivatives

    Notes
    -----
    Neglects aeroelastic effects and the dependence of lift, drag and moment on thrust. Cdrag_alpha and Cdrag_u hold
    inviscid induced drag, so wave and form drag slopes are not included.

    References
    ----------
    [1] Etkin, B., and Reid, L. D., Dynamics of Flight: Stability and Control, 3rd ed., Wiley, 1996:
        (5.2,1) and (5.2,3) alpha derivatives; (5.3,8), (5.3,9), (5.3,11), (5.3,12) u derivatives.
    """
    conditions  = state.conditions
    derivatives = conditions.static_stability.derivatives
    coefficients = conditions.aerodynamics.coefficients
    CL_0        = coefficients.lift.total
    CD_0        = coefficients.drag.total
    K           = settings.viscous_lift_dependent_drag_factor

    # viscous lift-dependent drag K CD_p CL^2, as in induced_drag
    Cdrag_alpha = derivatives.Cdrag_alpha + 2 * K * coefficients.drag.parasite.total * CL_0 * derivatives.Clift_alpha

    values = {'CX_alpha': CL_0 - Cdrag_alpha,
              'CZ_alpha': -(derivatives.Clift_alpha + CD_0),
              'CX_u'    : -derivatives.Cdrag_u + thrust_coefficient_derivative(conditions, vehicle),
              'CZ_u'    : -derivatives.Clift_u}
    overrides = state.analyses.aerodynamics.stability_derivatives
    for name, value in values.items():
        override = getattr(overrides, name)
        derivatives[name] = value if override is None else override * np.ones_like(CL_0)
    return


def thrust_coefficient_derivative(conditions, vehicle):
    """
    C_T_u of the vehicle: -2 C_T for jets (constant thrust) and -3 C_T for propeller-driven propulsors (constant
    power), by each propulsor's thrust; zero with no thrust. Etkin & Reid (5.3,7) to (5.3,9).
    """
    Propulsors = RCAIDE.Library.Components.Powertrain.Propulsors
    jets       = (Propulsors.Turbofan, Propulsors.Turbojet)
    q_S        = conditions.freestream.dynamic_pressure * vehicle.reference_area
    C_T_u      = np.zeros_like(q_S)
    for network in vehicle.networks:
        for propulsor in network.propulsors:
            if propulsor.tag not in conditions.energy.propulsors:
                continue
            C_T    = np.divide(conditions.energy.propulsors[propulsor.tag].outputs.thrust[:, 0:1], q_S,
                               out=np.zeros_like(q_S), where=q_S > 0)
            C_T_u  = C_T_u - (2 if isinstance(propulsor, jets) else 3) * C_T
    return C_T_u
