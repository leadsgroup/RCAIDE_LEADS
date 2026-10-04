# RCAIDE/Library/Methods/Powertrain/Converters/EHD_Electrode_Array/compute_electrode_drag.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

NACA_0010_THICKNESS_RATIO = 0.10

# ----------------------------------------------------------------------------------------------------------------------
#  compute_collector_drag_coefficient
# ----------------------------------------------------------------------------------------------------------------------
def compute_collector_drag_coefficient(reynolds_number, boundary_layer='laminar'):
    """
    Default estimate of the NACA 0010 collector section drag coefficient (R24).

    Parameters
    ----------
    reynolds_number : float or numpy.ndarray
        Chord Reynolds number Re_c = V_inf·c/nu [-]
    boundary_layer : str, optional
        'laminar' (C_f = 1.328/sqrt(Re_c)) or 'turbulent' (C_f = 0.074/Re_c^0.2), default 'laminar'

    Returns
    -------
    c_d : numpy.ndarray
        Section drag coefficient [-], c_d = 2·C_f·FF with FF = 1 + 2(t/c) + 60(t/c)^4, t/c = 0.10.
        Zero where Re_c = 0 (no freestream, so no drag).

    Notes
    -----
    Unreliable at the low Reynolds numbers (~1e4) of slow EHD aircraft; a user-supplied collector
    drag coefficient is strongly recommended. The spec marks the form-factor constants [VERIFY]
    against Hoerner (1965).

    References
    ----------
    [1] Hoerner, S. F., Fluid-Dynamic Drag, 1965 (form factor).
    [2] Blasius / Prandtl flat-plate skin friction (standard).
    """
    Re = np.asarray(reynolds_number, dtype=float)
    if boundary_layer == 'laminar':
        C_f = np.divide(1.328, np.sqrt(Re), out=np.zeros_like(Re), where=Re > 0)
    elif boundary_layer == 'turbulent':
        C_f = np.divide(0.074, Re**0.2, out=np.zeros_like(Re), where=Re > 0)
    else:
        raise ValueError("collector_boundary_layer must be 'laminar' or 'turbulent', got " + repr(boundary_layer))
    t_c = NACA_0010_THICKNESS_RATIO
    FF  = 1.0 + 2.0 * t_c + 60.0 * t_c**4
    return 2.0 * C_f * FF

# ----------------------------------------------------------------------------------------------------------------------
#  compute_electrode_drag
# ----------------------------------------------------------------------------------------------------------------------
def compute_electrode_drag(density, velocity, collector_chord, emitter_diameter, span,
                           collector_drag_coefficient, emitter_drag_coefficient):
    """
    Computes the collector and emitter-wire drag of one unit in flight (spec 3.3 step 7, R24).

    Parameters
    ----------
    density : float or numpy.ndarray
        Freestream density rho [kg/m^3]
    velocity : float or numpy.ndarray
        Freestream speed V_inf [m/s]
    collector_chord : float
        c [m]
    emitter_diameter : float
        D_w [m]
    span : float
        b [m]
    collector_drag_coefficient : float or numpy.ndarray
        c_d [-] (user value or R24)
    emitter_drag_coefficient : float
        C_D,w [-]

    Returns
    -------
    D_c : float or numpy.ndarray
        Collector drag per unit [N], 1/2·rho·V_inf^2·c·b·c_d
    D_w : float or numpy.ndarray
        Wire drag per unit [N], 1/2·rho·V_inf^2·D_w·b·C_D,w

    Notes
    -----
    Collector and wire drag live inside the propulsor. The collectors must not also be modelled as
    wings or fuselage parts, or the drag is counted twice (spec 3.3, drag bookkeeping rule).
    """
    q   = 0.5 * density * velocity**2
    D_c = q * collector_chord * span * collector_drag_coefficient
    D_w = q * emitter_diameter * span * emitter_drag_coefficient
    return D_c, D_w
