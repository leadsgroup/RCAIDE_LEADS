# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/compute_rotor_wake_induced_velocity.py
# 
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.compute_wake_induced_velocity import compute_wake_induced_velocity

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  compute_rotor_wake_induced_velocity
# ----------------------------------------------------------------------------------------------------------------------  
def compute_rotor_wake_induced_velocity(rotor, conditions, points):
    """
    Computes the velocity induced by a rotor wake at points in the vehicle frame, using the wake model of the
    rotor's fidelity. Fidelities without a wake model induce no velocity.

    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor whose performance has been computed in conditions.energy.converters[rotor.tag]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions
    points : Data
        Evaluation points with XC, YC, ZC of shape (ctrl_pts, n_points) [m]

    Returns
    -------
    V_ind : numpy.ndarray
        Induced velocity at the points, shape (ctrl_pts, n_points, 3) [m/s]
    """
    ctrl_pts = len(points.XC)
    if rotor.fidelity == 'Blade_Element_Momentum_Theory_Helmholtz_Wake':
        V_ind = compute_wake_induced_velocity(rotor,conditions.energy.converters[rotor.tag],points,ctrl_pts)
    else:
        V_ind = np.zeros((ctrl_pts,len(points.XC[0]),3))
    return V_ind
