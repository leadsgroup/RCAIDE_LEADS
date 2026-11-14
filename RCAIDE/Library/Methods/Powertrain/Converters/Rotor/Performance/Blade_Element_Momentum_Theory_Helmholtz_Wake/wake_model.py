# RCAIDE/Library/Framework/Analyses/Propulsion/Blade_Element_Momentum_Theory_Helmholtz_Wake/wake_model.py
#
# Created:  Jan 2022, R. Erhard
# Modified:  Jun 2024, M. Clarke  
#            Nov 2026, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core import Data, array_namespace
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift.BET_calculations import compute_airfoil_aerodynamics,compute_inflow_and_tip_loss
# Python imports
import numpy as np 
import warnings

# ----------------------------------------------------------------------------------------------------------------------
# wake model
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_wake(rotor,wake_inputs,conditions):
    """
    Evaluates the rotor wake using Helmholtz vortex theory.
    
    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor component with the following attributes:
            - number_of_blades : int
                Number of blades on the rotor
            - tip_radius : float
                Tip radius of the rotor [m]
            - hub_radius : float
                Hub radius of the rotor [m]
            - sol_tolerance : float
                Solution tolerance for wake convergence
    wake_inputs : Data
        Wake input parameters with:
            - ctrl_pts : int
                Number of control points
            - Nr : int
                Number of radial stations
            - Na : int
                Number of azimuthal stations
            - use_2d_analysis : bool
                Flag for 2D (azimuthal) analysis
            - velocity_total : array_like
                Total velocity magnitude [m/s]
            - velocity_axial : array_like
                Axial velocity component [m/s]
            - velocity_tangential : array_like
                Tangential velocity component [m/s]
            - twist_distribution : array_like
                Blade twist distribution [rad]
            - chord_distribution : array_like
                Blade chord distribution [m]
            - radius_distribution : array_like
                Radial station positions [m]
            - speed_of_sounds : array_like
                Speed of sound [m/s]
            - dynamic_viscosities : array_like
                Dynamic viscosity [kg/(m·s)]
    conditions : Data
        Flight conditions
    
    Returns
    -------
    va : array_like
        Axially-induced velocity from rotor wake [m/s]
    vt : array_like
        Tangentially-induced velocity from rotor wake [m/s]
    
    Notes
    -----
    This function evaluates the rotor wake using Helmholtz vortex theory to calculate
    the induced velocities. It solves for the inflow angle (PSI) that satisfies the
    circulation equation, then computes the axial and tangential induced velocities.
    
    The computation follows these steps:
        1. Initialize the inflow angle (PSI) array
        2. Solve for the inflow angle using a nonlinear equation solver
        3. Calculate the axial and tangential induced velocities from the converged solution
    
    **Major Assumptions**
        * The wake is modeled using Helmholtz vortex theory
        * The solution converges to a steady state
        * The inflow angle (PSI) is the primary variable being solved for
    
    **Theory**
    The wake model is based on Helmholtz vortex theory, which relates the circulation
    around the blade to the induced velocities in the wake. The key equation being solved is:
    
    .. math::
        R = \\Gamma - \\frac{1}{2}W \\cdot c \\cdot C_l = 0
    
    where:
        - Γ is the circulation
        - W is the relative velocity
        - c is the chord
        - Cl is the lift coefficient
    
    The circulation is related to the tangential induced velocity by:
    
    .. math::
        \\Gamma = v_t \\cdot \\frac{4\\pi r}{B} \\cdot F \\cdot \\sqrt{1 + \\left(\\frac{4\\lambda_w R}{\\pi B r}\\right)^2}
    
    where:
        - vt is the tangential induced velocity
        - r is the radial position
        - B is the number of blades
        - F is the tip loss factor
        - λw is the inflow ratio
    
    References
    ----------
    [1] Drela, M. "Qprop Formulation", MIT AeroAstro, June 2006 http://web.mit.edu/drela/Public/web/qprop/qprop_theory.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.BEMT_Helmholtz_performance
    """
    
    va, vt = wake_convergence(rotor, wake_inputs)
        
    return va, vt

# ---------------------------------------------------------------------------------------------------------------------- 
#  wake_convergence
# ---------------------------------------------------------------------------------------------------------------------- 
def wake_convergence(rotor,wake_inputs):
    """
    Wake evaluation is performed using a simplified vortex wake method for Fidelity Zero, 
    following Helmholtz vortex theory.
    
    Assumptions:
    None

    Source:
    Drela, M. "Qprop Formulation", MIT AeroAstro, June 2006
    http://web.mit.edu/drela/Public/web/qprop/qprop_theory.pdf

    Inputs:
       rotor        - rotor
       wake_inputs.
          Ua        - Axial velocity
          Ut        - Tangential velocity
          r         - radius distribution
       
    Outputs:
       va  - axially-induced velocity from rotor wake
       vt  - tangentially-induced velocity from rotor wake
    
    Properties Used:
    None
    
    """        
    # one inflow angle per station: (ctrl_pts, Nr) for the 1D analysis, (ctrl_pts, Nr, Na) for the 2D analysis
    if wake_inputs.use_2d_analysis:
        shape = (wake_inputs.ctrl_pts, wake_inputs.Nr, wake_inputs.Na)
    else:
        shape = (wake_inputs.ctrl_pts, wake_inputs.Nr)

    xp = array_namespace(*[wake_inputs[name] for name in WAKE_ARRAYS])
    if xp is np:
        PSI = stationwise_newton(np.ones(shape), wake_inputs, rotor)
    else:
        PSI = inflow_angle_jax(shape, wake_inputs, rotor)
    
    # Calculate the velocities given PSI
    va, vt = va_vt(PSI, wake_inputs, rotor)
    
    return va, vt

# the arrays of wake_inputs that can carry derivatives
WAKE_ARRAYS = ('velocity_total', 'velocity_axial', 'velocity_tangential', 'twist_distribution',
               'chord_distribution', 'radius_distribution', 'speed_of_sounds', 'dynamic_viscosities')

# the rotor values the residual and the station-wise solve read
ROTOR_VALUES = ('tip_radius', 'thickness_to_chord', 'number_of_blades', 'sol_tolerance')
POLAR_ARRAYS  = ('reynolds_numbers', 'angle_of_attacks', 'lift_coefficients', 'drag_coefficients')

def rotor_values(rotor, n_airfoils):
    """The rotor values followed by the polar tables of each airfoil, in a fixed order."""
    values = [rotor[name] for name in ROTOR_VALUES]
    for airfoil in list(rotor.airfoils)[:n_airfoils]:
        values += [airfoil.polars[name] for name in POLAR_ARRAYS]
    return values

def numpy_rotor(values, polar_stations, n_airfoils, tag):
    """A NumPy rotor holding what the residual and the station-wise solve read, rebuilt from rotor_values."""
    rotor = Data(tag=tag, airfoil_polar_stations=polar_stations, airfoils=[])
    for name, value in zip(ROTOR_VALUES, values):
        rotor[name] = np.asarray(value)[()]
    for k in range(n_airfoils):
        tables = values[len(ROTOR_VALUES) + k*len(POLAR_ARRAYS):len(ROTOR_VALUES) + (k + 1)*len(POLAR_ARRAYS)]
        rotor.airfoils.append(Data(polars=Data(zip(POLAR_ARRAYS, [np.asarray(table) for table in tables]))))
    return rotor

def inflow_angle_jax(shape, wake_inputs, rotor):
    """
    Inflow angles on JAX, with derivatives by the implicit function theorem.

    The NumPy station-wise solve finds the angles outside the compiled graph (jax.pure_callback), so values equal
    the NumPy path's. jax.lax.custom_root then differentiates residual(PSI) = 0 instead of the solver iterations.
    The residual of a station depends only on its own angle, so the Jacobian is diagonal and the tangent solve is
    one division per station.
    """
    import jax
    import jax.numpy as jnp

    # everything numeric the NumPy solve reads travels as a callback argument, so nothing of the rotor or the wake
    # inputs is frozen at the values of the design the function was compiled with
    static_inputs  = {name: wake_inputs[name] for name in ('ctrl_pts', 'Nr', 'Na', 'use_2d_analysis')}
    polar_stations = list(rotor.airfoil_polar_stations)
    n_airfoils     = len(rotor.airfoils) if len(polar_stations) != 0 else 0
    tag            = rotor.tag

    def solve_numpy(*values):
        numpy_inputs = Data(static_inputs)
        for name, value in zip(WAKE_ARRAYS, values):
            numpy_inputs[name] = np.asarray(value)
        return stationwise_newton(np.ones(shape), numpy_inputs, numpy_rotor(values[len(WAKE_ARRAYS):], polar_stations, n_airfoils, tag))

    def solve(f, guess):
        arrays = [jnp.asarray(wake_inputs[name], dtype=jnp.float64) for name in WAKE_ARRAYS]
        arrays = arrays + [jnp.asarray(value, dtype=jnp.float64) for value in rotor_values(rotor, n_airfoils)]
        return jax.pure_callback(solve_numpy, jax.ShapeDtypeStruct(shape, jnp.float64), *arrays, vmap_method='sequential')

    def tangent_solve(g, y):
        # g is the linearized residual; with a diagonal Jacobian, g(ones) is that diagonal
        return y / g(jnp.ones_like(y))

    return jax.lax.custom_root(lambda PSI: residual(PSI, wake_inputs, rotor), jnp.ones(shape), solve, tangent_solve)

def stationwise_newton(PSI, wake_inputs, rotor, max_iterations=50, max_step=0.2, n_scan=61):
    """
    Solves the BEVW residual for the inflow angle at every station with a vectorized Newton iteration,
    falling back to a bracketed bisection at stations where Newton does not converge.

    Parameters
    ----------
    PSI : numpy.ndarray
        Initial inflow angles, one per station, shape (ctrl_pts, Nr) or (ctrl_pts, Nr, Na) [rad]
    wake_inputs : Data
        Wake inputs used by residual()
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor with sol_tolerance
    max_iterations : int
        Maximum number of Newton iterations
    max_step : float
        Largest inflow angle change per Newton iteration [rad]
    n_scan : int
        Number of inflow angles scanned to bracket a root at stations where Newton fails

    Returns
    -------
    PSI : numpy.ndarray
        Converged inflow angles, same shape as the input [rad]

    Notes
    -----
    The residual of a station depends only on its own inflow angle, so the Jacobian is diagonal and each
    derivative is a forward difference of the full residual array. Solving the stations together as one system
    lets a few stiff stations stall all of them. Where the residual is not monotonic Newton can cycle or reach a
    root outside the scanned interval; those stations take the sign change closest to the initial guess and bisect
    it (bracketing as in Ning, "A simple solution method for the blade element momentum equations with guaranteed
    convergence", Wind Energy 17(9), 2014). Assumption: the physical root lies in [-pi/2, pi] and is the one closest
    to the initial guess. A sign change where bisection does not reduce the residual is a jump, not a root, and the
    next closest one is tried. A warning reports stations left unconverged.
    """
    tol        = rotor.sol_tolerance
    h          = 1e-7
    PSI_lower  = -np.pi/2
    PSI_upper  = np.pi
    PSI_0      = PSI.astype(float).copy()
    PSI        = PSI_0.copy()
    active     = np.ones(PSI.shape, dtype=bool)
    for i in range(max_iterations):
        R      = residual(PSI, wake_inputs, rotor)
        dR     = (residual(PSI + h, wake_inputs, rotor) - R)/h
        dR[dR == 0.] = h
        step   = np.clip(R/dR, -max_step, max_step)*active
        PSI   -= step
        active = np.abs(step) > tol*(np.abs(PSI) + tol)
        if not np.any(active):
            break

    # stations still moving, or converged outside the scanned interval, are bracketed and bisected
    active = active | (PSI < PSI_lower) | (PSI > PSI_upper)
    if not np.any(active):
        return PSI

    # bracket the root closest to the initial guess at those stations
    scan   = np.linspace(PSI_lower, PSI_upper, n_scan)
    R_scan = np.array([residual(np.where(active, angle, PSI), wake_inputs, rotor) for angle in scan])
    change = np.sign(R_scan[:-1]) != np.sign(R_scan[1:])
    middle = (0.5*(scan[:-1] + scan[1:])).reshape((-1,) + (1,)*PSI.ndim)   # one scan interval per row, any station shape
    dist   = np.where(change, np.abs(middle - PSI_0[None]), np.inf)
    rows   = np.arange(n_scan - 1).reshape((-1,) + (1,)*PSI.ndim)
    found  = np.zeros(PSI.shape, dtype=bool)
    jump   = None

    # a sign change can also be a jump of the residual (in reverse flow, the angle of attack jumps by 2 pi where the
    # axial velocity changes sign); bisection then ends with a large residual, and the next closest sign change is tried
    for attempt in range(n_scan - 1):
        bracketed = active & ~found & np.isfinite(np.min(dist, axis=0))
        if not np.any(bracketed):
            break
        k        = np.argmin(dist, axis=0)
        lo       = np.where(bracketed, scan[k], PSI)
        hi       = np.where(bracketed, scan[k + 1], PSI)
        R_ends   = np.maximum(np.abs(np.take_along_axis(R_scan, k[None], 0)[0]), np.abs(np.take_along_axis(R_scan, k[None] + 1, 0)[0]))
        PSI_b    = bisection(lo, hi, bracketed, PSI, wake_inputs, rotor, tol)
        R_b      = residual(PSI_b, wake_inputs, rotor)
        root     = bracketed & (np.abs(R_b) <= 1e-3*R_ends)
        PSI      = np.where(root, PSI_b, PSI)
        found    = found | root
        if jump is None:
            jump = np.where(bracketed & ~root, PSI_b, np.nan)   # the jump, kept where no root is found
        dist     = np.where((rows == k[None]) & (bracketed & ~root)[None], np.inf, dist)

    unconverged = active & ~found
    if np.any(unconverged):
        if jump is not None:
            PSI = np.where(unconverged & np.isfinite(jump), jump, PSI)
        warnings.warn(f"{rotor.tag}: BEMT inflow angle not converged at {np.sum(unconverged)} of {PSI.size} stations "
                      f"(no root of the residual in [-pi/2, pi])", RuntimeWarning)
    return PSI

def bisection(lo, hi, bracketed, PSI, wake_inputs, rotor, tol):
    """Bisects the residual on [lo, hi] at the bracketed stations; the other stations keep PSI."""
    R_lo = residual(lo, wake_inputs, rotor)
    while np.any(bracketed & (hi - lo > tol*(np.abs(lo) + tol))):
        mid   = 0.5*(lo + hi)
        R_mid = residual(np.where(bracketed, mid, PSI), wake_inputs, rotor)
        same  = np.sign(R_mid) == np.sign(R_lo)
        lo    = np.where(bracketed & same, mid, lo)
        R_lo  = np.where(bracketed & same, R_mid, R_lo)
        hi    = np.where(bracketed & ~same, mid, hi)
    return np.where(bracketed, 0.5*(lo + hi), PSI)

def residual(PSI, wake_inputs, rotor):
    """
    BEVW residual on the blade circulation at every station, same shape as PSI.

    Inputs:
       PSI                        inflow angle, one per station                   [rad]
       wake_inputs                velocities, blade geometry and flow properties  [-]
       rotor                      rotor                                           [-]

    Outputs:
       Rsquiggly                  circulation residual                            [m^2/s]
    """    
    
    # Unpack inputs to rotor wake fidelity zero
    U               = wake_inputs.velocity_total
    Ua              = wake_inputs.velocity_axial
    Ut              = wake_inputs.velocity_tangential
    use_2d_analysis = wake_inputs.use_2d_analysis        
    beta            = wake_inputs.twist_distribution
    c               = wake_inputs.chord_distribution
    r               = wake_inputs.radius_distribution
    a               = wake_inputs.speed_of_sounds
    nu              = wake_inputs.dynamic_viscosities
    ctrl_pts        = wake_inputs.ctrl_pts
    Nr              = wake_inputs.Nr
    Na              = wake_inputs.Na

    # Unpack rotor data        
    R            = rotor.tip_radius
    B            = rotor.number_of_blades
    tc           = rotor.thickness_to_chord
    airfoils     = rotor.airfoils
    a_loc        = rotor.airfoil_polar_stations
    
    # compute velocities
    xp           = array_namespace(PSI, U, Ua, Ut)
    sin_psi      = xp.sin(PSI)
    cos_psi      = xp.cos(PSI)
    Wa           = 0.5*Ua + 0.5*U*sin_psi
    Wt           = 0.5*Ut + 0.5*U*cos_psi
    vt           = Ut - Wt

    # compute blade airfoil forces and properties
    Cl, Cdval, alpha, alpha_disc,Ma,W,Re,Re_disc = compute_airfoil_aerodynamics(beta,c,r,R,B,Wa,Wt,a,nu,airfoils,a_loc,ctrl_pts,Nr,Na,tc,use_2d_analysis)

    # compute inflow velocity and tip loss factor
    lamdaw, F, piece = compute_inflow_and_tip_loss(r,R,Wa,Wt,B)

    # compute Newton residual on circulation
    Gamma       = vt*(4.*np.pi*r/B)*F*(1.+(4.*lamdaw*R/(np.pi*B*r))*(4.*lamdaw*R/(np.pi*B*r)))**0.5
    Rsquiggly   = Gamma - 0.5*W*c*Cl
    
    return Rsquiggly
 
def va_vt(PSI, wake_inputs, rotor):
    """
    Induced velocities from the converged inflow angle.

    Inputs:
       PSI                        inflow angle, one per station                   [rad]
       wake_inputs                velocities                                      [-]
       rotor                      rotor                                           [-]

    Outputs:
       va                         axially induced velocity                        [m/s]
       vt                         tangentially induced velocity                   [m/s]
    """    
    
    # Unpack inputs to rotor wake fidelity zero
    U               = wake_inputs.velocity_total
    Ua              = wake_inputs.velocity_axial
    Ut              = wake_inputs.velocity_tangential
    
    # compute velocities
    xp           = array_namespace(PSI, U, Ua, Ut)
    sin_psi      = xp.sin(PSI)
    cos_psi      = xp.cos(PSI)
    Wa           = 0.5*Ua + 0.5*U*sin_psi
    Wt           = 0.5*Ut + 0.5*U*cos_psi
    va           = Wa - Ua
    vt           = Ut - Wt

    return va, vt
