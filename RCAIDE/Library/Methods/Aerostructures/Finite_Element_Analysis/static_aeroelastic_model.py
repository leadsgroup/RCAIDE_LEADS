# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/static_aeroelastic_model.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                                                      import Data
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM                              import VLM
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_vortex_distribution     import generate_vortex_distribution
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.FEA                          import assemble_beam, panel_forces, point_loads_to_element_loads, self_weight_load, nodal_force_vector, solve_beam, recover_stresses
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing              import discretize_wing, deform_vortex_distribution
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.assign_component_masses      import assign_component_masses

import numpy as np

STANDARD_GRAVITY = 9.81   # [m/s^2] used by self_weight_load

# Tip deflection over semi-span up to which the model's tip deflection, tip twist and lift coefficient are within
# about 1% of the coupled iteration that rebuilds the mesh. Measured on a straight wing and on the uCRM wing
# (RCAIDE_DEV_NOTES/aerostructural_surrogate_physics_and_numerics.md, section 14); twist is the first to degrade.
VALIDATED_TIP_DEFLECTION_RATIO = 0.10

# ----------------------------------------------------------------------
#  build_static_aeroelastic_model
# ----------------------------------------------------------------------
def build_static_aeroelastic_model(vehicle, aerodynamics_settings, structural_settings, mach_numbers,
                                   angles_of_attack=np.linspace(-8.0, 12.0, 6)*np.pi/180, number_of_stations=16,
                                   feedback_components=(0, 1, 2, 3, 4, 5), feedback_angles_of_attack=np.array([0.0, 8.0])*np.pi/180,
                                   rotation_step=0.5*np.pi/180, translation_step=0.05):
    """
    Static aeroelastic model of the vehicle's wings,

        (K - q Q(alpha)) u = q f(alpha) + P,

    the form of the static aeroelastic equations of motion in the MSC Nastran Aeroelastic Analysis User's Guide
    (2023.1, Eq. 2-109), with K the beam stiffness, Q the aerodynamic forces due to structural deformation, f the
    rigid-wing aerodynamic forces per unit dynamic pressure q, and P the inertial loads.

    Q is built from the vortex lattice method by displacing the wing at a set of spanwise stations, one degree of
    freedom at a time, and taking the change in the forces on the structure. Only feedback_components are fed back
    to the aerodynamics. Vertical wings are not fed back. The force on a panel is its pressure, which grows with
    angle of attack, along its rotated normal, so Q varies linearly with angle of attack. It is built at the two
    feedback_angles_of_attack and interpolated between them.

    The rigid-wing forces f are stored at each of mach_numbers and angles_of_attack. Dynamic pressure, load
    factor and fuel masses enter solve_static_aeroelastic_model directly.

    Args:
        vehicle               : vehicle, with its geometry already preprocessed
        aerodynamics_settings : vortex lattice settings
        structural_settings   : aerostructures settings (discretiation, spar_cutouts)
        mach_numbers          : 1D array of Mach numbers, increasing
        angles_of_attack      : 1D array of angles of attack [rad], increasing
        number_of_stations    : spanwise stations on each wing
        feedback_components   : nodal degrees of freedom fed back to the aerodynamics at each station
                                (0, 1, 2 translations in x, y, z; 3, 4, 5 rotations about x, y, z)

    Returns:
        model : Data used by solve_static_aeroelastic_model
    """
    mach_numbers              = np.atleast_1d(np.array(mach_numbers, dtype=float))
    angles_of_attack          = np.atleast_1d(np.array(angles_of_attack, dtype=float))
    feedback_angles_of_attack = np.atleast_1d(np.array(feedback_angles_of_attack, dtype=float))
    if np.any(np.diff(mach_numbers) <= 0) or np.any(np.diff(angles_of_attack) <= 0):
        raise ValueError('mach_numbers and angles_of_attack must be increasing')
    if len(feedback_angles_of_attack) != 2 or feedback_angles_of_attack[0] == feedback_angles_of_attack[1]:
        raise ValueError('feedback_angles_of_attack must be two different angles')
    n_mach           = len(mach_numbers)
    n_alpha          = len(angles_of_attack)

    # beam, component masses and inertial responses of each wing
    wings = Data()
    for wing in vehicle.wings.values():
        node_data = discretize_wing(wing, structural_settings.discretiation)
        beam      = assemble_beam(wing, node_data, structural_settings)
        masses    = assign_component_masses(vehicle, wing, node_data)

        w               = Data()
        w.wing          = wing
        w.node_data     = node_data
        w.beam          = beam
        w.station_nodes = np.zeros(0, dtype=int)
        w.u_fixed, w.u_fuel, w.load_fixed, w.load_fuel = inertial_responses(wing, node_data, beam, masses)
        wings[wing.tag] = w

    # stations: (wing tag, station, degree of freedom), and the shape of each station's unit displacement along its wing
    stations = []
    for wing in vehicle.wings.values():
        if wing.vertical:
            continue
        n_nodes = len(wings[wing.tag].node_data.Y_nodes)
        nodes   = np.round(np.linspace(0, n_nodes - 1, number_of_stations + 1)).astype(int)[1:]   # the root is clamped
        wings[wing.tag].station_nodes  = nodes
        wings[wing.tag].station_shapes = station_shapes(wings[wing.tag].node_data.y_local, nodes)
        for k in range(number_of_stations):
            for component in feedback_components:
                stations.append((wing.tag, k, component))
    n_stations = len(stations)

    # element loads (w_x, w_y, w_z, t_y) per unit dynamic pressure, and the beam's response to them
    for w in wings.values():
        n_elements   = len(w.node_data.Le)
        w.load_rigid = np.zeros((n_mach, n_alpha, 4, n_elements))          # rigid wing
        w.u_rigid    = np.zeros((n_mach, n_alpha, w.beam.total_dof))
        w.load_G     = np.zeros((n_mach, 2, 4, n_elements, n_stations))    # unit displacement of each station
        w.G          = np.zeros((n_mach, 2, w.beam.total_dof, n_stations))

    dCLift_deta = np.zeros((n_mach, 2, n_stations))
    dCM_deta    = np.zeros((n_mach, 2, n_stations))
    CLift_rigid = np.zeros((n_mach, n_alpha))
    CM_rigid    = np.zeros((n_mach, n_alpha))

    for m, mach in enumerate(mach_numbers):
        # rigid wing at each angle of attack
        jig_VD = generate_vortex_distribution(aerodynamic_conditions(mach, angles_of_attack[0]), aerodynamics_settings, vehicle)
        for a, alpha in enumerate(angles_of_attack):
            loads, results = aerodynamic_loads(mach, alpha, jig_VD, aerodynamics_settings, vehicle, wings)
            for tag, w in wings.items():
                w.load_rigid[m, a] = loads[tag]
                w.u_rigid[m, a]    = beam_response(w, loads[tag])
            CLift_rigid[m, a] = results.CLift[0, 0]
            CM_rigid[m, a]    = results.CM[0, 0]

        # unit displacement of each station, by central difference about the jig shape, at the two feedback angles of attack
        for r, alpha in enumerate(feedback_angles_of_attack):
            for k, (station_tag, station_k, component) in enumerate(stations):
                step                   = translation_step if component < 3 else rotation_step
                loads_plus,  results_plus  = aerodynamic_loads(mach, alpha, deformed_mesh(jig_VD, vehicle, wings, station_tag, station_k, component,  step), aerodynamics_settings, vehicle, wings)
                loads_minus, results_minus = aerodynamic_loads(mach, alpha, deformed_mesh(jig_VD, vehicle, wings, station_tag, station_k, component, -step), aerodynamics_settings, vehicle, wings)
                for tag, w in wings.items():
                    w.load_G[m, r, :, :, k] = (loads_plus[tag] - loads_minus[tag]) / (2 * step)
                    w.G[m, r, :, k]         = beam_response(w, w.load_G[m, r, :, :, k])
                dCLift_deta[m, r, k] = (results_plus.CLift[0, 0] - results_minus.CLift[0, 0]) / (2 * step)
                dCM_deta[m, r, k]    = (results_plus.CM[0, 0]    - results_minus.CM[0, 0])    / (2 * step)

    model             = Data()
    model.mach        = mach_numbers
    model.alpha       = angles_of_attack
    model.feedback_alpha = np.array(feedback_angles_of_attack, dtype=float)
    model.wings       = wings
    model.stations    = stations
    model.CLift_rigid = CLift_rigid
    model.CM_rigid    = CM_rigid
    model.dCLift_deta = dCLift_deta
    model.dCM_deta    = dCM_deta
    return model


# ----------------------------------------------------------------------
#  solve_static_aeroelastic_model
# ----------------------------------------------------------------------
def solve_static_aeroelastic_model(model, mach, dynamic_pressure, alpha, load_factor=1.0, gravity=STANDARD_GRAVITY, fuel_masses=None, coupled=True):
    """
    Deformation of the wings at one flight condition.

    The model is linear in the deformation, and it always returns the linear solution. Two measures say how far
    to trust it, and both are smooth, so an optimizer can constrain them:
      tip_deflection_ratio   : tip deflection over semi-span of each wing. Up to VALIDATED_TIP_DEFLECTION_RATIO
                               the model is within about 1% of the coupled iteration that rebuilds the mesh.
      divergence_determinant : det(I - q S G), one at zero dynamic pressure, zero at the divergence dynamic
                               pressure and negative just above it, where the solution has no physical meaning.
    within_validated_range combines the two.

    The one-way response u_0 = K^-1 [q f(alpha) + P] is exact for the beam. The aerodynamic feedback
    is added through the displacements eta at the stations: u = u_0 + q G eta, with eta = (I - q S G)^-1 S u_0
    and S the fed-back degrees of freedom at the station nodes.

    Args:
        model            : Data from build_static_aeroelastic_model
        mach             : Mach number
        dynamic_pressure : [Pa]
        alpha            : angle of attack [rad]
        load_factor      : inertial load factor
        gravity          : [m/s^2]
        fuel_masses      : dict of fuel mass [kg] by fuel tank tag; tanks left out are empty
        coupled          : False gives the one-way response

    Returns:
        results : Data by wing tag with deflection (nodes, 3), rotation (nodes, 3), elastic_twist (nodes,),
                  normal_stress, shear_stress, margin_of_safety (elements,) and tip_deflection_ratio, plus eta,
                  CLift, CM, divergence_determinant and within_validated_range
    """
    q           = dynamic_pressure
    fuel_masses = fuel_masses or {}

    # linear interpolation in Mach number (held at the ends) and in angle of attack (extended past the ends)
    m_lo, m_hi, m_w = bracket(model.mach, mach, hold_ends=True)
    a_lo, a_hi, a_w = bracket(model.alpha, alpha, hold_ends=False)
    def at_mach(table):
        return (1 - m_w) * table[m_lo] + m_w * table[m_hi]
    def at_mach_and_alpha(table):
        at_m = at_mach(table)
        return (1 - a_w) * at_m[a_lo] + a_w * at_m[a_hi]
    f_w = (alpha - model.feedback_alpha[0]) / (model.feedback_alpha[1] - model.feedback_alpha[0])
    def feedback_at_mach_and_alpha(table):
        at_m = at_mach(table)
        return (1 - f_w) * at_m[0] + f_w * at_m[1]

    # one-way response of each wing
    inertial_scale = load_factor * gravity / STANDARD_GRAVITY
    u_0    = {}
    load_0 = {}
    G      = {}
    for tag, w in model.wings.items():
        u_inertial    = w.u_fixed.copy()
        load_inertial = w.load_fixed.copy()
        for tank_tag in w.u_fuel:
            u_inertial    += fuel_masses.get(tank_tag, 0.0) * w.u_fuel[tank_tag]
            load_inertial += fuel_masses.get(tank_tag, 0.0) * w.load_fuel[tank_tag]
        u_0[tag]    = q * at_mach_and_alpha(w.u_rigid)    + inertial_scale * u_inertial
        load_0[tag] = q * at_mach_and_alpha(w.load_rigid) + inertial_scale * load_inertial
        G[tag]      = feedback_at_mach_and_alpha(w.G)

    # displacements at the stations, with the aerodynamic feedback
    n_stations = len(model.stations)
    eta_0      = np.zeros(n_stations)
    S_G        = np.zeros((n_stations, n_stations))
    for k, (tag, station_k, component) in enumerate(model.stations):
        dof        = 6 * model.wings[tag].station_nodes[station_k] + component
        eta_0[k]   = u_0[tag][dof]
        S_G[k, :]  = G[tag][dof, :]
    system = np.eye(n_stations) - q * S_G
    if coupled and n_stations > 0:
        eta = np.linalg.solve(system, eta_0)
    else:
        eta = np.zeros(n_stations)

    results     = Data()
    results.eta = eta if coupled else eta_0
    for tag, w in model.wings.items():
        u                             = u_0[tag] + q * G[tag] @ eta
        w_x, w_y, w_z, t_y            = load_0[tag] + q * feedback_at_mach_and_alpha(w.load_G) @ eta
        normal_stress, shear_stress, margin_of_safety = recover_stresses(w.wing, w.node_data, w.beam, u, w_x, w_y, w_z, t_y)

        results[tag]                  = Data()
        results[tag].deflection       = np.column_stack((u[0::6], u[1::6], u[2::6]))
        results[tag].rotation         = np.column_stack((u[3::6], u[4::6], u[5::6]))
        results[tag].elastic_twist    = u[3::6] * np.sin(w.node_data.sweep_nodes) + u[4::6] * np.cos(w.node_data.sweep_nodes)
        results[tag].normal_stress    = normal_stress
        results[tag].shear_stress     = shear_stress
        results[tag].margin_of_safety = margin_of_safety
        results[tag].tip_deflection_ratio = np.linalg.norm(results[tag].deflection[-1]) / w.node_data.total_span
    results.divergence_determinant = np.linalg.det(system) if coupled else 1.0
    largest_ratio                  = max(results[tag].tip_deflection_ratio for tag in model.wings.keys())
    results.within_validated_range = bool((results.divergence_determinant > 0) & (largest_ratio <= VALIDATED_TIP_DEFLECTION_RATIO))
    results.CLift = at_mach_and_alpha(model.CLift_rigid) + feedback_at_mach_and_alpha(model.dCLift_deta) @ eta
    results.CM    = at_mach_and_alpha(model.CM_rigid)    + feedback_at_mach_and_alpha(model.dCM_deta)    @ eta
    return results


# ----------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------
def divergence_dynamic_pressure(model, mach, alpha=0.0):
    """
    Lowest dynamic pressure [Pa] at which the wings diverge, where (I - q S G) becomes singular, or infinity if
    there is none. This is the divergence eigenvalue problem of the MSC Nastran Aeroelastic Analysis User's Guide
    (2023.1, Eq. 2-145) on the station degrees of freedom.
    """
    m_lo, m_hi, m_w = bracket(model.mach, mach, hold_ends=True)
    f_w             = (alpha - model.feedback_alpha[0]) / (model.feedback_alpha[1] - model.feedback_alpha[0])
    n_stations      = len(model.stations)
    S_G             = np.zeros((n_stations, n_stations))
    for k, (tag, station_k, component) in enumerate(model.stations):
        G_table   = model.wings[tag].G
        G_at_mach = (1 - m_w) * G_table[m_lo] + m_w * G_table[m_hi]
        dof       = 6 * model.wings[tag].station_nodes[station_k] + component
        S_G[k, :] = (1 - f_w) * G_at_mach[0][dof, :] + f_w * G_at_mach[1][dof, :]
    eigenvalues = np.linalg.eigvals(S_G)
    positive    = eigenvalues.real[(np.abs(eigenvalues.imag) < 1e-9 * np.max(np.abs(eigenvalues))) & (eigenvalues.real > 0)]
    if len(positive) == 0:
        return np.inf
    return 1.0 / np.max(positive)


def bracket(table, value, hold_ends):
    """Indices of the two table entries around value and the weight of the upper one."""
    if len(table) == 1:
        return 0, 0, 0.0
    hi     = int(np.clip(np.searchsorted(table, value), 1, len(table) - 1))
    lo     = hi - 1
    weight = float((value - table[lo]) / (table[hi] - table[lo]))
    if hold_ends:
        weight = float(np.clip(weight, 0.0, 1.0))
    return lo, hi, weight


def inertial_responses(wing, node_data, beam, masses):
    """
    Beam response at one g to the masses that do not change in flight (structure, ribs, point masses, empty
    tanks), and to one kilogram of fuel in each tank.
    """
    g = STANDARD_GRAVITY

    def element_loads(w_z, point_pts, point_masses):
        zeros = np.zeros_like(node_data.Le)
        w_x, w_y, t_y = zeros, zeros, zeros
        if len(point_masses) > 0:
            loads               = np.column_stack((np.zeros((len(point_masses), 2)), -np.array(point_masses) * g))
            w_x, w_y, w_zp, t_y = point_loads_to_element_loads(wing, node_data, np.array(point_pts).reshape(-1, 3), loads)
            w_z                 = w_z + w_zp
        return np.array([w_x, w_y, w_z, t_y])

    # one kilogram of contents in each tank, on this half wing
    load_fuel  = {}
    tank_mass  = {}
    for (point, fuel_tag, mass, share, tank_tag) in masses.fuel_points:
        load_fuel[tank_tag] = element_loads(np.zeros_like(node_data.Le), [point], [share])
        tank_mass[tank_tag] = mass
    for (fuel_tag, mass, share, element_fraction, tank_tag) in masses.fuel_spread:
        load_fuel[tank_tag] = element_loads(-share * element_fraction / node_data.Le * g, [], [])
        tank_mass[tank_tag] = mass

    load_fixed = element_loads(self_weight_load(wing, node_data, beam), masses.point_pts, masses.point_masses)
    for tank_tag in load_fuel:
        load_fixed += tank_mass[tank_tag] * load_fuel[tank_tag]

    w_beam  = Data(beam=beam, node_data=node_data)
    u_fixed = beam_response(w_beam, load_fixed)
    u_fuel  = {tank_tag: beam_response(w_beam, load_fuel[tank_tag]) for tank_tag in load_fuel}
    return u_fixed, u_fuel, load_fixed, load_fuel


def beam_response(w, loads):
    """Nodal displacements of a wing's beam under element loads (w_x, w_y, w_z, t_y)."""
    return solve_beam(w.beam, nodal_force_vector(w.beam, w.node_data, loads[0], loads[1], loads[2], loads[3]))


def station_shapes(y_local, station_nodes):
    """
    Piecewise-linear shape of a unit twist at each station along the beam: one at its own station node, zero at
    its neighbours (and at the root), held constant outboard of the last station.
    """
    y_stations = np.concatenate(([y_local[0]], y_local[station_nodes]))
    shapes     = np.zeros((len(station_nodes), len(y_local)))
    for k in range(len(station_nodes)):
        values        = np.zeros(len(y_stations))
        values[k + 1] = 1.0
        shapes[k]     = np.interp(y_local, y_stations, values)
    return shapes


def deformed_mesh(jig_VD, vehicle, wings, station_tag, station_k, component, step):
    """Vortex lattice mesh with one degree of freedom at one station of one wing displaced by step."""
    deformation = Data()
    for tag, w in wings.items():
        n_nodes                               = len(w.node_data.Y_nodes)
        deformation[tag]                      = Data()
        deformation[tag].structural_node_data = w.node_data
        deformation[tag].deflection           = np.zeros((1, n_nodes, 3))
        deformation[tag].rotation             = np.zeros((1, n_nodes, 3))
    shape = step * wings[station_tag].station_shapes[station_k]
    if component < 3:
        deformation[station_tag].deflection[0, :, component]   = shape
    else:
        deformation[station_tag].rotation[0, :, component - 3] = shape
    return deform_vortex_distribution(jig_VD, vehicle, deformation, ti=0)


def aerodynamic_conditions(mach, alpha):
    """One symmetric flight condition for the vortex lattice method."""
    conditions                                 = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.mach_number          = np.array([[mach]])
    conditions.freestream.velocity             = np.array([[mach * 340.0]])   # the loads are per unit dynamic pressure
    conditions.aerodynamics.angles.alpha       = np.array([[alpha]])
    conditions.aerodynamics.angles.beta        = np.array([[0.0]])
    conditions.static_stability.pitch_rate     = np.array([[0.0]])
    conditions.static_stability.roll_rate      = np.array([[0.0]])
    conditions.static_stability.yaw_rate       = np.array([[0.0]])
    return conditions


def aerodynamic_loads(mach, alpha, VD, aerodynamics_settings, vehicle, wings):
    """Element loads (w_x, w_y, w_z, t_y) on each wing's beam per unit dynamic pressure, from a vortex lattice solve
    on the mesh VD."""
    results = VLM(aerodynamic_conditions(mach, alpha), aerodynamics_settings, vehicle, precomputed_VD=VD)
    VD      = results.VD

    panels_per_wing = VD.n_sw[0] * VD.n_cw[0]
    breaks          = np.concatenate(([0], np.cumsum(panels_per_wing)))
    vd_idx          = 0
    loads           = {}
    for wing in vehicle.wings.values():
        w                    = wings[wing.tag]
        aero_pts, aero_loads = panel_forces(results, VD, 0, 0, int(breaks[vd_idx]), int(breaks[vd_idx + 1]), 1.0)
        loads[wing.tag]      = np.array(point_loads_to_element_loads(wing, w.node_data, aero_pts, aero_loads))
        vd_idx += 2 if wing.xz_plane_symmetric else 1
    return loads, results
