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
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.FEA                          import assemble_beam, panel_forces, point_loads_to_element_loads, self_weight_load, nodal_force_vector, solve_beam
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing              import discretize_wing, deform_vortex_distribution
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.assign_component_masses      import assign_component_masses

import numpy as np

STANDARD_GRAVITY = 9.81   # [m/s^2] used by self_weight_load

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
    mach_numbers     = np.atleast_1d(np.array(mach_numbers, dtype=float))
    angles_of_attack = np.atleast_1d(np.array(angles_of_attack, dtype=float))
    n_mach           = len(mach_numbers)
    n_alpha          = len(angles_of_attack)

    # beam, component masses and inertial responses of each wing
    wings = Data()
    for wing in vehicle.wings.values():
        node_data = discretize_wing(wing, structural_settings.discretiation)
        beam      = assemble_beam(wing, node_data, structural_settings)
        masses    = assign_component_masses(vehicle, wing, node_data)

        w              = Data()
        w.node_data    = node_data
        w.beam         = beam
        w.u_fixed, w.u_fuel = inertial_responses(wing, node_data, beam, masses)
        wings[wing.tag] = w

    # twist stations: (wing tag, node index), and the shape of each station's unit twist along its wing
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

    for w in wings.values():
        w.u_rigid  = np.zeros((n_mach, n_alpha, w.beam.total_dof))      # response to the rigid-wing loads, per unit q
        w.G        = np.zeros((n_mach, 2, w.beam.total_dof, n_stations))   # response to the loads of each unit station displacement, per unit q

    dCLift_deta = np.zeros((n_mach, 2, n_stations))
    dCM_deta    = np.zeros((n_mach, 2, n_stations))
    CLift_rigid = np.zeros((n_mach, n_alpha))
    CM_rigid    = np.zeros((n_mach, n_alpha))

    for m, mach in enumerate(mach_numbers):
        # rigid wing at each angle of attack
        jig_VD = generate_vortex_distribution(aerodynamic_conditions(mach, angles_of_attack[0]), aerodynamics_settings, vehicle)
        for a, alpha in enumerate(angles_of_attack):
            F, results = aerodynamic_forces(mach, alpha, jig_VD, aerodynamics_settings, vehicle, wings)
            for tag, w in wings.items():
                w.u_rigid[m, a] = solve_beam(w.beam, F[tag])
            CLift_rigid[m, a] = results.CLift[0, 0]
            CM_rigid[m, a]    = results.CM[0, 0]

        # unit displacement of each station, by central difference about the jig shape, at the two feedback angles of attack
        for r, alpha in enumerate(feedback_angles_of_attack):
            for k, (station_tag, station_k, component) in enumerate(stations):
                step                   = translation_step if component < 3 else rotation_step
                F_plus,  results_plus  = aerodynamic_forces(mach, alpha, deformed_mesh(jig_VD, vehicle, wings, station_tag, station_k, component,  step), aerodynamics_settings, vehicle, wings)
                F_minus, results_minus = aerodynamic_forces(mach, alpha, deformed_mesh(jig_VD, vehicle, wings, station_tag, station_k, component, -step), aerodynamics_settings, vehicle, wings)
                for tag, w in wings.items():
                    w.G[m, r, :, k] = solve_beam(w.beam, (F_plus[tag] - F_minus[tag]) / (2 * step))
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
        results : Data by wing tag with deflection (nodes, 3), rotation (nodes, 3) and elastic_twist (nodes,),
                  plus eta, CLift and CM
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
    u_0 = {}
    G   = {}
    for tag, w in model.wings.items():
        u_inertial = w.u_fixed.copy()
        for tank_tag, u_per_kg in w.u_fuel.items():
            u_inertial += fuel_masses.get(tank_tag, 0.0) * u_per_kg
        u_0[tag] = q * at_mach_and_alpha(w.u_rigid) + inertial_scale * u_inertial
        G[tag]   = feedback_at_mach_and_alpha(w.G)

    # twist at the stations, with the aerodynamic feedback
    n_stations = len(model.stations)
    eta_0      = np.zeros(n_stations)
    S_G        = np.zeros((n_stations, n_stations))
    for k, (tag, station_k, component) in enumerate(model.stations):
        dof        = 6 * model.wings[tag].station_nodes[station_k] + component
        eta_0[k]   = u_0[tag][dof]
        S_G[k, :]  = G[tag][dof, :]
    if coupled and n_stations > 0:
        eta = np.linalg.solve(np.eye(n_stations) - q * S_G, eta_0)
    else:
        eta = np.zeros(n_stations)

    results     = Data()
    results.eta = eta if coupled else eta_0
    for tag, w in model.wings.items():
        u                           = u_0[tag] + q * G[tag] @ eta
        results[tag]                = Data()
        results[tag].deflection     = np.column_stack((u[0::6], u[1::6], u[2::6]))
        results[tag].rotation       = np.column_stack((u[3::6], u[4::6], u[5::6]))
        results[tag].elastic_twist  = u[3::6] * np.sin(w.node_data.sweep_nodes) + u[4::6] * np.cos(w.node_data.sweep_nodes)
    results.CLift = at_mach_and_alpha(model.CLift_rigid) + feedback_at_mach_and_alpha(model.dCLift_deta) @ eta
    results.CM    = at_mach_and_alpha(model.CM_rigid)    + feedback_at_mach_and_alpha(model.dCM_deta)    @ eta
    return results


# ----------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------
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

    def response(w_z, point_pts, point_masses):
        zeros = np.zeros_like(node_data.Le)
        w_x, w_y, t_y = zeros, zeros, zeros
        if len(point_masses) > 0:
            loads              = np.column_stack((np.zeros((len(point_masses), 2)), -np.array(point_masses) * g))
            w_x, w_y, w_zp, t_y = point_loads_to_element_loads(wing, node_data, np.array(point_pts).reshape(-1, 3), loads)
            w_z                = w_z + w_zp
        return solve_beam(beam, nodal_force_vector(beam, node_data, w_x, w_y, w_z, t_y))

    # one kilogram of contents in each tank, on this half wing
    u_fuel     = {}
    tank_mass  = {}
    for (point, fuel_tag, mass, share, tank_tag) in masses.fuel_points:
        u_fuel[tank_tag]    = response(np.zeros_like(node_data.Le), [point], [share])
        tank_mass[tank_tag] = mass
    for (fuel_tag, mass, share, element_fraction, tank_tag) in masses.fuel_spread:
        u_fuel[tank_tag]    = response(-share * element_fraction / node_data.Le * g, [], [])
        tank_mass[tank_tag] = mass

    u_fixed = response(self_weight_load(wing, node_data, beam), masses.point_pts, masses.point_masses)
    for tank_tag in u_fuel:
        u_fixed += tank_mass[tank_tag] * u_fuel[tank_tag]
    return u_fixed, u_fuel


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


def aerodynamic_forces(mach, alpha, VD, aerodynamics_settings, vehicle, wings):
    """Nodal forces on each wing's beam per unit dynamic pressure, from a vortex lattice solve on the mesh VD."""
    results = VLM(aerodynamic_conditions(mach, alpha), aerodynamics_settings, vehicle, precomputed_VD=VD)
    VD      = results.VD

    panels_per_wing = VD.n_sw[0] * VD.n_cw[0]
    breaks          = np.concatenate(([0], np.cumsum(panels_per_wing)))
    vd_idx          = 0
    forces          = {}
    for wing in vehicle.wings.values():
        w                    = wings[wing.tag]
        aero_pts, aero_loads = panel_forces(results, VD, 0, 0, int(breaks[vd_idx]), int(breaks[vd_idx + 1]), 1.0)
        w_x, w_y, w_z, t_y   = point_loads_to_element_loads(wing, w.node_data, aero_pts, aero_loads)
        forces[wing.tag]     = nodal_force_vector(w.beam, w.node_data, w_x, w_y, w_z, t_y)
        vd_idx += 2 if wing.xz_plane_symmetric else 1
    return forces, results
