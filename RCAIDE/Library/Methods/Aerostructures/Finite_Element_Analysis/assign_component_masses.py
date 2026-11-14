# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/assign_component_masses.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data
import numpy as np

# ----------------------------------------------------------------------
#  assign_component_masses
# ----------------------------------------------------------------------
def assign_component_masses(geometry, wing, node_data):
    """
    Component masses carried by the FEA model of one wing.

    The FEA models one half of an xz-symmetric wing, so a component on the other side (y < 0 relative to the
    wing origin) belongs to the mirrored half and is skipped. Battery modules and fuel tanks are carried by the
    wing named by their wing_tag; propulsors have no wing tag, so wing-mounted propulsors are carried by main
    wings.

    An integral tank on a symmetric wing spans both halves, with its centre of gravity on the centreline. Each
    half carries half its fuel, spread over the elements between the tank's bounding segments in proportion to
    the local section area (chord times depth), which assumes a uniform fuel level across the tank. Other tanks
    are point masses at their centre of gravity.

    Returns:
        Data with
          point_pts, point_masses : constant point masses [m], [kg]
          fuel_points             : list of (point, fuel_tag, tank_mass, share, tank_tag)
          fuel_spread             : list of (fuel_tag, tank_mass, share, element_fraction, tank_tag)
    """
    symmetric = wing.xz_plane_symmetric
    y_origin  = wing.origin[0][1]

    point_pts    = []
    point_masses = []
    fuel_points  = []
    fuel_spread  = []

    for network in geometry.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                if source.wing_tag != wing.tag:
                    continue
                if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank):
                    share = 0.5 if symmetric else 1.0
                    fuel_spread.append((source.fuel.tag, source.mass_properties.mass, share,
                                        integral_tank_element_fraction(source, wing, node_data), source.tag))
                else:
                    point = (np.array(source.origin[0]) + np.array(source.mass_properties.center_of_gravity)).reshape(3)
                    if on_this_half(point, symmetric, y_origin):
                        fuel_points.append((point, source.fuel.tag, source.mass_properties.mass, 1.0, source.tag))

            elif isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                for battery_module in source.modules:
                    if battery_module.wing_tag != wing.tag:
                        continue
                    point = (np.array(battery_module.origin[0]) + np.array(battery_module.mass_properties.center_of_gravity)).reshape(3)
                    if on_this_half(point, symmetric, y_origin):
                        point_pts.append(point)
                        point_masses.append(battery_module.mass_properties.mass)

        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            for propulsor in network.propulsors:
                if not propulsor.wing_mounted:
                    continue
                point = (np.array(propulsor.origin[0]) + np.array(propulsor.mass_properties.center_of_gravity)).reshape(3)
                if on_this_half(point, symmetric, y_origin):
                    point_pts.append(point)
                    point_masses.append(propulsor.mass_properties.mass)

    return Data(point_pts    = np.array(point_pts).reshape(-1, 3),
                point_masses = np.array(point_masses, dtype=float),
                fuel_points  = fuel_points,
                fuel_spread  = fuel_spread)


def on_this_half(point, symmetric, y_origin):
    """True if a component at point is carried by the modelled half of the wing."""
    return (not symmetric) or (point[1] - y_origin >= 0.0)


def integral_tank_element_fraction(fuel_tank, wing, node_data):
    """
    Fraction of a half-wing's tank contents in each element: elements between the tank's bounding segments,
    weighted by section area (chord times depth) times element length.
    """
    inner_tag, outer_tag = fuel_tank.segments_bounding_tank[0], fuel_tank.segments_bounding_tank[-1]
    y_inner  = wing.segments[inner_tag].percent_span_location * node_data.total_span
    y_outer  = wing.segments[outer_tag].percent_span_location * node_data.total_span
    inside   = (node_data.Y_elems >= y_inner) & (node_data.Y_elems <= y_outer)
    if not np.any(inside):
        raise ValueError(f"Integral tank '{fuel_tank.tag}' covers no elements of wing '{wing.tag}'")
    weight   = np.where(inside, node_data.chord_elems * node_data.t_elems * node_data.Le, 0.0)
    return weight / np.sum(weight)
