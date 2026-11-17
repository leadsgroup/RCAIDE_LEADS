# VnV/Verification/geometry/test_solids.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import trimesh

from RCAIDE.Library.Methods.Geometry.Airfoil import compute_naca_4series
from RCAIDE.Library.Methods.Geometry.Solids  import (polygon_properties, cut_section, loft_properties, combine_properties,
                                                     mirror_properties, scale_properties)

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The native solids (polygon moments, airfoil cuts, lofts, and their combination, mirror image and scaling) are
    # compared with trimesh on the same solids: a box, a swept and tapered loft of a cambered airfoil cut at its spars,
    # a loft between two different airfoils, and a two-panel symmetric tank scaled to its usable volume. They must
    # agree to round-off. The polygon moments are also compared with trimesh's area of the same section.
    check_polygon()
    check_box()
    check_airfoil_loft()
    check_mixed_airfoil_loft()
    check_symmetric_tank()
    print('test_solids passed')
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_polygon():
    # a non-convex cut section: area and centroid against the sum of a fan of signed triangles over the same polygon
    x, z      = naca_cut('4412', 0.1, 0.7)
    area, (sx, sz), _ = polygon_properties(x, z)
    triangles = fan_triangles(x, z)
    areas     = 0.5 * ((triangles[:, 1, 0] - triangles[:, 0, 0]) * (triangles[:, 2, 1] - triangles[:, 0, 1])
                       - (triangles[:, 2, 0] - triangles[:, 0, 0]) * (triangles[:, 1, 1] - triangles[:, 0, 1]))
    centers   = triangles.mean(axis=1)
    compare('polygon area', area, abs(np.sum(areas)), 1e-13)
    compare('polygon centroid x', sx / area, np.sum(areas * centers[:, 0]) / np.sum(areas), 1e-12)
    compare('polygon centroid z', sz / area, np.sum(areas * centers[:, 1]) / np.sum(areas), 1e-12)
    return


def check_box():
    # a box 2 by 3 by 0.5 m, its corner at (1, 4, -0.2): a loft of two equal rectangles against trimesh's box
    x = np.array([1., 3., 3., 1.])
    z = np.array([-0.2, -0.2, 0.3, 0.3])
    properties = loft_properties(x, z, x, z, 4., 7.)
    box        = trimesh.creation.box(extents=[2., 3., 0.5])
    box.apply_translation([2., 5.5, 0.05])
    compare_with_mesh('box', properties, box, 1e-12)
    return


def check_airfoil_loft():
    # NACA 2412 cut at 15% and 65% chord, root chord 5 m to tip chord 1.5 m, 30 degrees of leading-edge sweep,
    # 5 degrees of dihedral, over 12 m of span
    inner_x, inner_z = place(*naca_cut('2412', 0.15, 0.65), chord=5.0, x0=0.0, z0=0.0)
    outer_x, outer_z = place(*naca_cut('2412', 0.15, 0.65), chord=1.5, x0=12. * np.tan(np.radians(30.)), z0=12. * np.tan(np.radians(5.)))
    properties = loft_properties(inner_x, inner_z, outer_x, outer_z, 0., 12.)
    compare_with_mesh('swept tapered loft', properties, loft_mesh(inner_x, inner_z, outer_x, outer_z, 0., 12.), 1e-11)
    return


def check_mixed_airfoil_loft():
    # a thick root airfoil to a thin cambered tip airfoil, cut at different spar positions
    inner_x, inner_z = place(*naca_cut('0018', 0.10, 0.60), chord=4.0, x0=0.5, z0=0.1)
    outer_x, outer_z = place(*naca_cut('6409', 0.20, 0.70), chord=2.0, x0=3.0, z0=0.6)
    properties = loft_properties(inner_x, inner_z, outer_x, outer_z, 1.5, 9.0)

    # the side faces are twisted here, so flat triangles only approach the blended surface: the mesh's center of
    # gravity must approach the native one as the square of the number of rings, and agree on a fine mesh
    errors = []
    for rings in (1, 4, 16):
        mesh = loft_mesh(inner_x, inner_z, outer_x, outer_z, 1.5, 9.0, rings)
        errors.append(np.max(np.abs(np.asarray(properties.center_of_gravity) - mesh.center_mass)))
        print(f"  mixed airfoil loft, {rings:3d} rings: center of gravity difference {errors[-1]:.2e} m")
    assert errors[1] < errors[0] / 10 and errors[2] < errors[1] / 10, f"the mesh does not approach the loft: {errors}"
    compare_with_mesh('mixed airfoil loft', properties, loft_mesh(inner_x, inner_z, outer_x, outer_z, 1.5, 9.0, 128), 1e-11)
    return


def check_symmetric_tank():
    # two panels, mirrored across the plane of symmetry and scaled to 85% of their volume: combine_properties,
    # mirror_properties and scale_properties against the same operations on trimesh meshes
    x0, z0 = place(*naca_cut('2412', 0.15, 0.65), chord=6.0, x0=0.0, z0=0.0)
    x1, z1 = place(*naca_cut('2412', 0.15, 0.65), chord=3.5, x0=2.5, z0=0.4)
    x2, z2 = place(*naca_cut('2410', 0.15, 0.65), chord=1.2, x0=7.0, z0=1.0)
    panels = [loft_properties(x0, z0, x1, z1, 2.0, 5.0), loft_properties(x1, z1, x2, z2, 5.0, 14.0)]
    half   = scale_properties(combine_properties(panels), 0.85)
    whole  = combine_properties([half, mirror_properties(half)])

    # the outer panel blends two airfoils, so its mesh needs fine rings (see check_mixed_airfoil_loft)
    mesh   = trimesh.util.concatenate([loft_mesh(x0, z0, x1, z1, 2.0, 5.0), loft_mesh(x1, z1, x2, z2, 5.0, 14.0, 128)])
    mesh.apply_transform(trimesh.transformations.scale_matrix(0.85 ** (1. / 3.), origin=mesh.center_mass))
    mirror = mesh.copy()
    mirror.vertices = mirror.vertices * np.array([1., -1., 1.])
    mirror.faces    = mirror.faces[:, ::-1]
    compare_with_mesh('symmetric scaled two-panel tank', whole, trimesh.util.concatenate([mesh, mirror]), 1e-11)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def naca_cut(code, start, end, number_of_stations=81):
    """A NACA 4-series airfoil per unit chord cut between two chord fractions."""
    airfoil = compute_naca_4series(code, 201)
    return cut_section(airfoil.x_upper_surface, airfoil.y_upper_surface, airfoil.x_lower_surface,
                       airfoil.y_lower_surface, start, end, number_of_stations)


def place(x, z, chord, x0, z0):
    """A section per unit chord scaled to a chord and moved to a leading edge position."""
    return x * chord + x0, z * chord + z0


def fan_triangles(x, z):
    """Triangles from the first vertex to every edge of a polygon; signed, so they sum to the polygon."""
    points = np.column_stack([x, z])
    return np.stack([np.repeat(points[:1], len(x) - 2, axis=0), points[1:-1], points[2:]], axis=1)


def loft_mesh(inner_x, inner_z, outer_x, outer_z, inner_y, outer_y, rings=1):
    """The loft as a closed trimesh: rings of matched vertices along the linear blend joined by two triangles per
    side face, fan-triangulated caps. Between sections of different shape a side face is twisted, so flat triangles
    only approach the blended surface as the number of rings grows."""
    n     = len(inner_x)
    verts = []
    for t in np.linspace(0., 1., rings + 1):
        verts.append(np.column_stack([(1 - t) * inner_x + t * outer_x, np.full(n, (1 - t) * inner_y + t * outer_y),
                                      (1 - t) * inner_z + t * outer_z]))
    verts = np.vstack(verts)
    last  = rings * n
    # each side face is split into four triangles around its center, which lies on the blended surface, so the
    # twist of the face cancels to leading order
    centers = []
    faces   = []
    for r in range(rings):
        for k in range(n):
            a, b = r * n + k, r * n + (k + 1) % n
            c    = len(verts) + len(centers)
            centers.append(0.25 * (verts[a] + verts[b] + verts[n + a] + verts[n + b]))
            faces += [[a, b, c], [b, n + b, c], [n + b, n + a, c], [n + a, a, c]]
    verts = np.vstack([verts, np.array(centers)])
    for k in range(1, n - 1):
        faces += [[0, k + 1, k], [last, last + k, last + k + 1]]
    mesh = trimesh.Trimesh(verts, np.array(faces), process=False)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def compare_with_mesh(name, properties, mesh, tolerance):
    """Volume, center of gravity and inertia per unit mass against trimesh's mass properties of the same solid."""
    compare(f'{name} volume', properties.volume, mesh.volume, tolerance)
    scale = np.max(np.abs(mesh.center_mass)) + np.sqrt(np.max(np.abs(mesh.moment_inertia)) / mesh.mass)
    error = np.max(np.abs(np.asarray(properties.center_of_gravity) - mesh.center_mass)) / scale
    print(f"  {name} center of gravity: native {np.asarray(properties.center_of_gravity)}, trimesh {mesh.center_mass}, relative difference {error:.1e}")
    assert error <= tolerance, f"{name}: center of gravity differs by {error}"
    inertia = mesh.moment_inertia / mesh.mass
    error   = np.max(np.abs(np.asarray(properties.inertia_per_mass) - inertia)) / np.max(np.abs(inertia))
    print(f"  {name} inertia per unit mass: relative difference {error:.1e}")
    assert error <= tolerance, f"{name}: inertia per unit mass differs by {error}"
    return


def compare(name, value, expected, tolerance):
    error = abs(value - expected) / max(abs(expected), 1e-300)
    print(f"  {name}: native {value:.12g}, reference {expected:.12g}, relative difference {error:.1e}")
    assert error <= tolerance, f"{name}: {value} against {expected}"
    return


if __name__ == '__main__':
    main()
