# RCAIDE/Library/Plots/Geometry/generate_3d_cuboid_points.py
#
# Created: Oct 2025, S Shekar
import numpy as np
from RCAIDE.Framework.Core import Data

def generate_3d_cuboid_points(component):
    """
    Generate 3D surface points for a cuboid component (e.g. cargo bay, battery module).
    Applies orientation_euler_angles rotation if defined on the component.
    """

    cuboid_points = np.zeros((4, 4, 3))

    l = component.length
    w = component.width
    h = component.height

    # build cuboid in local frame (4 cross-sections along length)
    cuboid_points[0, :, 0] = np.array([0, 0, 0, 0])
    cuboid_points[0, :, 1] = np.array([0, 0, 0, 0])
    cuboid_points[0, :, 2] = np.array([0, 0, 0, 0])

    cuboid_points[1, :, 0] = np.array([0, 0, 0, 0])
    cuboid_points[1, :, 1] = np.array([-w/2, w/2, w/2, -w/2])
    cuboid_points[1, :, 2] = np.array([-h/2, -h/2, h/2, h/2])

    cuboid_points[2, :, 0] = np.array([l, l, l, l])
    cuboid_points[2, :, 1] = np.array([-w/2, w/2, w/2, -w/2])
    cuboid_points[2, :, 2] = np.array([-h/2, -h/2, h/2, h/2])

    cuboid_points[3, :, 0] = np.array([l, l, l, l])
    cuboid_points[3, :, 1] = np.array([0, 0, 0, 0])
    cuboid_points[3, :, 2] = np.array([0, 0, 0, 0])

    # apply euler angle rotation if defined
    euler_angles = getattr(component, 'orientation_euler_angles', [0., 0., 0.])
    if any(a != 0. for a in euler_angles):
        rot = _euler_rotation_matrix(euler_angles[0], euler_angles[1], euler_angles[2])
        for i in range(4):
            for j in range(4):
                cuboid_points[i, j, :] = rot @ cuboid_points[i, j, :]

    # translate to component origin
    cuboid_points[:, :, 0] += component.origin[0][0]
    cuboid_points[:, :, 1] += component.origin[0][1]
    cuboid_points[:, :, 2] += component.origin[0][2]

    G = Data()
    G.PTS = cuboid_points

    return G


def _euler_rotation_matrix(phi, theta, psi):
    """Build a 3x3 rotation matrix from X-Y-Z euler angles (phi, theta, psi)."""
    cx, sx = np.cos(phi),   np.sin(phi)
    cy, sy = np.cos(theta), np.sin(theta)
    cz, sz = np.cos(psi),   np.sin(psi)

    Rx = np.array([[1,  0,   0],
                   [0,  cx, -sx],
                   [0,  sx,  cx]])

    Ry = np.array([[ cy, 0, sy],
                   [  0, 1,  0],
                   [-sy, 0, cy]])

    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [ 0,   0, 1]])

    return Rz @ Ry @ Rx
