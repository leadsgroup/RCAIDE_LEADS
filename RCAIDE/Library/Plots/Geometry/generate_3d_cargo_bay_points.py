# RCAIDE/Library/Plots/Geometry/generate_3d_cargo_bay_points.py
#
# Created: Oct 2025, S Shekar
import numpy as np
from RCAIDE.Framework.Core import Data


def generate_3d_cargo_bay_points(cargo_bay):
    """
    Generate VTK geometry for a cargo container with an LD-style chamfered profile.

    Parameters
    ----------
    cargo_bay : RCAIDE cargo bay component
        Must expose `.length`, `.width`, `.height`, and `.origin` attributes.

    Returns
    -------
    G : Data
        RCAIDE Data object with a `PTS` array of shape (4, 6, 3) ready for
        `generate_vtk_object`.  The cross-section is a hexagon with a flat top,
        vertical sides, and 45-degree chamfers on the two bottom corners —
        matching the profile of a lower-deck (LD) unit load device (ULD).

        Cross-section (front view):

             ___________
            |           |   flat top
            |           |   vertical sides
             \_________/    chamfered bottom corners
    """
    l  = cargo_bay.length
    w  = cargo_bay.width
    h  = cargo_bay.height
    ox = cargo_bay.origin[0][0]
    oy = cargo_bay.origin[0][1]
    oz = cargo_bay.origin[0][2]

    # Chamfer size — ~25 % of the smaller of width/height gives a realistic LD shape.
    chamfer = min(w, h) * 0.25

    # 6-point hexagonal cross-section in the y-z plane (centred at origin y/z).
    # Points run counter-clockwise from bottom-left:
    #   bottom-left → bottom-right → right → top-right → top-left → left
    ys = np.array([-w/2 + chamfer,  w/2 - chamfer,   w/2,   w/2,  -w/2,  -w/2])
    zs = np.array([-h/2,           -h/2,    -h/2 + chamfer,  h/2,  h/2,  -h/2 + chamfer])

    pts = np.zeros((4, 6, 3))

    # Slice 0: front cap — all points collapse to centre so generate_vtk_object
    #          produces a filled triangular fan closing the front face.
    pts[0, :, 0] = ox
    pts[0, :, 1] = oy
    pts[0, :, 2] = oz

    # Slice 1: front hexagonal cross-section
    pts[1, :, 0] = ox
    pts[1, :, 1] = ys + oy
    pts[1, :, 2] = zs + oz

    # Slice 2: rear hexagonal cross-section
    pts[2, :, 0] = ox + l
    pts[2, :, 1] = ys + oy
    pts[2, :, 2] = zs + oz

    # Slice 3: rear cap — symmetric degenerate fan closing the back face.
    pts[3, :, 0] = ox + l
    pts[3, :, 1] = oy
    pts[3, :, 2] = oz

    G = Data()
    G.PTS = pts
    return G
