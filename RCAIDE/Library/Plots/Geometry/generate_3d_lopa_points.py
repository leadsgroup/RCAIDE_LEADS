# RCAIDE/Library/Plots/Geometry/generate_3d_lopa_points.py
#
# Created: Oct 2025, S Shekar
import numpy as np
import pyvista as pv
from RCAIDE.Framework.Core import Data

def generate_3d_lopa_points(structure):
    """
    Generate VTK geometry for Layout Of Passenger Accommodation (LOPA).
    Expected columns in layout.object_coordinates (from 2D plot code):
        [.., 2:x, 3:y, 4:z?, 5:length, 6:width, 7:F_c, 8:B_c, 9:E_c, 10:seat, 11:Em_row, 12:Gal_Lav]
    """
    layout = getattr(structure, "layout_of_passenger_accommodations", None)
    if layout is None or not hasattr(layout, "object_coordinates"):
        return _empty_geometry()
    coords = np.asarray(layout.object_coordinates, float)
    seat_height_global = getattr(structure, "seat_height", None)
    if seat_height_global is None:
        seat_height_global = getattr(layout, "seat_height", None)

    # Accumulate cube PolyData by (class, is_emergency_row) to allow a single
    # add_mesh call per group instead of one per seat.
    batches: dict[tuple, list] = {
        ("first",      False): [],
        ("first",      True):  [],
        ("business",   False): [],
        ("business",   True):  [],
        ("economy",    False): [],
        ("economy",    True):  [],
        ("galley_lav", False): [],
        ("galley_lav", True):  [],
    }

    business_count     = 0
    economy_count      = 0
    first_count        = 0
    galley_lav_count   = 0
    emergency_row_count = 0

    for row in coords:
        x      = row[2] + layout.origin[0][0]
        y      = row[3] + layout.origin[0][1]
        z      = row[4] + layout.origin[0][2] if row.shape[0] > 4  else 0.0
        length = abs(row[5])
        width  = abs(row[6])
        F_c    = row[7]  if row.shape[0] > 7  else 0.0
        B_c    = row[8]  if row.shape[0] > 8  else 0.0
        E_c    = row[9]  if row.shape[0] > 9  else 0.0
        seat   = row[10] if row.shape[0] > 10 else 0.0
        Em_row = row[11] if row.shape[0] > 11 else 0.0
        GalLav = row[12] if row.shape[0] > 12 else 0.0

        seat_height = seat_height_global if seat_height_global is not None else width
        is_em = bool(Em_row == 1.0)

        cube = pv.Cube(
            center=(float(x), float(y), float(z + 0.5 * seat_height)),
            x_length=float(length),
            y_length=float(width),
            z_length=float(seat_height),
        )

        if GalLav == 1.0:
            galley_lav_count += 1
            batches[("galley_lav", is_em)].append(cube)
            if is_em:
                emergency_row_count += 1
            continue

        if seat != 1.0:
            continue

        if F_c == 1.0:
            first_count += 1
            batches[("first", is_em)].append(cube)
        elif B_c == 1.0:
            business_count += 1
            batches[("business", is_em)].append(cube)
        elif E_c == 1.0:
            economy_count += 1
            batches[("economy", is_em)].append(cube)

        if is_em:
            emergency_row_count += 1

    geom = _empty_geometry()
    geom._lopa_batches = {
        key: pv.merge(meshes) for key, meshes in batches.items() if meshes
    }
    geom.metadata = Data()
    geom.metadata.business_seat_count  = int(business_count)
    geom.metadata.economy_seat_count   = int(economy_count)
    geom.metadata.first_seat_count     = int(first_count)
    geom.metadata.galley_lav_count     = int(galley_lav_count)
    geom.metadata.emergency_row_count  = int(emergency_row_count)
    return geom


def _empty_geometry():
    geom = Data()
    geom.PTS = np.zeros((1, 1, 3))
    geom._lopa_seats   = []   # kept for backwards compatibility
    geom._lopa_batches = {}
    geom.metadata = Data()
    geom.metadata.business_seat_count  = 0
    geom.metadata.economy_seat_count   = 0
    geom.metadata.first_seat_count     = 0
    geom.metadata.galley_lav_count     = 0
    geom.metadata.emergency_row_count  = 0
    return geom
