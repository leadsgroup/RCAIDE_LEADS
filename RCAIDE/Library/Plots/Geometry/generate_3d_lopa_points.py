# RCAIDE/Library/Plots/Geometry/generate_3d_lopa_points.py
#
# Created: Oct 2025, S Shekar
import numpy as np
import vtk
import matplotlib.colors as mcolors
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
    # Seat height preference: structure.seat_height → layout.seat_height → local width
    seat_height_global = getattr(structure, "seat_height", None)
    if seat_height_global is None:
        seat_height_global = getattr(layout, "seat_height", None)
    seats = []
    business_count = 0
    economy_count = 0
    first_count = 0
    galley_lav_count = 0
    emergency_row_count = 0
    for row in coords:
        # Parse row variables
        x = row[2] + layout.cabin_x_offset
        y = row[3]
        z = row[4] if row.shape[0] > 4 else 0.0
        length = abs(row[5])
        width  = abs(row[6])
        F_c     = row[7]  if row.shape[0] > 7  else 0.0
        B_c     = row[8]  if row.shape[0] > 8  else 0.0
        E_c     = row[9]  if row.shape[0] > 9  else 0.0
        seat    = row[10] if row.shape[0] > 10 else 0.0
        Em_row  = row[11] if row.shape[0] > 11 else 0.0
        GalLav  = row[12] if row.shape[0] > 12 else 0.0
        # Determine per-row seat height (no external constants)
        seat_height = seat_height_global if seat_height_global is not None else width
        # Galley/Lav blocks
        if GalLav == 1.0:
            galley_lav_count += 1
            cube = vtk.vtkCubeSource()
            # Center at half-height above the local z
            cube.SetCenter(float(x), float(y), float(z + 0.5 * seat_height))
            cube.SetXLength(float(length))
            cube.SetYLength(float(width))
            cube.SetZLength(float(seat_height))
            cube.Update()
            pd = vtk.vtkPolyData()
            pd.ShallowCopy(cube.GetOutput())
            seats.append({
                "polydata": pd,
                "class": "galley_lav",
                "emergency_row": bool(Em_row == 1.0),
                "flags": {"F": F_c, "B": B_c, "E": E_c, "seat": seat, "GalLav": GalLav}
            })
            if Em_row == 1.0:
                emergency_row_count += 1
            continue
        # Seats (only if seat flag is 1)
        if seat != 1.0:
            continue
        seat_class = None
        if F_c == 1.0:
            seat_class = "first"
            first_count += 1
        elif B_c == 1.0:
            seat_class = "business"
            business_count += 1
        elif E_c == 1.0:
            seat_class = "economy"
            economy_count += 1
        # Build the cube
        cube = vtk.vtkCubeSource()
        cube.SetCenter(float(x), float(y), float(z + 0.5 * seat_height))
        cube.SetXLength(float(length))
        cube.SetYLength(float(width))
        cube.SetZLength(float(seat_height))
        cube.Update()
        pd = vtk.vtkPolyData()
        pd.ShallowCopy(cube.GetOutput())
        seats.append({
            "polydata": pd,
            "class": seat_class,
            "emergency_row": bool(Em_row == 1.0),
            "flags": {"F": F_c, "B": B_c, "E": E_c, "seat": seat, "GalLav": GalLav}
        })
        if Em_row == 1.0:
            emergency_row_count += 1
    geom = _empty_geometry()
    geom._lopa_seats = seats
    geom.metadata = Data()
    geom.metadata.business_seat_count = int(business_count)
    geom.metadata.economy_seat_count  = int(economy_count)
    geom.metadata.first_seat_count    = int(first_count)
    geom.metadata.galley_lav_count    = int(galley_lav_count)
    geom.metadata.emergency_row_count = int(emergency_row_count)
    return geom
def _empty_geometry():
    geom = Data()
    geom.PTS = np.zeros((1, 1, 3))
    geom._lopa_seats = []
    geom.metadata = Data()
    geom.metadata.business_seat_count = 0
    geom.metadata.economy_seat_count  = 0
    geom.metadata.first_seat_count    = 0
    geom.metadata.galley_lav_count    = 0
    geom.metadata.emergency_row_count = 0
    return geom