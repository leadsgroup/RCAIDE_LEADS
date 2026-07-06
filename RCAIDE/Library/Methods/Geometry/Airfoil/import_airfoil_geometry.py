# RCAIDE/Library/Methods/Geometry/Airfoil/import_airfoil_geometry.py

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from RCAIDE.Framework.Core import  Data
import numpy as np
from scipy import interpolate

# ----------------------------------------------------------------------------------------------------------------------
# import_airfoil_geometry
# ----------------------------------------------------------------------------------------------------------------------
def import_airfoil_geometry(airfoil_geometry_file, npoints = 201,surface_interpolation = 'cubic'):
    """This imports an airfoil geometry from a text file  and store
    the coordinates of upper and lower surfaces as well as the mean
    camberline

    Assumptions:
    Works for Selig and Lednicer airfoil formats. Automatically detects which format based off first line of data. Assumes it is one of those two.
    Source:
    airfoiltools.com/airfoil/index - method for determining format and basic error checking
    Inputs:
    airfoil_geometry_files   <list of strings>
    surface_interpolation   - type of interpolation used in the SciPy function. Preferable options are linear, quardratic and cubic.
    Full list of options can be found here :
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html#scipy.interpolate.interp1d
    Outputs:
    airfoil_data.
        thickness_to_chord
        x_coordinates
        y_coordinates
        x_upper_surface
        x_lower_surface
        y_upper_surface
        y_lower_surface
        camber_coordinates
    Properties Used:
    N/A
    """

    if npoints%2 != 1:
        npoints+= 1
        print('Number of points must be odd, changing to ' + str(npoints) + ' points')

    geometry     = Data()
    half_npoints = npoints//2

    # Open file and read column names and data block
    try:
        f = open(airfoil_geometry_file)
    except:
        raise FileNotFoundError('Airfoil file not in correct directory. Update file path of airfoil in vehicle setup.')

    # Extract data
    data_block = f.readlines()
    try:
        # Check for header block
        first_element = float(data_block[0][0])
        if first_element == 1.:
            lednicer_format = False
    except:
        # Check for format line and remove header block
        format_line = data_block[1]

        # Check if it's a Selig or Lednicer file
        try:
            format_flag = float(format_line.strip().split()[0])
        except:
            format_flag = float(format_line.strip().split(',')[0])

        if format_flag > 1.01: # Amount of wiggle room per airfoil tools
            lednicer_format = True
            # Remove header block
            data_block      = data_block[3:]
        else:
            lednicer_format = False
            # Remove header block
            data_block = data_block[1:]

    # Close the file
    f.close()

    if lednicer_format:
        x_up_surf = []
        y_up_surf = []
        x_lo_surf = []
        y_lo_surf = []

        # Loop through each value: append to each column
        upper_surface_flag = True
        for line_count , line in enumerate(data_block):
            #check for blank line which signifies the upper/lower surface division
            line_check = data_block[line_count].strip()
            if line_check == '':
                upper_surface_flag = False
                continue
            if upper_surface_flag:
                x_up_surf.append(float(data_block[line_count].strip().split()[0]))
                y_up_surf.append(float(data_block[line_count].strip().split()[1]))
            else:
                x_lo_surf.append(float(data_block[line_count].strip().split()[0]))
                y_lo_surf.append(float(data_block[line_count].strip().split()[1]))

    else:
        # Parse all data points first, then split at the leading edge (minimum x).
        # Selig format runs TE→LE on the upper surface, then LE→TE on the lower surface.
        # Detecting the split by "x starts increasing" is unreliable when the raw data has
        # non-monotone x values (e.g. reflex airfoils like the Eppler 325), so we instead
        # locate the leading edge as the global minimum-x point after reading everything.
        x_all = []
        y_all = []
        for line in data_block:
            line_stripped = line.strip().replace(',', '')
            if not line_stripped:
                continue
            parts = line_stripped.split()
            if len(parts) < 2:
                continue
            try:
                x_all.append(float(parts[0]))
                y_all.append(float(parts[1]))
            except ValueError:
                continue

        x_all_arr = np.array(x_all)
        y_all_arr = np.array(y_all)

        # Leading edge index: global minimum x
        le_idx = np.argmin(x_all_arr)

        # Upper surface: TE→LE in file, reversed to LE→TE
        x_up_surf = list(x_all_arr[:le_idx + 1][::-1])
        y_up_surf = list(y_all_arr[:le_idx + 1][::-1])

        # Lower surface: LE→TE
        x_lo_surf = list(x_all_arr[le_idx:])
        y_lo_surf = list(y_all_arr[le_idx:])

    x_up_surf = np.array(x_up_surf)
    x_lo_surf = np.array(x_lo_surf)
    y_up_surf = np.array(y_up_surf)
    y_lo_surf = np.array(y_lo_surf)

    # Check for extra zeros (OpenVSP exports extra zeros); check each surface independently
    # since with global-LE detection, duplicates may appear in x_lo_surf but not x_up_surf
    if len(np.unique(x_lo_surf)) != len(x_lo_surf):
        x_lo_surf = x_lo_surf[1:]
        y_lo_surf = y_lo_surf[1:]
    if len(np.unique(x_up_surf)) != len(x_up_surf):
        x_up_surf = x_up_surf[1:]
        y_up_surf = y_up_surf[1:]

    # create custom spacing for more points and leading and trailing edge
    t            = np.linspace(0,4,npoints-1)
    delta        = 0.25
    A            = 5
    f            = 0.25
    smoothsq     = 5 + (2*A/np.pi) *np.arctan(np.sin(2*np.pi*t*f + np.pi/2)/delta)
    dim_spacing  = np.append(0,np.cumsum(smoothsq)/sum(smoothsq))

    # compute thickness, camber and concatenate coodinates
    x_data        = np.hstack((x_lo_surf[::-1], x_up_surf[1:]))
    y_data        = np.hstack((y_lo_surf[::-1], y_up_surf[1:]))
    tck,u         = interpolate.splprep([x_data,y_data],k=3,s=0)
    out           = interpolate.splev(dim_spacing,tck)
    x_data        = out[0]
    y_data        = out[1]

    # shift points to leading edge (x = 0, y = 0)
    x_delta  = min(x_data)
    x_data   = x_data - x_delta

    arg_min  = np.argmin(x_data)
    y_delta  = y_data[arg_min]
    y_data   = y_data - y_delta

    if (x_data[arg_min] == 0) and (y_data[arg_min]  == 0):
        x_data[arg_min]  = 0
        y_data[arg_min]  = 0

    # make sure points start and end at x = 1.0
    x_data[0]  = 1.0
    x_data[-1] = 1.0

    # make sure a small gap at trailing edge
    if (y_data[0] == y_data[-1]):
        y_data[0]          = y_data[0]  - 1E-4
        y_data[-1]         = y_data[-1] + 1E-4

    # Resample both surfaces onto a shared x grid (LE→TE) so that thickness and
    # camber are computed at matching chordwise stations. Index-based resampling
    # was used previously but produces x-mismatches up to ~10 % chord and cubic
    # overshoot into negative x near the LE for reflex airfoils (e.g. Eppler 325).
    x_up_surf_old  = np.array(x_up_surf)
    y_up_surf_old  = np.array(y_up_surf)
    x_lo_surf_old  = np.array(x_lo_surf)
    y_lo_surf_old  = np.array(y_lo_surf)

    x_common       = np.linspace(x_up_surf_old[0], 1.0, half_npoints)
    x_up_surf_new  = x_common
    x_lo_surf_new  = x_common

    arry_up_interp = interpolate.interp1d(x_up_surf_old, y_up_surf_old, kind=surface_interpolation,
                                          bounds_error=False, fill_value=(y_up_surf_old[0], y_up_surf_old[-1]))
    y_up_surf_new  = arry_up_interp(x_common)

    arry_lo_interp = interpolate.interp1d(x_lo_surf_old, y_lo_surf_old, kind=surface_interpolation,
                                          bounds_error=False, fill_value=(y_lo_surf_old[0], y_lo_surf_old[-1]))
    y_lo_surf_new  = arry_lo_interp(x_common)

    # compute thickness, camber and concatenate coodinates
    thickness      = y_up_surf_new - y_lo_surf_new
    camber         = y_lo_surf_new + thickness/2
    max_t          = np.max(thickness)
    max_c          = max(x_data) - min(x_data)
    t_c            = max_t/max_c

    geometry.thickness_to_chord = t_c
    geometry.max_thickness      = max_t
    geometry.x_coordinates      = x_data
    geometry.y_coordinates      = y_data
    geometry.x_upper_surface    = x_up_surf_new
    geometry.x_lower_surface    = x_lo_surf_new
    geometry.y_upper_surface    = y_up_surf_new
    geometry.y_lower_surface    = y_lo_surf_new
    geometry.camber_coordinates = camber

    return geometry
