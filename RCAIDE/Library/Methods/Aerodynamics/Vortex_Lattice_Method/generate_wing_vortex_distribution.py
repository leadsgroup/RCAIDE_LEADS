# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/generate_wing_vortex_distribution.py
# 
# Created:  May 2018, M. Clarke
# Modified: Apr 2020, M. Clarke
#           Jun 2021, A. Blaufox
#           Apr 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                  import Data, array_namespace
from RCAIDE.Library.Methods.Geometry.Airfoil                import compute_naca_4series, import_airfoil_geometry
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments
  
# package imports 
import numpy as np


# ----------------------------------------------------------------------
#  Discretize Wings
# ----------------------------------------------------------------------
def generate_wing_vortex_distribution(VD,wing,n_cw,n_sw,spc,layout=None):
    """ This generates vortex distribution points for the given wing 

    Assumptions:  
    
    For control surfaces, "positve" deflection corresponds to the RH rule where the axis of rotation is the OUTBOARD-pointing hinge vector
    symmetry: the LH rule is applied to the reflected surface for non-ailerons. Ailerons follow a RH rule for both sides
    
    The hinge_vector will only ever be calcualted on the first strip of any control/all-moving surface. It is assumed that all control
    surfaces are trapezoids, thus needing only one hinge, and that all all-moving surfaces have exactly one point of rotation.

    Source:   
    None
    
    Inputs:   
    VD                   - vortex distribution 
    
    Properties Used:
    N/A
    """  

    # unpack inputs from wing
    span          = wing.spans.projected
    xz_sym        = wing.xz_plane_symmetric
    xy_sym        = wing.xy_plane_symmetric
    yz_sym        = wing.yz_plane_symmetric
    wing_origin   = wing.origin[0]
    xp            = array_namespace(*wing_lattice_inputs(wing))

    VD.vortex_lift.append(wing.vortex_lift)
    if xz_sym is True :
        span = span/2
        VD.vortex_lift.append(wing.vortex_lift)

    VD.counter  +=1

    # spanwise stations as fractions of the span, and their locations
    if spc == True: # discretize wing using cosine spacing
        n               = np.linspace(n_sw+1,0,n_sw+1)         # vectorize
        thetan          = n*(np.pi/2)/(n_sw+1)                 # angular stations
        y_coordinates   = span*np.cos(thetan)   # y locations based on the angular spacing
    else:
        y_coordinates   = span*np.linspace(0,1,n_sw+1) if xp is not np else np.linspace(0,span,n_sw+1)
    del_y = y_coordinates[1:] - y_coordinates[:-1]

    # The layout holds what fixes the structure of the lattice: the wing segment of each station, its airfoil and
    # control surfaces. It is found from the wing's numbers on NumPy; a JAX trace of the same wing is given it.
    if layout is None:
        if xp is not np:
            raise ValueError("on JAX the lattice layout of the wing is needed: wing_lattice_layout(wing,n_cw,n_sw,spc) on NumPy")
        layout = wing_lattice_layout(wing,n_cw,n_sw,spc)

    # ---------------------------------------------------------------------------------------
    # STEP 1: Define empty vectors for coordinates of panes, control points and bound vortices
    # ---------------------------------------------------------------------------------------
    symmetry_mask = np.array([[0,0,0],[yz_sym,xz_sym,xy_sym]])
    signs         = np.array([1, -1])
    wing_seg_tags = list(wing.segments.keys())

    side_idx = 0
    for sym_sign in signs[[True,xz_sym]]:
        yz_sym_sign   = 1 if symmetry_mask[side_idx,0] == 0 else -1
        xz_sym_sign   = 1 if symmetry_mask[side_idx,1] == 0 else -1
        xy_sym_sign   = 1 if symmetry_mask[side_idx,2] == 0 else -1

        # rows of the nodes (w), the control points (c) and the bound vortices (h) of each spanwise station
        x_w_rows, y_w_rows, z_w_rows = [], [], []
        x_c_rows, y_c_rows, z_c_rows = [], [], []
        x_h_rows, y_h_rows, z_h_rows = [], [], []
        cs_ws = []

        # ---------------------------------------------------------------------------------------
        # STEP 2: Discretize the wing sections into panels
        # ---------------------------------------------------------------------------------------
        for y_i in range(len(y_coordinates)):
            station = layout.stations[y_i]
            delta_y = y_coordinates[y_i]

            inboard_segment  = wing.segments[wing_seg_tags[station.segment]]
            outboard_segment = wing.segments[wing_seg_tags[station.segment+1]]

            segment_root_chord    = wing.chords.root*inboard_segment.root_chord_percent
            segment_tip_chord     = wing.chords.root*outboard_segment.root_chord_percent
            segment_root_twist    = inboard_segment.twist
            segment_tip_twist     = outboard_segment.twist
            inboard_segment_y     = inboard_segment.percent_span_location * span
            outboard_segment_y    = outboard_segment.percent_span_location * span
            segment_span          = outboard_segment_y -  inboard_segment_y
            local_y_val           = delta_y - inboard_segment_y
            local_chord           = segment_root_chord -  (segment_root_chord - segment_tip_chord) *  local_y_val / segment_span
            local_twist           = segment_root_twist -  (segment_root_twist - segment_tip_twist) *  local_y_val / segment_span

            dihedral = inboard_segment.dihedral_outboard
            LE_sweep = inboard_segment.sweeps.leading_edge

            if wing.vertical:
                delta_z  = inboard_segment.origin[0][1] +  xp.tan(dihedral) * local_y_val
                delta_x  = inboard_segment.origin[0][0] +  xp.tan(LE_sweep) * local_y_val
            else:
                delta_z  = inboard_segment.origin[0][2] +  xp.tan(dihedral) * local_y_val
                delta_x  = inboard_segment.origin[0][0] +  xp.tan(LE_sweep) * local_y_val

            # airfoil at the station, scaled by the local chord
            airfoil_x_pts_dim = station.airfoil_x*local_chord
            airfoil_z_pts_dim = station.airfoil_z*local_chord

            # deflection of the control surfaces at the station
            LE_angle = 0.0
            TE_angle = 0.0
            if station.leading_edge_surface is not None:
                LE_angle = station.leading_edge_surface.deflection
            if station.trailing_edge_surface is not None:
                cs = station.trailing_edge_surface
                if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
                    TE_angle = cs.deflection * sym_sign
                elif type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder:
                    TE_angle = cs.deflection * xz_sym_sign
                elif type(cs) in (RCAIDE.Library.Components.Wings.Control_Surfaces.Flaperon,
                                   RCAIDE.Library.Components.Wings.Control_Surfaces.Elevon,
                                   RCAIDE.Library.Components.Wings.Control_Surfaces.Ruddervator):
                    # symmetric command plus the differential command, which flips on the mirrored side
                    TE_angle = cs.deflection + cs.secondary_deflection * sym_sign
                else:
                    TE_angle = cs.deflection

            airfoil_x_pts_1, airfoil_z_pts_1 = apply_control_surface_deflections(airfoil_x_pts_dim, airfoil_z_pts_dim,LE_angle, station.leading_edge_chord_fraction,
                                                                                 TE_angle, station.trailing_edge_chord_fraction, station.ahead_of_leading_hinge, station.behind_trailing_hinge)

            diff_x = xp.diff(airfoil_x_pts_1)
            diff_z = xp.diff(airfoil_z_pts_1)

            # get bound vortex locations
            airfoil_x_pts_1_h = airfoil_x_pts_1[:-1] + diff_x/4
            airfoil_z_pts_1_h = airfoil_z_pts_1[:-1] + diff_z/4

            # get control point locations
            airfoil_x_pts_1_c = airfoil_x_pts_1[:-1] + 3*diff_x/4
            airfoil_z_pts_1_c = airfoil_z_pts_1[:-1] + 3*diff_z/4

            # Apply leading edge sweep
            airfoil_x_pts_2   = delta_x + airfoil_x_pts_1
            airfoil_x_pts_2_h = delta_x + airfoil_x_pts_1_h
            airfoil_x_pts_2_c = delta_x + airfoil_x_pts_1_c

            # Apply dihedral
            airfoil_z_pts_3 = airfoil_z_pts_1 + delta_z
            airfoil_z_pts_3_h = airfoil_z_pts_1_h + delta_z
            airfoil_z_pts_3_c = airfoil_z_pts_1_c + delta_z

            # Apply twist about the leading edge
            pivot_x = delta_x
            pivot_z = delta_z
            airfoil_x_pts_4    = pivot_x + xp.cos(local_twist)*(airfoil_x_pts_2-pivot_x) + xp.sin(local_twist)*( airfoil_z_pts_3- pivot_z)
            airfoil_x_pts_4_h  = pivot_x + xp.cos(local_twist)*(airfoil_x_pts_2_h-pivot_x) + xp.sin(local_twist)*( airfoil_z_pts_3_h- pivot_z)
            airfoil_x_pts_4_c  = pivot_x + xp.cos(local_twist)*(airfoil_x_pts_2_c-pivot_x) + xp.sin(local_twist)*( airfoil_z_pts_3_c- pivot_z)
            airfoil_z_pts_4   = pivot_z - xp.sin(local_twist)*(airfoil_x_pts_2-pivot_x) + xp.cos(local_twist)*( airfoil_z_pts_3- pivot_z)
            airfoil_z_pts_4_h = pivot_z - xp.sin(local_twist)*(airfoil_x_pts_2_h-pivot_x) + xp.cos(local_twist)*( airfoil_z_pts_3_h- pivot_z)
            airfoil_z_pts_4_c = pivot_z - xp.sin(local_twist)*(airfoil_x_pts_2_c-pivot_x) + xp.cos(local_twist)*( airfoil_z_pts_3_c- pivot_z)

            # the spanwise coordinate is lateral, or vertical on a vertical surface
            span_w = xp.ones(len(airfoil_x_pts_4))*delta_y
            span_c = xp.ones(len(airfoil_x_pts_4_c))*delta_y
            span_h = xp.ones(len(airfoil_x_pts_4_h))*delta_y
            if wing.vertical:
                x_w_rows.append(airfoil_x_pts_4);   y_w_rows.append(airfoil_z_pts_4);   z_w_rows.append(span_w)
                x_c_rows.append(airfoil_x_pts_4_c); y_c_rows.append(airfoil_z_pts_4_c); z_c_rows.append(span_c)
                x_h_rows.append(airfoil_x_pts_4_h); y_h_rows.append(airfoil_z_pts_4_h); z_h_rows.append(span_h)
            else:
                x_w_rows.append(airfoil_x_pts_4);   y_w_rows.append(span_w); z_w_rows.append(airfoil_z_pts_4)
                x_c_rows.append(airfoil_x_pts_4_c); y_c_rows.append(span_c); z_c_rows.append(airfoil_z_pts_4_c)
                x_h_rows.append(airfoil_x_pts_4_h); y_h_rows.append(span_h); z_h_rows.append(airfoil_z_pts_4_h)

            cs_ws.append(local_chord)

            # a station inboard of the tip starts a chordwise strip of panels
            if station.starts_strip:
                LE_inds        = np.full(n_cw, False)
                TE_inds        = np.full(n_cw, False)
                LE_inds[0]     = True
                TE_inds[-1]    = True

                RNMAX                      = np.ones(n_cw, np.int16)*n_cw
                panel_numbers              = np.linspace(1,n_cw,n_cw, dtype=np.int16)
                exposed_leading_edge_flag  = 1

                VD.panels_per_strip.append(np.ravel(RNMAX))
                VD.chordwise_panel_number.append(np.ravel(panel_numbers))
                VD.leading_edge_indices.append(np.ravel(LE_inds))
                VD.trailing_edge_indices.append(np.ravel(TE_inds))
                VD.exposed_leading_edge_flag.append(np.atleast_1d(exposed_leading_edge_flag))

        x_w, y_w, z_w = xp.stack(x_w_rows), xp.stack(y_w_rows), xp.stack(z_w_rows)
        x_c, y_c, z_c = xp.stack(x_c_rows), xp.stack(y_c_rows), xp.stack(z_c_rows)
        x_h, y_h, z_h = xp.stack(x_h_rows), xp.stack(y_h_rows), xp.stack(z_h_rows)
        cs_ws         = xp.stack([xp.asarray(chord,dtype=float) for chord in cs_ws])

        cs_w  = (cs_ws[:1] +  cs_ws[1:]) / 2

        # mirror image of the surface on the second side
        if side_idx == 1:
            side_x, side_y, side_z = yz_sym_sign, xz_sym_sign, xy_sym_sign
        else:
            side_x, side_y, side_z = 1, 1, 1

        # corners of each panel
        xa1_w = x_w[:-1,:-1]* side_x
        ya1_w = y_w[:-1,:-1]* side_y
        za1_w = z_w[:-1,:-1]* side_z
        xa2_w = x_w[:-1,1:]* side_x
        ya2_w = y_w[:-1,1:]* side_y
        za2_w = z_w[:-1,1:]* side_z
        xb1_w = x_w[1:,:-1]* side_x
        yb1_w = y_w[1:,:-1]* side_y
        zb1_w = z_w[1:,:-1]* side_z
        xb2_w = x_w[1:,1:]* side_x
        yb2_w = y_w[1:,1:]* side_y
        zb2_w = z_w[1:,1:]* side_z

        # control points at the two ends of each panel
        xac_w = x_c[:-1]* side_x
        yac_w = y_c[:-1]* side_y
        zac_w = z_c[:-1]* side_z
        xbc_w = x_c[1:]* side_x
        ybc_w = y_c[1:]* side_y
        zbc_w = z_c[1:]* side_z

        # ends of the bound vortex of each panel
        xah_w = x_h[:-1]* side_x
        yah_w = y_h[:-1]* side_y
        zah_w = z_h[:-1]* side_z
        xbh_w = x_h[1:]* side_x
        ybh_w = y_h[1:]* side_y
        zbh_w = z_h[1:]* side_z

        xch_w = (xah_w + xbh_w)/2
        ych_w = (yah_w + ybh_w)/2
        zch_w = (zah_w + zbh_w)/2

        xc_w  = (xac_w + xbc_w)/2
        yc_w  = (yac_w + ybc_w)/2
        zc_w  = (zac_w + zbc_w)/2

        wing_origin_x = wing_origin[0]* side_x
        wing_origin_y = wing_origin[1]* side_y
        wing_origin_z = wing_origin[2]* side_z

        # ---------------------------------------------------------------------------------------
        # STEP 3: Store wing in vehicle vector
        # ---------------------------------------------------------------------------------------
        n_panels = n_sw*n_cw

        # adjusting coordinate axis so reference point is at the nose of the aircraft
        VD.XAH.append(xp.ravel(xah_w) + wing_origin_x) # ends of the bound vortex
        VD.YAH.append(xp.ravel(yah_w) + wing_origin_y)
        VD.ZAH.append(xp.ravel(zah_w) + wing_origin_z)
        VD.XBH.append(xp.ravel(xbh_w) + wing_origin_x)
        VD.YBH.append(xp.ravel(ybh_w) + wing_origin_y)
        VD.ZBH.append(xp.ravel(zbh_w) + wing_origin_z)
        VD.XCH.append(xp.ravel(xch_w) + wing_origin_x) # center of the bound vortex
        VD.YCH.append(xp.ravel(ych_w) + wing_origin_y)
        VD.ZCH.append(xp.ravel(zch_w) + wing_origin_z)
        VD.XA1.append(xp.ravel(xa1_w) + wing_origin_x) # corners of the panel
        VD.YA1.append(xp.ravel(ya1_w) + wing_origin_y)
        VD.ZA1.append(xp.ravel(za1_w) + wing_origin_z)
        VD.XA2.append(xp.ravel(xa2_w) + wing_origin_x)
        VD.YA2.append(xp.ravel(ya2_w) + wing_origin_y)
        VD.ZA2.append(xp.ravel(za2_w) + wing_origin_z)
        VD.XB1.append(xp.ravel(xb1_w) + wing_origin_x)
        VD.YB1.append(xp.ravel(yb1_w) + wing_origin_y)
        VD.ZB1.append(xp.ravel(zb1_w) + wing_origin_z)
        VD.XB2.append(xp.ravel(xb2_w) + wing_origin_x)
        VD.YB2.append(xp.ravel(yb2_w) + wing_origin_y)
        VD.ZB2.append(xp.ravel(zb2_w) + wing_origin_z)
        VD.XAC.append(xp.ravel(xac_w) + wing_origin_x) # control points at the ends of the panel
        VD.YAC.append(xp.ravel(yac_w) + wing_origin_y)
        VD.ZAC.append(xp.ravel(zac_w) + wing_origin_z)
        VD.XBC.append(xp.ravel(xbc_w) + wing_origin_x)
        VD.YBC.append(xp.ravel(ybc_w) + wing_origin_y)
        VD.ZBC.append(xp.ravel(zbc_w) + wing_origin_z)
        VD.XC.append(xp.ravel(xc_w)  + wing_origin_x)  # control point of the panel
        VD.YC.append(xp.ravel(yc_w)  + wing_origin_y)
        VD.ZC.append(xp.ravel(zc_w)  + wing_origin_z)
        VD.X.append(xp.ravel(x_w) * yz_sym_sign + wing_origin_x)  # panel nodes (mirrored like the panel corners)
        VD.Y.append(xp.ravel(y_w) * xz_sym_sign + wing_origin_y)
        VD.Z.append(xp.ravel(z_w) * xy_sym_sign + wing_origin_z)
        VD.CS.append(xp.ravel(cs_w))
        VD.DY.append(xp.ravel(del_y))

        # increment number of wings and panels
        VD.n_w  += 1
        VD.horizontal_tail = np.append(VD.horizontal_tail, int(isinstance(wing, RCAIDE.Library.Components.Wings.Horizontal_Tail)))
        VD.n_cp += n_panels

        # store this wing's discretization information
        first_panel_ind  = VD.n_cp - n_panels # equal to the old VD.XAH.size, now that XAH is accumulated as a list
        first_strip_ind  = VD.chordwise_breaks.size
        chordwise_breaks = first_panel_ind + np.arange(n_panels)[0::n_cw]
        ID               = VD.counter*1

        VD.chordwise_breaks = np.append(VD.chordwise_breaks, np.int32(chordwise_breaks))
        VD.spanwise_breaks  = np.append(VD.spanwise_breaks , np.int32(first_strip_ind ))
        VD.n_sw             = np.append(VD.n_sw            , np.int16(n_sw)            )
        VD.n_cw             = np.append(VD.n_cw            , np.int16(n_cw)            )
        VD.surface_ID.append(np.ones(n_cw*n_sw)*ID*xz_sym_sign)
        VD.surface_ID_full.append(np.ones((n_cw+1)*(n_sw+1))*ID*xz_sym_sign)

        side_idx += 1

    if ( xz_sym == True) or (yz_sym == True)  or (xy_sym == True ):
        sym = 1
    else:
        sym = 0
    VD.symmetric_wings = np.append(VD.symmetric_wings, int(sym))

    if wing.vertical:
        VD.vertical_wing =  np.append(VD.vertical_wing, int(1))
    else:
        VD.vertical_wing =  np.append(VD.vertical_wing, int(0))

    area = wing.areas.projected
    VD.wing_areas.append(area/(sym+1))
    if sym:
        VD.wing_areas.append(area/(sym+1))

    return VD


def wing_lattice_inputs(wing):
    """The numbers of a wing that its lattice is computed from: a JAX value among them traces the lattice on JAX."""
    inputs = [wing.spans.projected, wing.chords.root, wing.areas.projected, wing.origin[0][0], wing.origin[0][1], wing.origin[0][2]]
    for segment in wing.segments:
        inputs += [segment.root_chord_percent, segment.twist, segment.dihedral_outboard, segment.sweeps.leading_edge,
                   segment.origin[0][0], segment.origin[0][1], segment.origin[0][2]]
    for control_surface in wing.control_surfaces:
        inputs += [control_surface.deflection, control_surface.get('secondary_deflection', 0.)]
    return inputs


def wing_lattice_layout(wing,n_cw,n_sw,spc):
    """ What fixes the structure of a wing's lattice, found from the wing's present numbers (NumPy only).

    Inputs:
    wing      wing with at least two segments
    n_cw      number of chordwise panels
    n_sw      number of spanwise panels
    spc       True for cosine spanwise spacing

    Outputs:
    layout.stations   one entry per spanwise station:
       segment                        index of the wing segment inboard of the station
       airfoil_x, airfoil_z           camber line of the airfoil at the station, per unit chord
       leading_edge_surface           slat at the station, or None
       trailing_edge_surface          flap, aileron, elevator, rudder or mixed surface at the station, or None
       leading_edge_chord_fraction    chord fraction of the leading-edge surface
       trailing_edge_chord_fraction   chord fraction of the trailing-edge surface
       ahead_of_leading_hinge         True for the airfoil points aft of the slat hinge (they do not move)
       behind_trailing_hinge          True for the airfoil points ahead of the trailing-edge hinge (they do not move)
       starts_strip                   False for the station at the outboard end of a segment's last panel
    """
    span = wing.spans.projected
    if wing.xz_plane_symmetric is True:
        span = span/2

    if spc == True:
        n               = np.linspace(n_sw+1,0,n_sw+1)
        thetan          = n*(np.pi/2)/(n_sw+1)
        y_coordinates   = span*np.cos(thetan)
    else:
        y_coordinates   = np.linspace(0,span,n_sw+1)

    wing_seg_tags = list(wing.segments.keys())
    airfoils      = {}
    layout        = Data()
    layout.stations = []
    seg_idx       = 0
    for y_i in range(len(y_coordinates)):
        delta_y = y_coordinates[y_i]
        eta_y   = delta_y / span

        # determine the segment of the station
        inboard_segment  = wing.segments[wing_seg_tags[seg_idx]]
        outboard_segment = wing.segments[wing_seg_tags[seg_idx+1]]
        try:
            if delta_y == outboard_segment.percent_span_location * span:
                pass
            else:
                while delta_y >= outboard_segment.percent_span_location * span:
                    seg_idx += 1
                    inboard_segment = wing.segments[wing_seg_tags[seg_idx]]
                    outboard_segment = wing.segments[wing_seg_tags[seg_idx+1]]
        except:
            raise AssertionError("Insufficient surface panelization discretization. Increase number of spanwise and chordwise vortices.")

        segment_root_chord    = wing.chords.root*inboard_segment.root_chord_percent
        segment_tip_chord     = wing.chords.root*outboard_segment.root_chord_percent
        inboard_segment_y     = inboard_segment.percent_span_location * span
        outboard_segment_y    = outboard_segment.percent_span_location * span
        segment_span          = outboard_segment_y -  inboard_segment_y
        local_y_val           = delta_y - inboard_segment_y
        local_percent_y_val   = local_y_val /segment_span
        local_chord           = segment_root_chord -  (segment_root_chord - segment_tip_chord) *  local_y_val / segment_span

        # leading edge sweep of the segment from its quarter-chord sweep
        if inboard_segment.sweeps.leading_edge is None:
            old_sweep                 = inboard_segment.sweeps.quarter_chord
            new_sweep                 = convert_sweep_segments(old_sweep, inboard_segment, outboard_segment, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=0.0)
            inboard_segment.sweeps.leading_edge = new_sweep

        # camber lines between the two airfoils of the segment, computed once for the segment
        if seg_idx not in airfoils:
            airfoils[seg_idx] = interpolated_airfoil_points(inboard_segment, outboard_segment, n_cw+1)
        airfoil_x, airfoil_z = airfoil_points_at(airfoils[seg_idx], local_percent_y_val, n_cw+1)

        # control surfaces at the station
        station                              = Data()
        station.segment                      = seg_idx
        station.airfoil_x                    = airfoil_x
        station.airfoil_z                    = airfoil_z
        station.leading_edge_surface         = None
        station.trailing_edge_surface        = None
        station.leading_edge_chord_fraction  = 0.0
        station.trailing_edge_chord_fraction = 0.0
        LE_angle = 0.0
        TE_angle = 0.0
        for cs in wing.control_surfaces:
            if (eta_y >= cs.span_fraction_start) and  (eta_y <= cs.span_fraction_end):
                if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler:
                    # spoilers are captured by the empirical high-lift model; no geometric panel deflection
                    continue
                elif type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:
                    station.leading_edge_surface        = cs
                    station.leading_edge_chord_fraction = cs.chord_fraction
                    LE_angle                            = cs.deflection
                else:
                    station.trailing_edge_surface        = cs
                    station.trailing_edge_chord_fraction = cs.chord_fraction
                    TE_angle                             = cs.deflection + cs.get('secondary_deflection', 0.)

        # airfoil points that the control surfaces do not move
        airfoil_x_pts = airfoil_x*local_chord
        chord         = airfoil_x_pts[-1]
        LE_chord_loc  = chord*station.leading_edge_chord_fraction
        TE_chord_loc  = chord*(1-station.trailing_edge_chord_fraction)
        station.ahead_of_leading_hinge = airfoil_x_pts > LE_chord_loc
        station.behind_trailing_hinge  = airfoil_x_pts < TE_chord_loc
        if np.all(station.ahead_of_leading_hinge | ((LE_chord_loc - airfoil_x_pts) == 0)) and (LE_angle != 0):
            print("Warning: Control surface deflection angle is non-zero but control surface is not actually deflecting any points. \n Check control surface chord fraction and airfoil discretization.")
        if np.all(station.behind_trailing_hinge | ((airfoil_x_pts - TE_chord_loc) == 0)) and (TE_angle != 0):
            print("Warning: Control surface deflection angle is non-zero but control surface is not actually deflecting any points. \n Check control surface chord fraction and airfoil discretization.")

        station.starts_strip = bool(delta_y != outboard_segment.percent_span_location * span)
        layout.stations.append(station)
    return layout


def interpolated_airfoil_points(inboard_segment,outboard_segment,ncpts):
        """ Camber lines interpolated between the airfoils of two wing segments.

        Assumptions:
        Linear geometric transition between airfoils

        Inputs:
        inboard_segment, outboard_segment    wing segments with an airfoil, or none for a flat plate
        ncpts                                number of chordwise points

        Outputs:
        x_vals, cambers                      (ncpts stations, ncpts points) chordwise locations and camber, per unit chord
        """
        if inboard_segment.airfoil:
            if type(inboard_segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                a_geo_1 = compute_naca_4series(inboard_segment.airfoil.NACA_4_Series_code,ncpts*2-1, thickness_multiplier = inboard_segment.airfoil.thickness_multiplier)
            else:
                a_geo_1 = import_airfoil_geometry(inboard_segment.airfoil.coordinate_file,ncpts*2+1, thickness_multiplier = inboard_segment.airfoil.thickness_multiplier)
        else:
            a_geo_1 =  Data()
            a_geo_1.camber_coordinates = np.zeros(ncpts)
            a_geo_1.x_upper_surface    = np.linspace(0,1,ncpts)

        if outboard_segment.airfoil:
            if type(outboard_segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                a_geo_2 = compute_naca_4series(outboard_segment.airfoil.NACA_4_Series_code,ncpts*2-1, thickness_multiplier = outboard_segment.airfoil.thickness_multiplier)
            else:
                a_geo_2 = import_airfoil_geometry(outboard_segment.airfoil.coordinate_file,ncpts*2+1, thickness_multiplier = outboard_segment.airfoil.thickness_multiplier)
        else:
            a_geo_2 =  Data()
            a_geo_2.camber_coordinates = np.zeros(ncpts)
            a_geo_2.x_upper_surface    = np.linspace(0,1,ncpts)

        # linearly interpolate between the two airfoil surfaces
        z = np.linspace(0,1,ncpts)

        y_u_lb = a_geo_1.camber_coordinates
        y_u_ub = a_geo_2.camber_coordinates
        x_u_lb = a_geo_1.x_upper_surface
        x_u_ub = a_geo_2.x_upper_surface

        cambers = (z[None,...] * (y_u_ub[...,None] - y_u_lb[...,None]) + (y_u_lb[...,None])).T
        x_vals  = (z[None,...] * (x_u_ub[...,None] - x_u_lb[...,None]) + (x_u_lb[...,None])).T
        return x_vals, cambers


def airfoil_points_at(airfoil_points,local_percent_y_val,ncpts):
    """The camber line at a fraction of the span of a segment, from interpolated_airfoil_points."""
    x_vals, cambers = airfoil_points
    idx = int(local_percent_y_val *ncpts)
    if idx == len(cambers):
        idx -= 1
    return x_vals[idx], cambers[idx]


def apply_control_surface_deflections(airfoil_x_pts, airfoil_z_pts,LE_angle, LE_chord_fraction, TE_angle, TE_chord_fraction,
                                      ahead_of_leading_hinge, behind_trailing_hinge):
    """ Moves the camber line of an airfoil with the deflection of a leading-edge and a trailing-edge control surface.

    Assumptions:
    The points are rotated about a hinge at the chord fraction of each surface

    Inputs:
    airfoil_x_pts, airfoil_z_pts   camber line                                              [m]
    LE_angle, TE_angle             deflection of the leading and trailing edge surface       [rad]
    LE_chord_fraction              chord fraction of the leading-edge surface                [-]
    TE_chord_fraction              chord fraction of the trailing-edge surface               [-]
    ahead_of_leading_hinge         True for the points the leading-edge surface does not move
    behind_trailing_hinge          True for the points the trailing-edge surface does not move

    Outputs:
    airfoil_x_pts_defl, airfoil_z_pts_defl   deflected camber line                           [m]
    """
    xp    = array_namespace(airfoil_x_pts, airfoil_z_pts, LE_angle, TE_angle)
    chord = airfoil_x_pts[-1]

    # determine the location of the hinge
    LE_chord_loc = chord*LE_chord_fraction
    TE_chord_loc = chord*(1-TE_chord_fraction)

    # distance from each point to the hinge
    LE_hinge_distance = xp.where(ahead_of_leading_hinge, 0., LE_chord_loc - airfoil_x_pts)
    TE_hinge_distance = xp.where(behind_trailing_hinge , 0., airfoil_x_pts - TE_chord_loc)

    # compute the x and z coordinate changes due to deflection
    LE_x_deflection = xp.where(ahead_of_leading_hinge, 0.,  (1 - xp.cos(LE_angle))*LE_hinge_distance)
    TE_x_deflection = xp.where(behind_trailing_hinge , 0., -(1 - xp.cos(TE_angle))*TE_hinge_distance)

    LE_z_deflection = xp.where(ahead_of_leading_hinge, 0., -xp.sin(LE_angle)*LE_hinge_distance)
    TE_z_deflection = xp.where(behind_trailing_hinge , 0., -xp.sin(TE_angle)*TE_hinge_distance)

    Total_CS_z_deflection = LE_z_deflection + TE_z_deflection
    Total_CS_x_deflection = LE_x_deflection + TE_x_deflection

    # apply the changes
    airfoil_x_pts_defl = airfoil_x_pts + Total_CS_x_deflection
    airfoil_z_pts_defl = airfoil_z_pts + Total_CS_z_deflection

    return airfoil_x_pts_defl, airfoil_z_pts_defl
