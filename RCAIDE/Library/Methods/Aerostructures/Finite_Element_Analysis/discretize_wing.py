# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/discretize_wing.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.postprocess_vortex_distribution import compute_panel_area, compute_unit_normal

def discretize_wing(wing, num_elements):
    """
    Translates RCAIDE wing geometry into high-resolution FEA nodes.
    """
     
    symmetric = wing.xz_plane_symmetric
    semi_span = wing.spans.projected / (1 + symmetric) 
    
    seg_list = list(wing.segments.keys())
    seg_elements = np.zeros(len(wing.segments) - 1)
    for seg_i in range(len(wing.segments) - 1):
        inboard_seg  =  wing.segments[seg_list[seg_i]]
        outboard_seg = wing.segments[seg_list[seg_i + 1]]

        if seg_i == 0:
            seg_span =  (outboard_seg.percent_span_location - (inboard_seg.percent_span_location + wing.percent_span_unexposed)) * semi_span
        else:
            seg_span  = (outboard_seg.percent_span_location - inboard_seg.percent_span_location) * semi_span
        seg_elements[seg_i] = ((seg_span / semi_span) * num_elements)

    wing_t_c = wing.thickness_to_chord

    seg_elements  = np.round(seg_elements).astype(int)
    # Correct rounding error so total is always exactly num_elements
    seg_elements[-1] += num_elements - int(np.sum(seg_elements))
    X_nodes         = np.empty((1, 0))
    Y_nodes         = np.empty((1, 0))
    Z_nodes         = np.empty((1, 0))
    spar_f_nodes    = np.empty((1, 0))
    spar_r_nodes    = np.empty((1, 0))
    chord_nodes     = np.empty((1, 0))
    twist_nodes     = np.empty((1, 0))
    t_c_nodes       = np.empty((1, 0))
    sweep_nodes     = np.empty((1, 0))
    dihedral_nodes  = np.empty((1, 0))
    
    
    # account for any offset from the root
    # X_0/Z_0 are computed along the spar path (not the leading edge) to stay
    # consistent with how the loop advances positions using spar_sweep.
    first_inboard_seg  = wing.segments[seg_list[0]]
    next_seg           = wing.segments[seg_list[1]]
    dih_0              = first_inboard_seg.dihedral_outboard
    mid_spar_0         = (first_inboard_seg.structural.front_spar_percent_chord + first_inboard_seg.structural.rear_spar_percent_chord) / 2
    mid_spar_1         = (next_seg.structural.front_spar_percent_chord          + next_seg.structural.rear_spar_percent_chord)          / 2
    inboard_x_ea       = first_inboard_seg.origin[0][0] + mid_spar_0 * first_inboard_seg.root_chord_percent * wing.chords.root
    inboard_y_ea       = first_inboard_seg.origin[0][1]
    outboard_x_ea      = next_seg.origin[0][0]          + mid_spar_1 * next_seg.root_chord_percent          * wing.chords.root
    outboard_y_ea      = next_seg.origin[0][1]
    spar_sweep_0       = np.arctan((outboard_x_ea - inboard_x_ea) / (outboard_y_ea - inboard_y_ea))
    del_y              = wing.percent_span_unexposed * semi_span
    X_0 = inboard_x_ea + del_y * np.tan(spar_sweep_0)
    Y_0 = inboard_y_ea + del_y
    Z_0 = del_y * np.tan(dih_0) / np.cos(spar_sweep_0)
      
    for i in range(len(wing.segments)-1):
         
        # current segment 
        inboard_seg  = wing.segments[seg_list[i]]
        outboard_seg = wing.segments[seg_list[i + 1]]
        
        # number of elements 
        n_elem  = int(seg_elements[i])
        
        # number of nodes 
        n_nodes = n_elem + 1
        
        # Interpolate the spars
        front_spar_pts = np.linspace(inboard_seg.structural.front_spar_percent_chord, outboard_seg.structural.front_spar_percent_chord, n_nodes)
        rear_spar_pts  = np.linspace(inboard_seg.structural.rear_spar_percent_chord, outboard_seg.structural.rear_spar_percent_chord, n_nodes)
        
        # Calculate Sweep of the Elastic Axis (mid-spar) using physical distances
        mid_spar_in   = (inboard_seg.structural.front_spar_percent_chord  + inboard_seg.structural.rear_spar_percent_chord)  / 2
        mid_spar_out  = (outboard_seg.structural.front_spar_percent_chord + outboard_seg.structural.rear_spar_percent_chord) / 2
        inboard_x_ea  = inboard_seg.origin[0][0]  + mid_spar_in  * inboard_seg.root_chord_percent  * wing.chords.root
        inboard_y_ea  = inboard_seg.origin[0][1]
        outboard_x_ea = outboard_seg.origin[0][0] + mid_spar_out * outboard_seg.root_chord_percent * wing.chords.root
        outboard_y_ea = outboard_seg.origin[0][1]

        spar_sweep = np.arctan((outboard_x_ea - inboard_x_ea) / (outboard_y_ea - inboard_y_ea))
        dihedral_rad  = inboard_seg.dihedral_outboard  
        
        # Calculate True Spar Length for this segment
        if i == 0:
            seg_span = (outboard_seg.percent_span_location - (inboard_seg.percent_span_location + wing.percent_span_unexposed)) * semi_span
            L_spar   = (seg_span / np.cos(spar_sweep)) / np.cos(dihedral_rad)
           
            c_diff          =  outboard_seg.root_chord_percent*wing.chords.root - inboard_seg.root_chord_percent*wing.chords.root
            y_local_non_dim =  wing.percent_span_unexposed * semi_span / seg_span
            c_root          = inboard_seg.root_chord_percent*wing.chords.root + c_diff * y_local_non_dim
            tw_start        = inboard_seg.twist + (outboard_seg.twist - inboard_seg.twist) * y_local_non_dim
            t_c_in          = inboard_seg.thickness_to_chord  if inboard_seg.thickness_to_chord  > 0 else wing_t_c
            t_c_out         = outboard_seg.thickness_to_chord if outboard_seg.thickness_to_chord > 0 else wing_t_c
            t_c_start       = t_c_in + (t_c_out - t_c_in) * y_local_non_dim

            # Local 1D arrays
            y_local = np.linspace(0, L_spar, n_nodes)
            c_arr   = np.linspace(c_root,    outboard_seg.root_chord_percent*wing.chords.root, n_nodes)
            tw_arr  = np.linspace(tw_start,  outboard_seg.twist,                               n_nodes)
            t_c_arr = np.linspace(t_c_start, t_c_out,                                          n_nodes)
        else:
            seg_span = (outboard_seg.percent_span_location - inboard_seg.percent_span_location) * semi_span
            L_spar = (seg_span / np.cos(spar_sweep)) / np.cos(dihedral_rad)
        
            # Local 1D arrays
            t_c_in  = inboard_seg.thickness_to_chord  if inboard_seg.thickness_to_chord  > 0 else wing_t_c
            t_c_out = outboard_seg.thickness_to_chord if outboard_seg.thickness_to_chord > 0 else wing_t_c
            y_local = np.linspace(0, L_spar, n_nodes)
            c_arr   = np.linspace(inboard_seg.root_chord_percent*wing.chords.root, outboard_seg.root_chord_percent*wing.chords.root, n_nodes)
            tw_arr  = np.linspace(inboard_seg.twist, outboard_seg.twist, n_nodes)
            t_c_arr = np.linspace(t_c_in, t_c_out, n_nodes)
        
        # Transform local spar distance into Global X, Y, Z
        # We start from the exact (X,Y,Z) where the last segment ended
        local_pts_x = X_0 +  y_local * np.sin(spar_sweep) * np.cos(dihedral_rad)
        local_pts_y = Y_0 +  y_local * np.cos(spar_sweep) * np.cos(dihedral_rad)
        local_pts_z = Z_0 +  y_local * np.sin(dihedral_rad)
         
        X_nodes = np.hstack((X_nodes,np.atleast_2d(local_pts_x)))    
        Y_nodes = np.hstack((Y_nodes,np.atleast_2d(local_pts_y)))   
        Z_nodes = np.hstack((Z_nodes,np.atleast_2d(local_pts_z)))
          
        spar_f_nodes = np.hstack(( spar_f_nodes, np.atleast_2d(front_spar_pts)))
        spar_r_nodes = np.hstack(( spar_r_nodes, np.atleast_2d(rear_spar_pts)))
        chord_nodes  = np.hstack(( chord_nodes , np.atleast_2d(c_arr)))
        twist_nodes  = np.hstack(( twist_nodes , np.atleast_2d(tw_arr)))
        t_c_nodes    = np.hstack(( t_c_nodes , np.atleast_2d(t_c_arr)))
        # Store element-wise angles for the rotation matrices later 

        # only add sweep and dihedral nodes for the last segment to avoid duplicates at segment boundaries
        if i+1 == len(wing.segments)-1: 
            sweep_nodes     = np.hstack((sweep_nodes ,np.ones((1, n_nodes))*spar_sweep))
            dihedral_nodes  = np.hstack((dihedral_nodes  ,np.ones((1, n_nodes))*dihedral_rad ))
        else: 
            sweep_nodes     = np.hstack((sweep_nodes ,np.ones((1, n_nodes))*spar_sweep))
            dihedral_nodes  = np.hstack((dihedral_nodes  ,np.ones((1, n_nodes))*dihedral_rad ))

            # remove last node 
            sweep_nodes  = sweep_nodes[:, :-1]
            dihedral_nodes = dihedral_nodes[:, :-1]
            X_0 = X_nodes[:,-1]
            Y_0 = Y_nodes[:,-1]
            Z_0 = Z_nodes[:,-1]
            
            X_nodes = X_nodes[:, :-1]
            Y_nodes = Y_nodes[:, :-1]
            Z_nodes = Z_nodes[:, :-1]

            spar_f_nodes = spar_f_nodes[:, :-1]       
            spar_r_nodes = spar_r_nodes[:, :-1]
            chord_nodes  = chord_nodes[:, :-1]  
            twist_nodes  = twist_nodes[:, :-1]  
            t_c_nodes    = t_c_nodes[:, :-1]  

    sweep_mid_elems = (sweep_nodes[0,  :-1]    +  sweep_nodes[0, 1:]    ) /2     
    dihedral_elems  = (dihedral_nodes[0,  :-1] +  dihedral_nodes[0, 1:] ) /2     
    twist_elems     = (twist_nodes[0,  :-1]    +  twist_nodes[0, 1:]    ) /2
         
    geom = Data( 
        X_nodes         = X_nodes[0],
        Y_nodes         = Y_nodes[0],
        Z_nodes         = Z_nodes[0],
        spar_f_nodes    = spar_f_nodes[0], 
        spar_r_nodes    = spar_r_nodes[0], 
        chord_nodes     = chord_nodes[0],
        sweep_nodes     = sweep_nodes[0],  
        twist_nodes     = twist_nodes[0], 
        t_c_nodes       = t_c_nodes[0], 
        sweep_mid_elems = sweep_mid_elems,
        dihedral_elems  = dihedral_elems,
        twist_elems     = twist_elems,
        total_span      = semi_span) 
    
    X_nodes, Y_nodes, Z_nodes = geom.X_nodes, geom.Y_nodes, geom.Z_nodes
    Le      = np.sqrt(np.diff(X_nodes)**2 + np.diff(Y_nodes)**2 + np.diff(Z_nodes)**2)
    Le      = np.maximum(Le, 1e-6)
    X_elems = (X_nodes[:-1] + X_nodes[1:]) / 2
    Y_elems = (Y_nodes[:-1] + Y_nodes[1:]) / 2
    Z_elems = (Z_nodes[:-1] + Z_nodes[1:]) / 2
    
    # Calculate element-centered arrays
    chord_elems     = (geom.chord_nodes[:-1]  + geom.chord_nodes[1:])  / 2
    spar_f_elems    = (geom.spar_f_nodes[:-1] + geom.spar_f_nodes[1:]) / 2
    spar_r_elems    = (geom.spar_r_nodes[:-1] + geom.spar_r_nodes[1:]) / 2
    t_c_elems       = (geom.t_c_nodes[:-1]    + geom.t_c_nodes[1:])    / 2 
    y_local_path    = np.insert(np.cumsum(Le), 0, 0.0)
    t_elems         = chord_elems*t_c_elems
     
    # Append additional variables 
    geom.Le            = Le
    geom.X_elems       = X_elems
    geom.Y_elems       = Y_elems
    geom.Z_elems       = Z_elems
    geom.chord_elems   = chord_elems
    geom.spar_f_elems  = spar_f_elems
    geom.spar_r_elems  = spar_r_elems
    geom.t_c_elems     = t_c_elems
    geom.y_local       = y_local_path 
    geom.t_elems       = t_elems
    
    return geom
 
def map_panel_forces_to_fea(vlm_pts, vlm_F, fea_pts):
    """
    Translates 3D VLM panel forces onto 1D FEA beam elements using
    Rigid Link Equivalent Force/Moment transfer.
    Forces are distributed linearly between the two FEA nodes that bracket
    each VLM panel in the spanwise (Y) direction.
    """
    num_fea     = len(fea_pts)
    fea_forces  = np.zeros((num_fea, 3))
    fea_moments = np.zeros((num_fea, 3))
    Y_fea       = fea_pts[:, 1]

    for i in range(len(vlm_pts)):
        p_vlm = vlm_pts[i]
        f_vlm = vlm_F[i]
        y_p   = p_vlm[1]

        # Find the right-hand bracketing node; clamp so we always have a valid pair
        idx_r = int(np.searchsorted(Y_fea, y_p))
        idx_r = np.clip(idx_r, 1, num_fea - 1)
        idx_l = idx_r - 1

        # Linear interpolation weights
        dy  = Y_fea[idx_r] - Y_fea[idx_l]
        w_r = (y_p - Y_fea[idx_l]) / dy if dy > 1e-12 else 0.5
        w_l = 1.0 - w_r

        # Distribute force
        fea_forces[idx_l] += w_l * f_vlm
        fea_forces[idx_r] += w_r * f_vlm

        # Moment arm from each node to the load application point
        fea_moments[idx_l] += w_l * np.cross(p_vlm - fea_pts[idx_l], f_vlm)
        fea_moments[idx_r] += w_r * np.cross(p_vlm - fea_pts[idx_r], f_vlm)

    return fea_forces, fea_moments


def prepare_deflection_row_mapping(row_Y, fea_pts):
    """
    One-time (per jig shape) half of the structure-to-aero kinematic transfer:
    for a set of spanwise station Y-values, bracket each between the two nearest
    FEA nodes and cache everything that depends only on the undeformed geometry
    (bracket indices, interpolation weights, the elastic-axis position, and the
    beam's local tangent direction). None of this changes between aeroelastic
    iterations -- only the FEA deflection/twist values looked up through it do --
    so it is computed once and reused every iteration via apply_deflection_row_mapping.

    Args:
        row_Y   : (R,) spanwise Y-value of each station (e.g. one per VLM mesh row)
        fea_pts : (M,3) undeformed FEA node positions, ordered along the beam
                  (Y must be monotonic in the array's own order)

    Returns:
        Data(idx_l, idx_r, w_l, w_r, a0, tangent), each length R (a0/tangent (R,3))
    """
    row_Y   = np.asarray(row_Y)
    num_fea = len(fea_pts)
    Y_fea   = fea_pts[:, 1]

    idx_r = np.searchsorted(Y_fea, row_Y)
    idx_r = np.clip(idx_r, 1, num_fea - 1)
    idx_l = idx_r - 1

    dy  = Y_fea[idx_r] - Y_fea[idx_l]
    w_r = np.where(dy > 1e-12, (row_Y - Y_fea[idx_l]) / np.where(dy > 1e-12, dy, 1.0), 0.5)
    w_l = 1.0 - w_r

    a0      = w_l[:, None] * fea_pts[idx_l] + w_r[:, None] * fea_pts[idx_r]
    tangent = fea_pts[idx_r] - fea_pts[idx_l]
    tangent = tangent / np.linalg.norm(tangent, axis=1, keepdims=True)

    return Data(idx_l=idx_l, idx_r=idx_r, w_l=w_l, w_r=w_r, a0=a0, tangent=tangent)


def apply_deflection_row_mapping(mapping, pts_grid, deflection, elastic_twist):
    """
    Cheap, per-iteration half of the structure-to-aero kinematic transfer: apply a
    mapping already prepared by prepare_deflection_row_mapping to actual 3D points,
    given the current iteration's FEA deflection/elastic_twist. Every point sharing
    a row (e.g. all chordwise points of a VLM strip, which share the same spanwise
    Y in the rigid mesh) reuses that row's cached bracket/weights/tangent.

    Rotation is Rodrigues' about the beam's own local tangent (not a fixed global
    axis), which is what makes this correct automatically for a beam running in
    either spanwise direction (e.g. the mirrored half of an xz-symmetric wing) --
    a rotation is a pseudovector, so a fixed-axis assumption would get the mirrored
    side's chirality wrong, while rotating about the true local tangent self-corrects.

    Args:
        mapping       : Data from prepare_deflection_row_mapping, length R
        pts_grid      : (R, C, 3) undeformed point positions, C points per row
        deflection    : (M,3) FEA nodal translation (dx,dy,dz)
        elastic_twist : (M,)  FEA nodal rotation about the local beam tangent

    Returns:
        (R, C, 3) deformed point positions
    """
    idx_l, idx_r = mapping.idx_l, mapping.idx_r
    w_l, w_r     = mapping.w_l[:, None], mapping.w_r[:, None]

    trans = w_l * deflection[idx_l]    + w_r * deflection[idx_r]                  # (R,3)
    theta = w_l[:, 0] * elastic_twist[idx_l] + w_r[:, 0] * elastic_twist[idx_r]    # (R,)

    a0      = mapping.a0[:, None, :]        # (R,1,3), broadcasts over C
    tangent = mapping.tangent[:, None, :]   # (R,1,3)
    trans   = trans[:, None, :]             # (R,1,3)
    cos_t   = np.cos(theta)[:, None, None]  # (R,1,1)
    sin_t   = np.sin(theta)[:, None, None]

    r   = pts_grid - a0                                    # (R,C,3)
    dot = np.sum(tangent * r, axis=-1, keepdims=True)       # (R,C,1)
    r_rot = r * cos_t + np.cross(tangent, r) * sin_t + tangent * dot * (1.0 - cos_t)

    return a0 + trans + r_rot


def apply_structural_deflection_to_vd(pts, fea_pts, deflection, elastic_twist):
    """
    Point-wise convenience wrapper around prepare/apply_deflection_row_mapping,
    for arbitrary (unstructured) points that don't share rows -- see those two
    functions for the batched form used across a VLM mesh's point-sets.

    Args:
        pts           : (N,3) point positions in the undeformed (jig) shape
        fea_pts       : (M,3) undeformed FEA node positions, ordered along the beam
        deflection    : (M,3) FEA nodal translation (dx,dy,dz)
        elastic_twist : (M,)  FEA nodal rotation about the local beam tangent

    Returns:
        (N,3) deformed point positions
    """
    pts     = np.atleast_2d(pts)
    mapping = prepare_deflection_row_mapping(pts[:, 1], fea_pts)
    return apply_deflection_row_mapping(mapping, pts[:, None, :], deflection, elastic_twist)[:, 0, :]


def _mirror_fea_reference(fea_pts, deflection, elastic_twist):
    """
    Build the FEA reference arrays for an xz-symmetric wing's mirrored half from
    the solved (positive) half: reverse node order and negate Y (a true vector,
    both for node position and the deflection's Y-component) so Y stays monotonic
    increasing in the mirrored array; elastic_twist is left unchanged (a rotation
    about the local tangent is a pseudovector -- reversing the node order already
    flips the tangent direction, which is what correctly flips the physical sense
    of the rotation; negating the angle itself would double-flip it).
    """
    fea_pts_m       = fea_pts[::-1].copy()
    fea_pts_m[:, 1] *= -1
    deflection_m    = deflection[::-1].copy()
    deflection_m[:, 1] *= -1
    elastic_twist_m = elastic_twist[::-1].copy()
    return fea_pts_m, deflection_m, elastic_twist_m


def deform_vortex_distribution(VD, geometry, structural_results, ti=0):
    """
    Apply each wing's current FEA deflection/elastic_twist to every point-set in a
    jig-shape (rigid) vortex distribution, returning a deformed copy. This is the
    structure-to-aero half of the two-way aeroelastic coupling loop: it does not
    recompute the mesh from wing geometry (that stays fixed, the jig shape), it
    only rigidly displaces/rotates the existing points -- see
    prepare_/apply_deflection_row_mapping for the per-point transform.

    Point-sets sharing a spanwise row (confirmed against generate_wing_vortex_
    distribution.py) are transformed together via one shared row mapping rather
    than independently:
      - "A" family (row i of the raw (n_sw+1) grid): XA1, XA2, XAH, XAC
      - "B" family (row i+1):                        XB1, XB2, XBH, XBC
      - "center" family (midpoint of rows i, i+1):    XC,  XCH
      - the raw grid itself (all n_sw+1 rows):        X, Y, Z

    Symmetric (xz_plane_symmetric) wings get a second, mirrored group per the
    generation-order convention already used in FEA.py (surface_ID = +ID for the
    positive half, -ID for the mirrored half; vd_idx advances by 2 for such wings).

    Args:
        VD                  : jig-shape vortex distribution (unmodified; a deformed copy is returned)
        geometry            : vehicle (wings iterated in the same order used to build VD)
        structural_results  : Data keyed by wing.tag, each holding .structural_node_data
                               (from discretize_wing) and .deflection / .elastic_twist
        ti                  : control-point row to use from .deflection / .elastic_twist

    Returns:
        deformed copy of VD
    """
    A_FAMILY  = [('XA1','YA1','ZA1'), ('XA2','YA2','ZA2'), ('XAH','YAH','ZAH'), ('XAC','YAC','ZAC')]
    B_FAMILY  = [('XB1','YB1','ZB1'), ('XB2','YB2','ZB2'), ('XBH','YBH','ZBH'), ('XBC','YBC','ZBC')]
    C_FAMILY  = [('XC','YC','ZC'), ('XCH','YCH','ZCH')]
    RAW_GRID  = ('X','Y','Z')

    # generate_vortex_distribution() (unlike generate_aircraft_vortex_distribution())
    # wraps every field with a leading condition-batch dimension, e.g. XAH is
    # shape (n_conditions, N) not (N,). This function is only ever called with a
    # single-condition VD (one row at a time, matching how call_solvers uses it),
    # so we work on the flat [0] row and re-wrap into that same (1,N) convention
    # for whatever VLM()'s downstream code expects.
    assert VD.XAH.shape[0] == 1, "deform_vortex_distribution expects a single-condition VD"

    VD_new = Data(VD)
    flat = {}  # flat (N,) working copies, indexed by field name
    for prefix_tuple in [RAW_GRID] + A_FAMILY + B_FAMILY + C_FAMILY:
        for key in prefix_tuple:
            flat[key] = VD[key][0].copy()

    n_sw_flat = VD.n_sw[0]
    n_cw_flat = VD.n_cw[0]
    panel_sizes = n_sw_flat * n_cw_flat
    panel_offsets = np.concatenate(([0], np.cumsum(panel_sizes)))
    grid_sizes = (n_sw_flat + 1) * (n_cw_flat + 1)
    grid_offsets = np.concatenate(([0], np.cumsum(grid_sizes)))

    def stack(prefix_tuple, sl):
        x, y, z = prefix_tuple
        return np.column_stack((flat[x][sl], flat[y][sl], flat[z][sl]))

    def unstack_into(prefix_tuple, sl, pts):
        x, y, z = prefix_tuple
        flat[x][sl], flat[y][sl], flat[z][sl] = pts[:, 0], pts[:, 1], pts[:, 2]

    vd_idx = 0
    for wing in geometry.wings.values():
        sr = structural_results[wing.tag]
        node = sr.structural_node_data
        fea_pts_jig   = np.column_stack((node.X_nodes, node.Y_nodes, node.Z_nodes))
        deflection    = sr.deflection[ti]        # (n_nodes, 3)
        elastic_twist = sr.elastic_twist[ti, :, 0]  # (n_nodes,)

        groups = [(vd_idx, fea_pts_jig, deflection, elastic_twist)]
        if wing.xz_plane_symmetric:
            groups.append((vd_idx + 1, *_mirror_fea_reference(fea_pts_jig, deflection, elastic_twist)))

        for g_idx, fea_pts, defl, twist in groups:
            n_sw = int(n_sw_flat[g_idx])
            n_cw = int(n_cw_flat[g_idx])

            panel_sl = slice(int(panel_offsets[g_idx]), int(panel_offsets[g_idx + 1]))
            grid_sl  = slice(int(grid_offsets[g_idx]),  int(grid_offsets[g_idx + 1]))

            # raw grid: (n_sw+1) rows
            grid_pts = stack(RAW_GRID, grid_sl).reshape(n_sw + 1, n_cw + 1, 3)
            row_Y_raw = grid_pts[:, 0, 1]
            mapping_raw = prepare_deflection_row_mapping(row_Y_raw, fea_pts)
            grid_def = apply_deflection_row_mapping(mapping_raw, grid_pts, defl, twist)
            unstack_into(RAW_GRID, grid_sl, grid_def.reshape(-1, 3))

            mapping_A = Data(idx_l=mapping_raw.idx_l[:-1], idx_r=mapping_raw.idx_r[:-1],
                              w_l=mapping_raw.w_l[:-1],     w_r=mapping_raw.w_r[:-1],
                              a0=mapping_raw.a0[:-1],       tangent=mapping_raw.tangent[:-1])
            mapping_B = Data(idx_l=mapping_raw.idx_l[1:],  idx_r=mapping_raw.idx_r[1:],
                              w_l=mapping_raw.w_l[1:],      w_r=mapping_raw.w_r[1:],
                              a0=mapping_raw.a0[1:],        tangent=mapping_raw.tangent[1:])

            for family, mapping in ((A_FAMILY, mapping_A), (B_FAMILY, mapping_B)):
                for prefix in family:
                    pts = stack(prefix, panel_sl).reshape(n_sw, n_cw, 3)
                    pts_def = apply_deflection_row_mapping(mapping, pts, defl, twist)
                    unstack_into(prefix, panel_sl, pts_def.reshape(-1, 3))

            row_Y_mid = 0.5 * (row_Y_raw[:-1] + row_Y_raw[1:])
            mapping_C = prepare_deflection_row_mapping(row_Y_mid, fea_pts)
            for prefix in C_FAMILY:
                pts = stack(prefix, panel_sl).reshape(n_sw, n_cw, 3)
                pts_def = apply_deflection_row_mapping(mapping_C, pts, defl, twist)
                unstack_into(prefix, panel_sl, pts_def.reshape(-1, 3))

        vd_idx += 2 if wing.xz_plane_symmetric else 1

    # normals/panel_areas depend only on the (now deformed) panel corners --
    # compute_panel_area/compute_unit_normal only read XA1/XA2/XB1/XB2, so a
    # minimal flat proxy is enough; must run on flat arrays, not the (1,N) form.
    flat_corners = Data(XA1=flat['XA1'], YA1=flat['YA1'], ZA1=flat['ZA1'],
                         XA2=flat['XA2'], YA2=flat['YA2'], ZA2=flat['ZA2'],
                         XB1=flat['XB1'], YB1=flat['YB1'], ZB1=flat['ZB1'],
                         XB2=flat['XB2'], YB2=flat['YB2'], ZB2=flat['ZB2'])
    VD_new.panel_areas = np.atleast_2d(compute_panel_area(flat_corners))
    VD_new.normals     = compute_unit_normal(flat_corners)[None, :, :]

    # re-wrap into the (1,N) condition-batch convention generate_vortex_distribution uses
    for prefix_tuple in [RAW_GRID] + A_FAMILY + B_FAMILY + C_FAMILY:
        for key in prefix_tuple:
            VD_new[key] = np.atleast_2d(flat[key])

    return VD_new