# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/discretize_wing.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core import Data

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