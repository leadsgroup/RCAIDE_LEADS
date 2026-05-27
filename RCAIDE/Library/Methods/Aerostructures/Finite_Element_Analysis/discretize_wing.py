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
    
    # Assign elements proportionally, ensuring a minimum of 5 elements per segment
    seg_elements = np.zeros(len(wing.segments) - 1)

    seg_list = list(wing.segments.keys())
    for seg_i in range(len(wing.segments) - 1):
        inboard_seg  =  wing.segments[seg_list[seg_i]]
        outboard_seg = wing.segments[seg_list[seg_i + 1]]
        seg_span     = (outboard_seg.percent_span_location - inboard_seg.percent_span_location) * semi_span
        seg_elements[seg_i] = ((seg_span / semi_span) * num_elements)
 
    seg_elements    = np.round(seg_elements)
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
    X_0 = 0
    Y_0 = 0
    Z_0 = 0
      
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
        
        # Calculate Sweep of the Elastic Axis using physical distances
        inboard_x_fs  = inboard_seg.origin[0][0] + inboard_seg.structural.front_spar_percent_chord * inboard_seg.root_chord_percent*wing.chords.root
        inboard_y_fs  = inboard_seg.origin[0][1]   
        outboard_x_fs = outboard_seg.origin[0][0] + outboard_seg.structural.front_spar_percent_chord * outboard_seg.root_chord_percent*wing.chords.root
        outboard_y_fs = outboard_seg.origin[0][1]
        
        spar_sweep = np.arctan((outboard_x_fs-inboard_x_fs) / ( outboard_y_fs - inboard_y_fs)) 
        dihedral_rad  = inboard_seg.dihedral_outboard  
        
        # Calculate True Spar Length for this segment
        seg_span = (outboard_seg.percent_span_location - inboard_seg.percent_span_location) * semi_span
        L_spar = (seg_span / np.cos(spar_sweep)) / np.cos(dihedral_rad)
        
        # Local 1D arrays
        y_local = np.linspace(0, L_spar, n_nodes)
        c_arr   = np.linspace(inboard_seg.root_chord_percent*wing.chords.root, outboard_seg.root_chord_percent*wing.chords.root, n_nodes)
        tw_arr  = np.linspace(inboard_seg.twist, outboard_seg.twist, n_nodes)
        t_c_arr = np.linspace(inboard_seg.thickness_to_chord, outboard_seg.thickness_to_chord, n_nodes)
        
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

    sweep_mid_elems =  (sweep_nodes[0,  :-1] +  sweep_nodes[0, 1:] ) /2     
    dihedral_elems  =   (dihedral_nodes[0,  :-1] +  dihedral_nodes[0, 1:] ) /2     
    
         
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
        sweep_mid_elems =  sweep_mid_elems,
        dihedral_elems  =  dihedral_elems,
        total_span      =  semi_span) 
    
    X_nodes, Y_nodes, Z_nodes = geom.X_nodes, geom.Y_nodes, geom.Z_nodes
    Le = np.sqrt(np.diff(X_nodes)**2 + np.diff(Y_nodes)**2 + np.diff(Z_nodes)**2)
    Le = np.maximum(Le, 1e-6)
    Y_elems = (Y_nodes[:-1] + Y_nodes[1:]) / 2
    
    # Calculate element-centered arrays
    chord_elems     = (geom.chord_nodes[:-1]  + geom.chord_nodes[1:]) / 2
    spar_f_elems    = (geom.spar_f_nodes[:-1] + geom.spar_f_nodes[1:]) / 2
    spar_r_elems    = (geom.spar_r_nodes[:-1] + geom.spar_r_nodes[1:]) / 2
    t_c_elems       = (geom.t_c_nodes[:-1] + geom.t_c_nodes[1:]) / 2 
    y_local_path    = np.insert(np.cumsum(Le), 0, 0.0)
     
    # Append additional variables 
    geom.Le            = Le
    geom.Y_elems       = Y_elems
    geom.chord_elems   = chord_elems
    geom.spar_f_elems  = spar_f_elems
    geom.spar_r_elems  = spar_r_elems
    geom.t_c_elems     = t_c_elems
    geom.y_local       = y_local_path 
    return geom
 
def map_panel_forces_to_fea(vlm_pts, vlm_F, fea_pts):
    """
    Translates 3D VLM panel forces onto 1D FEA beam elements using 
    Rigid Link Equivalent Force/Moment transfer.
    """
    num_fea = len(fea_pts)
    fea_forces = np.zeros((num_fea, 3))
    fea_moments = np.zeros((num_fea, 3))
    
    Y_fea = fea_pts[:, 1]
    
    for i in range(len(vlm_pts)):
        p_vlm = vlm_pts[i]
        f_vlm = vlm_F[i]
        
        # 1. Find closest FEA element along the span
        closest_idx = np.argmin(np.abs(Y_fea - p_vlm[1]))
        p_fea = fea_pts[closest_idx]
        
        # 2. Add Forces
        fea_forces[closest_idx] += f_vlm
        
        # 3. Calculate Moment Arm & Torsion (r x F)
        r = p_vlm - p_fea 
        m_equiv = np.cross(r, f_vlm)
        fea_moments[closest_idx] += m_equiv
        
    return fea_forces, fea_moments