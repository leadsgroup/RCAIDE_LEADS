# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_multisegment_geometry.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# Import Supporting Functions
from RCAIDE.Framework.Core   import Data
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import  convert_sweep_segments

# Python Imports
import numpy as np 

# ----------------------------------------------------------------------
# Multi Segment GEometry Function 
# ----------------------------------------------------------------------
def compute_multisegment_geometry(wing, total_elements):
    """
    Takes a wing configuration with multiple segments and outputs continuous 
    1D arrays for FEA nodes and elements.
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
        seg_elements[seg_i] = ((seg_span / semi_span) * total_elements)
 
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
      
    for i in range(len(wing.segments)-1):
         
        # current segment 
        inboard_seg =  wing.segments[seg_list[i]]
        outboard_seg = wing.segments[seg_list[i + 1]]
        
        # number of elements 
        n_elem  = int(seg_elements[i])
        
        # number of nodes 
        n_nodes = n_elem + 1
        
        # Interpolate the spars
        front_spar_pts = np.linspace(inboard_seg.structural.front_spar_percent_chord, outboard_seg.structural.front_spar_percent_chord, n_nodes)
        rear_spar_pts = np.linspace(inboard_seg.structural.rear_spar_percent_chord, outboard_seg.structural.rear_spar_percent_chord, n_nodes)
        
        # Calculate Sweep of the Elastic Axis using physical distances 
        sweep_mid_rad = convert_sweep_segments(inboard_seg.sweeps.quarter_chord, inboard_seg, outboard_seg, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=0.5)
        dihedral_rad  = inboard_seg.dihedral_outboard  
        
        # Calculate True Spar Length for this segment
        seg_span = (outboard_seg.percent_span_location - inboard_seg.percent_span_location) * semi_span
        L_spar = (seg_span / np.cos(sweep_mid_rad)) / np.cos(dihedral_rad)
        
        # Local 1D arrays
        y_local = np.linspace(0, L_spar, n_nodes)
        c_arr   = np.linspace(inboard_seg.root_chord_percent*wing.chords.root, outboard_seg.root_chord_percent*wing.chords.root, n_nodes)
        tw_arr  = np.linspace(inboard_seg.twist, outboard_seg.twist, n_nodes)
        t_c_arr = np.linspace(inboard_seg.thickness_to_chord, outboard_seg.thickness_to_chord, n_nodes)
        
        # Transform local spar distance into Global X, Y, Z
        # We start from the exact (X,Y,Z) where the last segment ended
        local_pts_x = inboard_seg.origin[0][0] +  y_local * np.sin(sweep_mid_rad) * np.cos(dihedral_rad)
        local_pts_y = inboard_seg.origin[0][1] +  y_local * np.cos(sweep_mid_rad) * np.cos(dihedral_rad)
        local_pts_z = inboard_seg.origin[0][2] +  y_local * np.sin(dihedral_rad)
         
        X_nodes = np.hstack((X_nodes,np.atleast_2d(local_pts_x)))    
        Y_nodes = np.hstack((Y_nodes,np.atleast_2d(local_pts_y)))   
        Z_nodes = np.hstack((Z_nodes,np.atleast_2d(local_pts_z)))
          
        spar_f_nodes= np.hstack(( spar_f_nodes, np.atleast_2d(front_spar_pts)))
        spar_r_nodes= np.hstack(( spar_r_nodes, np.atleast_2d(rear_spar_pts)))
        chord_nodes = np.hstack(( chord_nodes , np.atleast_2d(c_arr)))
        twist_nodes = np.hstack(( twist_nodes , np.atleast_2d(tw_arr)))
        t_c_nodes = np.hstack(( t_c_nodes , np.atleast_2d(t_c_arr)))
        # Store element-wise angles for the rotation matrices later 

        # only add sweep and dihedral nodes for the last segment to avoid duplicates at segment boundaries
        if i+1 == len(wing.segments)-1: 
            sweep_nodes     = np.hstack((sweep_nodes ,np.ones((1, n_nodes))*sweep_mid_rad))
            dihedral_nodes  = np.hstack((dihedral_nodes  ,np.ones((1, n_nodes))*dihedral_rad ))
        else: 
            sweep_nodes     = np.hstack((sweep_nodes ,np.ones((1, n_nodes))*sweep_mid_rad))
            dihedral_nodes  = np.hstack((dihedral_nodes  ,np.ones((1, n_nodes))*dihedral_rad ))

            # remove last node 
            sweep_nodes  = sweep_nodes[:, :-1]
            dihedral_nodes = dihedral_nodes[:, :-1]
            X_nodes = X_nodes[:, :-1]
            Y_nodes = Y_nodes[:, :-1]
            Z_nodes = Z_nodes[:, :-1]   
            spar_f_nodes = spar_f_nodes[:, :-1]       
            spar_r_nodes = spar_r_nodes[:, :-1]
            chord_nodes  = chord_nodes[:, :-1]  
            twist_nodes  = twist_nodes[:, :-1]  
            t_c_nodes  = t_c_nodes[:, :-1]  

    sweep_mid_elems =  (sweep_nodes[0,  :-1] +  sweep_nodes[0, 1:] ) /2     
    dihedral_elems  =   (dihedral_nodes[0,  :-1] +  dihedral_nodes[0, 1:] ) /2     
    
         
    multi_seg_points = Data( 
        X_nodes = X_nodes[0],
        Y_nodes = Y_nodes[0],
        Z_nodes = Z_nodes[0],
        spar_f_nodes = spar_f_nodes[0], 
        spar_r_nodes = spar_r_nodes[0], 
        chord_nodes  = chord_nodes[0],
        sweep_nodes  = sweep_nodes[0],  
        twist_nodes  = twist_nodes[0], 
        t_c_nodes    = t_c_nodes[0], 
        sweep_mid_elems =  sweep_mid_elems,
        dihedral_elems  =  dihedral_elems,
        total_span      =  semi_span)
        
    # 4. PACKAGE THE DATA
    return multi_seg_points