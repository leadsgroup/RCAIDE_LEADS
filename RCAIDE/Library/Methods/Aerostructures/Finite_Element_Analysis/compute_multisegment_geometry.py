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
def compute_multisegment_geometry(wing, wing_config, total_elements):
    """
    Takes a wing configuration with multiple segments and outputs continuous 
    1D arrays for FEA nodes and elements.
    """
    segments = wing_config['segments']
    
    # 1. VALIDATION CHECK (Ensure C0 Continuity)
    for i in range(len(segments) - 1):
        if not np.isclose(segments[i]['chord_tip'], segments[i+1]['chord_root']):
            raise ValueError(f"Geometry Error: Discontinuity at Seg {i+1}-{i+2} boundary. Tip chord ({segments[i]['chord_tip']}m) != Root chord ({segments[i+1]['chord_root']}m).")
        if not np.isclose(segments[i]['twist_tip'], segments[i+1]['twist_root']):
            raise ValueError(f"Geometry Error: Discontinuity at Seg {i+1}-{i+2} boundary. Tip twist ({segments[i]['twist_tip']}°) != Root twist ({segments[i+1]['twist_root']}°).")
            
    # 2. PROPORTIONAL MESHING
    total_span = sum(seg['span'] for seg in segments)
    # Assign elements proportionally, ensuring a minimum of 5 elements per segment
    seg_elements = [max(5, int((seg['span'] / total_span) * total_elements)) for seg in segments]
    
    # Force the sum of elements to exactly match total_elements (absorb rounding errors)
    seg_elements[-1] += total_elements - sum(seg_elements) 

    X_nodes= np.empty((1, 0))
    Y_nodes= np.empty((1, 0))
    Z_nodes= np.empty((1, 0))
    spar_f_nodes= np.empty((1, 0))
    spar_r_nodes = np.empty((1, 0))
    chord_nodes= np.empty((1, 0))
    twist_nodes= np.empty((1, 0))
    sweep_nodes = np.empty((1, 0))
    dihedral_nodes  = np.empty((1, 0))
     
    seg_keys = list(wing.segments.keys()) 
    for i in range(len(segments)-1):
         
        # current segment
        seg             = segments[i]
        inboard_seg     = wing.segments[seg_keys[i]] 
        outboard_seg    = wing.segments[seg_keys[i+1]]   
        
        # number of elements 
        n_elem  = seg_elements[i]
        
        # number of nodes 
        n_nodes = n_elem + 1
        
        # Interpolate the spars
        front_spar_pts = np.linspace(seg['spar_f_root'], seg['spar_f_tip'], n_nodes)
        rear_spar_pts = np.linspace(seg['spar_r_root'], seg['spar_r_tip'], n_nodes)
        
        # Calculate Sweep of the Elastic Axis using physical distances 
        sweep_mid_rad = convert_sweep_segments(inboard_seg.sweeps.quarter_chord, inboard_seg, outboard_seg, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=0.5)
        dihedral_rad  = seg['dihedral']
        
        # Calculate True Spar Length for this segment
        L_spar = (seg['span'] / np.cos(sweep_mid_rad)) / np.cos(dihedral_rad)
        
        # Local 1D arrays
        y_local = np.linspace(0, L_spar, n_nodes)
        c_arr   = np.linspace(seg['chord_root'], seg['chord_tip'], n_nodes)
        tw_arr  = np.linspace(seg['twist_root'], seg['twist_tip'], n_nodes)
        
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
            
        # Store element-wise angles for the rotation matrices later 
        sweep_nodes = np.hstack((sweep_nodes ,np.ones((1, n_nodes))*sweep_mid_rad))
        dihedral_nodes  = np.hstack((dihedral_nodes  ,np.ones((1, n_nodes))*dihedral_rad ))
        

    sweep_mid_elems =  (sweep_nodes[0,  :-1] +  sweep_nodes[0, 1:] ) /2     
    dihedral_elems  =   (dihedral_nodes[0,  :-1] +  dihedral_nodes[0, 1:] ) /2     
    
         
    multi_seg_points = Data( 
        X_nodes = X_nodes[0],
        Y_nodes = Y_nodes[0],
        Z_nodes = Z_nodes[0],
        spar_f_nodes = spar_f_nodes[0], 
        spar_r_nodes = spar_r_nodes[0], 
        chord_nodes  = chord_nodes[0],  
        twist_nodes  = twist_nodes[0],  
        sweep_mid_elems =  sweep_mid_elems,
        dihedral_elems =  dihedral_elems,
        total_span =  total_span)
        
    # 4. PACKAGE THE DATA
    return multi_seg_points