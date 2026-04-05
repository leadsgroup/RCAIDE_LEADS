# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_multisegment_geometry.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# Import Supporting Functions
import RCAIDE 

# Python Imports
import numpy as np 

# ----------------------------------------------------------------------
# Multi Segment GEometry Function 
# ----------------------------------------------------------------------
def compute_multisegment_geometry(wing_config, total_elements):
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
    
    # 3. CONTINUOUS NODE GENERATION
    # Node arrays (Size: total_elements + 1)
    X_nodes, Y_nodes, Z_nodes = [], [], []
    chord_nodes, twist_nodes_deg = [], []
    spar_f_nodes, spar_r_nodes = [], []
    
    # Element arrays (Size: total_elements)
    sweep_mid_elems, dihedral_elems = [], []
    
    current_X, current_Y, current_Z = 0.0, 0.0, 0.0
    
    for i, seg in enumerate(segments):
        n_elem = seg_elements[i]
        n_nodes = n_elem + 1
        
        # Interpolate the spars
        sf_arr = np.linspace(seg['spar_f_root'], seg['spar_f_tip'], n_nodes)
        sr_arr = np.linspace(seg['spar_r_root'], seg['spar_r_tip'], n_nodes)
        
        # Calculate Sweep of the Elastic Axis using physical distances
        mid_f_root = (seg['spar_f_root'] + seg['spar_r_root']) / 2
        mid_f_tip = (seg['spar_f_tip'] + seg['spar_r_tip']) / 2
        offset_root = seg['chord_root'] * mid_f_root
        offset_tip = seg['span'] * np.tan(np.radians(seg['sweep_LE'])) + seg['chord_tip'] * mid_f_tip
        
        # True Sweep Angle of the structural box
        tan_sw_mid = (offset_tip - offset_root) / seg['span']
        sweep_mid_rad = np.arctan(tan_sw_mid)
        dihedral_rad = np.radians(seg['dihedral'])
        
        # Calculate True Spar Length for this segment
        L_spar = (seg['span'] / np.cos(sweep_mid_rad)) / np.cos(dihedral_rad)
        
        # Local 1D arrays
        y_local = np.linspace(0, L_spar, n_nodes)
        c_arr = np.linspace(seg['chord_root'], seg['chord_tip'], n_nodes)
        tw_arr = np.linspace(seg['twist_root'], seg['twist_tip'], n_nodes)
        
        # Transform local spar distance into Global X, Y, Z
        # We start from the exact (X,Y,Z) where the last segment ended
        x_seg = current_X + y_local * np.sin(sweep_mid_rad) * np.cos(dihedral_rad)
        y_seg = current_Y + y_local * np.cos(sweep_mid_rad) * np.cos(dihedral_rad)
        z_seg = current_Z + y_local * np.sin(dihedral_rad)
        
        # Append to global lists
        if i == 0:
            # First segment: Add all nodes including the root
            X_nodes.extend(x_seg)
            Y_nodes.extend(y_seg)
            Z_nodes.extend(z_seg)
            spar_f_nodes.extend(sf_arr)
            spar_r_nodes.extend(sr_arr)
            chord_nodes.extend(c_arr)
            twist_nodes_deg.extend(tw_arr)
        else:
            # Subsequent segments: Omit the first node to avoid duplicating the junction
            X_nodes.extend(x_seg[1:])
            Y_nodes.extend(y_seg[1:])
            Z_nodes.extend(z_seg[1:])
            spar_f_nodes.extend(sf_arr[1:])
            spar_r_nodes.extend(sr_arr[1:])
            chord_nodes.extend(c_arr[1:])
            twist_nodes_deg.extend(tw_arr[1:])
            
        # Store element-wise angles for the rotation matrices later
        sweep_mid_elems.extend([sweep_mid_rad] * n_elem)
        dihedral_elems.extend([dihedral_rad] * n_elem)
        
        # Update the "pen" to start the next segment at the exact end of this one
        current_X, current_Y, current_Z = x_seg[-1], y_seg[-1], z_seg[-1]
        
    # 4. PACKAGE THE DATA
    return {
        'X_nodes': np.array(X_nodes),
        'Y_nodes': np.array(Y_nodes),
        'Z_nodes': np.array(Z_nodes),
        'spar_f_nodes': np.array(spar_f_nodes),
        'spar_r_nodes': np.array(spar_r_nodes),
        'chord_nodes': np.array(chord_nodes),
        'twist_nodes_deg': np.array(twist_nodes_deg),
        'sweep_mid_elems': np.array(sweep_mid_elems),
        'dihedral_elems': np.array(dihedral_elems),
        'total_span': total_span
    }