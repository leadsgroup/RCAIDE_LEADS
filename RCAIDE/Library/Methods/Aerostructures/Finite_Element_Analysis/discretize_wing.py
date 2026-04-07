# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/discretize_wing.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_multisegment_geometry import compute_multisegment_geometry

def discretize_wing(wing, num_elements):
    """
    Translates RCAIDE wing geometry into high-resolution FEA nodes.
    """
    wing_config = translate_rcaide_to_config(wing)
    geom = compute_multisegment_geometry(wing_config, num_elements)
    
    X_nodes, Y_nodes, Z_nodes = geom['X_nodes'], geom['Y_nodes'], geom['Z_nodes']
    Le = np.sqrt(np.diff(X_nodes)**2 + np.diff(Y_nodes)**2 + np.diff(Z_nodes)**2)
    Le = np.maximum(Le, 1e-6)
    Y_elems = (Y_nodes[:-1] + Y_nodes[1:]) / 2
    
    # Calculate element-centered arrays
    chord_elems = (geom['chord_nodes'][:-1] + geom['chord_nodes'][1:]) / 2
    spar_f_elems = (geom['spar_f_nodes'][:-1] + geom['spar_f_nodes'][1:]) / 2
    spar_r_elems = (geom['spar_r_nodes'][:-1] + geom['spar_r_nodes'][1:]) / 2
    
    discretized_params = Data(
        X_nodes             = X_nodes,
        Y_nodes             = Y_nodes,
        Z_nodes             = Z_nodes,
        chord_nodes         = geom['chord_nodes'],
        twist_nodes_deg     = geom['twist_nodes_deg'],
        sweep_elems_rad     = geom['sweep_mid_elems'],
        dihedral_elems_rad  = geom['dihedral_elems'],
        total_span          = geom['total_span'],
        spar_f_nodes        = geom['spar_f_nodes'],
        spar_r_nodes        = geom['spar_r_nodes'],
        Le                  = Le,
        Y_elems             = Y_elems,
        chord_elems         = chord_elems,
        spar_f_elems        = spar_f_elems,
        spar_r_elems        = spar_r_elems,
        wing_config         = wing_config
    )
    return discretized_params

def translate_rcaide_to_config(wing):
    """
    Reads an RCAIDE Wing object and translates it to our dictionary config.
    """
    wing_config = {
        't_c': getattr(wing, 'thickness_to_chord', 0.121),
        'Skin_Top_Thick': 0.01026, 
        'Skin_Bot_Thick': 0.01026,
        'Rib_Spacing': 0.6,
        'Rib_Thick': 0.004,
        'Front_Spar': {'type': 'Rectangular', 't_web': 0.0065, 'w_cap': 0.0, 't_cap': 0.0},
        'Rear_Spar':  {'type': 'Rectangular', 't_web': 0.0065, 'w_cap': 0.0, 't_cap': 0.0},
        'segments': []
    }
    sym = wing.xz_plane_symmetric
    semi_span = wing.spans.projected / (1 + sym)
    segments = sorted(wing.segments.values(), key=lambda s: s.percent_span_location)
    num_segs = len(segments)

    for i, seg in enumerate(segments):
        y_root = seg.percent_span_location * semi_span
        if i < num_segs - 1:
            next_seg = segments[i+1]
            y_tip = next_seg.percent_span_location * semi_span
            chord_tip = next_seg.root_chord_percent * wing.chords.root
            twist_tip = next_seg.twist
        else:
            y_tip = semi_span
            chord_tip = wing.chords.tip
            twist_tip = getattr(wing.twists, 'tip', seg.twist) 

        seg_dict = {
            'span': y_tip - y_root,
            'sweep_LE': np.degrees(getattr(seg.sweeps, 'leading_edge', wing.sweeps.leading_edge)),
            'dihedral': np.degrees(getattr(seg, 'dihedral_outboard', wing.dihedral)),
            'chord_root': seg.root_chord_percent * wing.chords.root,
            'chord_tip': chord_tip,
            'twist_root': np.degrees(seg.twist),
            'twist_tip': np.degrees(twist_tip),
            'spar_f_root': getattr(seg, 'front_spar_fraction', 0.15),
            'spar_f_tip':  getattr(seg, 'front_spar_fraction', 0.15),
            'spar_r_root': getattr(seg, 'rear_spar_fraction', 0.65),
            'spar_r_tip':  getattr(seg, 'rear_spar_fraction', 0.65)
        }
        wing_config['segments'].append(seg_dict)

    return wing_config

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