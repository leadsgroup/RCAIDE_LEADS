
from RCAIDE.Framework.Core import Data

from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_multisegment_geometry        import compute_multisegment_geometry
import numpy as np


def discretize_wing(wing,num_elements):




   # Step 1: unpack wing properties 
    num_segs = len(wing.segments)
    if num_segs > 0:
        h_arr           = np.zeros(num_segs)
        chord_arr       = np.zeros(num_segs)
        span_locations  =  np.zeros(num_segs)
        twist_arr       =  np.zeros(num_segs)
        front_spar_locs = np.zeros(num_segs)
        for seg_i , seg in enumerate(wing.segments):
            chord_arr[seg_i]       = seg.percent_root_chord* wing.chords.root
            h_arr[seg_i]           = seg.thickness_to_chord * chord_arr[seg_i]
            span_locations[seg_i]  = seg.percent_span_location * wing.spans.projected
            twist_arr[seg_i]       = seg.twist 
            front_spar_locs[seg_i] = seg.front_spar_locatation[0] 
    else:
        h_arr          = np.zeros(2)
        chord_arr      = np.zeros(2)
        span_locations = np.zeros(2)
        twist_arr      = np.zeros(2)
        front_spar_locs = np.zeros(2)


        chord_arr[0] = wing.chords.root
        chord_arr[1] = wing.chords.tip

        h_arr[0] = wing.thickness_to_chord *chord_arr[0] 
        h_arr[1] = wing.thickness_to_chord *chord_arr[1] 

        span_locations[0] = 0 
        span_locations[0] = wing.spans.projected
        twist_arr[0]       = wing.twists.root
        twist_arr[1]       = wing.twists.tip

        front_spar_locs[0] = wing.front_spar_locations.root 
        front_spar_locs[1] = wing.front_spar_percent_chords.tip  
     
     
    # Step 2: Rediscretize wing properties by interpolating  
    discretized_span       = np.linspace(0, wing.spans.projected,num_elements)     

    # interpolation 
    discretized_chords = np.interp_1d(chord_arr,span_locations,discretized_span)
    discretized_twist  = np.interp_1d(twist_arr,span_locations,discretized_span)
    discretized_front_spar_locs   = np.interp_1d(front_spar_locs,span_locations,discretized_span)
 



    # # We need the element centers for calculating loads and stiffness 
    # y_elem_centers                  = np.cumsum(Le) - (Le / 2) # Distance along elastic axis 
    # spar_r_elems                    = (spar_r_nodes[:-1] + spar_r_nodes[1:]) / 2



    # # Calculate Element Length (Le) For Each Element
    # Le                              = np.sqrt(np.diff(X_nodes)**2 + np.diff(Y_nodes)**2 + np.diff(Z_nodes)**2)
    

        
    # Generate the geometry
    geom                            = compute_multisegment_geometry(wing, num_elements)
    
    # Extract the arrays for the FEA solver
    X_nodes                         = geom['X_nodes']
    Y_nodes                         = geom['Y_nodes']
    Z_nodes                         = geom['Z_nodes']
    chord_nodes                     = geom['chord_nodes']
    twist_nodes_deg                 = geom['twist_nodes_deg']
    sweep_elems_rad                 = geom['sweep_mid_elems']
    dihedral_elems_rad              = geom['dihedral_elems']
    total_span                      = geom['total_span']
    spar_f_nodes                    = geom['spar_f_nodes']
    spar_r_nodes                    = geom['spar_r_nodes']
        

    discretized_params = Data(
        discretized_chords_elems = discretized_chords,
        discretized_twist_elems = discretized_twist,
        discretized_front_spar_elems =discretized_front_spar_locs,
        X_nodes = X_nodes
    )

    return discretized_params