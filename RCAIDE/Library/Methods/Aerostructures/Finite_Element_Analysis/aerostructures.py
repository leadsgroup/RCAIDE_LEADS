# Python Imports
import numpy as np
import matplotlib.pyplot as plt
import time
import pyvista as pv

# Import Supporting Functions
import RCAIDE
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_material_properties import compute_material_properties
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_multisegment_geometry import compute_multisegment_geometry
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_wingbox_properties import compute_wingbox_properties
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_loads import compute_loads
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_3d_transformation_matrix import compute_3d_transformation_matrix
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_element_stiffness_arrays import compute_element_stiffness_arrays
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_force_vector import compute_force_vector
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.plot_wingbox import plot_wingbox

# --- Global PyVista Theme ---
pv.global_theme.font.family = 'times'
pv.global_theme.font.label_size = 14
pv.global_theme.font.title_size = 16

# Main Solver Function

def aerostructures():
    print("--- FEA SOLVER VERSION 12: Wingbox Segmentation (uCRM Capabilities)---")
    start_time = time.time()
    
    # --- INPUTS ---
    
    # --- WING DEFINITION ---
    wing_config = {
        # Global Wing Properties
        't_c': 0.12,
        'Skin_Top_Thick': 0.0065,
        'Skin_Bot_Thick': 0.0065,
        'Rib_Spacing': 0.6,
        'Rib_Thick': 0.004,
        'Front_Spar': {'type': 'I_Beam', 't_web': 0.008, 'w_cap': 0.080, 't_cap': 0.012},
        'Rear_Spar':  {'type': 'C_Channel', 't_web': 0.006, 'w_cap': 0.060, 't_cap': 0.008},
        
        # Segment Definitions
        # *Note: Outboard Root must perfectly match Inboard Tip For all Parameters
        'segments': [
            {   # INBOARD SEGMENT (Yehudi Break)
                'span': 1.5, 
                'sweep_LE': 28.18, 
                'dihedral': 0.0, 
                'chord_root': 5.000, 
                'chord_tip': 4.625, 
                'twist_root': 0.0, 
                'twist_tip': -0.32,
                'spar_f_root': 0.2994, 'spar_f_tip': 0.1500, # Spars taper to meet outboard section
                'spar_r_root': 0.7620, 'spar_r_tip': 0.6500
            },
            {   # OUTBOARD SEGMENT (Main Wing)
                'span': 12.5, 
                'sweep_LE': 28.18, 
                'dihedral': 4.0, 
                'chord_root': 4.625, 
                'chord_tip': 1.500, 
                'twist_root': -0.32, 
                'twist_tip': -3.0,
                'spar_f_root': 0.1500, 'spar_f_tip': 0.1500, # Constant fractions outboard
                'spar_r_root': 0.6500, 'spar_r_tip': 0.6500
            }
        ]
    }

    num_elements = 400
    num_nodes = num_elements + 1
    
    # Material
    E, G, Rho, Yield_Stress, Nu = compute_material_properties("Al7075_T6")
    
    # Generate the geometry
    geom = compute_multisegment_geometry(wing_config, num_elements)
    
    # Extract the arrays for the FEA solver
    X_nodes = geom['X_nodes']
    Y_nodes = geom['Y_nodes']
    Z_nodes = geom['Z_nodes']
    chord_nodes = geom['chord_nodes']
    twist_nodes_deg = geom['twist_nodes_deg']
    sweep_elems_rad = geom['sweep_mid_elems']
    dihedral_elems_rad = geom['dihedral_elems']
    total_span = geom['total_span']
    spar_f_nodes = geom['spar_f_nodes']
    spar_r_nodes = geom['spar_r_nodes']
    
    # Calculate Element Length (Le) For Each Element
    Le = np.sqrt(np.diff(X_nodes)**2 + np.diff(Y_nodes)**2 + np.diff(Z_nodes)**2)
    
    # We need the element centers for calculating loads and stiffness
    chord_elems = (chord_nodes[:-1] + chord_nodes[1:]) / 2
    twist_elems_rad = np.radians((twist_nodes_deg[:-1] + twist_nodes_deg[1:]) / 2)
    y_elem_centers = np.cumsum(Le) - (Le / 2) # Distance along elastic axis
    spar_f_elems = (spar_f_nodes[:-1] + spar_f_nodes[1:]) / 2
    spar_r_elems = (spar_r_nodes[:-1] + spar_r_nodes[1:]) / 2
    
    # --- WINGBOX ASSEMBLY ---
    A_arr, Ixx_arr, Izz_arr, J_arr, w_box_arr, h_arr = compute_wingbox_properties(
        chord_elems, wing_config['t_c'], 
        spar_f_elems, spar_r_elems, 
        wing_config['Front_Spar'], wing_config['Rear_Spar'],
        wing_config['Skin_Top_Thick'], wing_config['Skin_Bot_Thick']
    )
    
    # --- LOADS ---
    # Aero
    Y_elems = (Y_nodes[:-1] + Y_nodes[1:]) / 2
    load_w_z_aero, load_t_y_aero = compute_loads(5000, Y_elems, total_span)
    # Gravity (Ribs + Wing Structure)
    # Wing Structure Weight
    w_z_struct = -A_arr * Rho * 9.81
    
    # Rib Weight (Distributed)
    Airfoil_Area = 0.7 * chord_elems * h_arr
    Rib_Mass_Per_Meter = (Airfoil_Area * wing_config['Rib_Thick'] * Rho) / wing_config['Rib_Spacing']
    w_z_ribs = -Rib_Mass_Per_Meter * 9.81
    
    # Total Vertical Load
    load_w_z_total = load_w_z_aero # + w_z_struct + w_z_ribs # Weight of Wing Structure Neglected
    load_w_x = np.zeros(num_elements)
    load_w_y = np.zeros(num_elements)
    
    ti_assembly = time.time()
    # --- MATRICES ASSEMBLY ---
    T_all = compute_3d_transformation_matrix(sweep_elems_rad, dihedral_elems_rad, twist_elems_rad, num_elements)
    K_local = compute_element_stiffness_arrays(E, G, A_arr, J_arr, Ixx_arr, Izz_arr, Le, num_elements)
    
    # Rotate Stiffness
    K_temp = np.matmul(K_local, T_all)
    K_global_elem = np.matmul(np.transpose(T_all, (0, 2, 1)), K_temp)
    
    # Force Vector
    F_global_elem = compute_force_vector(load_w_x, load_w_y, load_w_z_total, load_t_y_aero, Le, num_elements, T_all)
    
    # Assembly
    dofs_per_node = 6
    total_dof = dofs_per_node * num_nodes
    K_global = np.zeros((total_dof, total_dof))
    F_global = np.zeros(total_dof)
    
    global_indices = np.zeros((num_elements, 12), dtype=int)
    node_indices = np.arange(num_nodes)
    dof_indices = np.arange(dofs_per_node)
    global_indices[:, :6] = dofs_per_node * node_indices[:-1, np.newaxis] + dof_indices
    global_indices[:, 6:] = dofs_per_node * node_indices[1:, np.newaxis] + dof_indices
    
    rows = global_indices[:, :, np.newaxis]
    cols = global_indices[:, np.newaxis, :]
    np.add.at(K_global, (rows, cols), K_global_elem)
    np.add.at(F_global, global_indices, F_global_elem)
    
    tf_assembly = time.time()
    print(f"Global Stiffness & Force Matrix Assembly Complete. (Time: {tf_assembly-ti_assembly:.5f} seconds)")
    
    # Boundary Conditions (Cantilever Beam)
    constrained_dof = np.arange(0, 6)
    all_dofs = np.arange(total_dof)
    free_dof = np.setdiff1d(all_dofs, constrained_dof)
    K_reduced = K_global[np.ix_(free_dof, free_dof)]
    F_reduced = F_global[free_dof]
    
    solve_start = time.time()
    u_reduced = np.linalg.solve(K_reduced, F_reduced)
    solve_end = time.time()
    print(f"Solve complete. (Time: {solve_end - solve_start:.5f} seconds)")
    
    u_full = np.zeros(total_dof)
    u_full[free_dof] = u_reduced
    
    # --- POST-PROCESSING ---
    w_global = u_full[2::6]
    theta_x = u_full[3::6]
    theta_y = u_full[4::6]
    
    # Interpolate element sweeps back to nodes to calculate Post-Processing Twist properly
    sweep_nodes_rad = np.interp(np.arange(num_nodes), np.arange(num_elements)+0.5, sweep_elems_rad)
    # Local Twist
    twist_local_deg = np.degrees(theta_x * np.sin(sweep_nodes_rad) + theta_y * np.cos(sweep_nodes_rad))
    
    # --- BUCKLING CHECK ---
    # 1. Calculate Bending Stress
    d_theta_x = np.diff(theta_x)
    d_theta_y = np.diff(theta_y)
    
    # Project the relative nodal rotations onto the element's local bending axis
    curvature = (d_theta_x * np.cos(sweep_elems_rad) - d_theta_y * np.sin(sweep_elems_rad)) / Le
    sigma_bending = E * (h_arr/2) * np.abs(curvature)
    
    # 2. Calculate Dynamic Buckling Coefficient 'k'
    # Aspect Ratio = Length (Rib Spacing) / Width (Box Width)
    # Note: If ribs are far apart (AR > 1), k drops to 4.0.
    #       If ribs are close (AR < 1), k shoots up.
    AR_arr = wing_config['Rib_Spacing'] / w_box_arr
    
    # Formula for Simply Supported Plate on 4 sides (Load in 'a' direction)
    # If AR < 1 (Wide short plate): k = (AR + 1/AR)^2
    # If AR >= 1 (Long narrow plate): k reaches min of 4.0 at integer intervals
    k_arr = np.where(AR_arr < 1, 
                     (AR_arr + 1/AR_arr)**2,  # Short plate boost
                     4.0)                     # Long plate conservative limit
    
    # 3. Clamping Factor
    # Set Fixity factor to 1 to model riveted joint and 1.75 to model perfect joint (clamped)
    # Clamping can raise k_min from 4.0 to ~7.0
    fixity_factor = 1.75 
    k_arr = k_arr * fixity_factor

    # 4. Calculate Critical Stress with dynamic k
    sigma_crit = (k_arr * np.pi**2 * E) / (12 * (1 - Nu**2)) * (wing_config['Skin_Top_Thick'] / w_box_arr)**2
    
    # 5. Margin of Safety
    MoS = (sigma_crit / (sigma_bending + 1e-6)) - 1
    min_margin = np.min(MoS)
    
    end_time = time.time()
    print(f"Total Script Time: {end_time - start_time:.4f} seconds")
    
    print(f"\n--- RESULTS ---")
    print(f"Max Tip Deflection: {w_global[-1]*1000:.2f} mm")
    print(f"Max Twist: {twist_local_deg[-1]:.2f} deg")
    print(f"Max Stress (Top Skin): {np.max(sigma_bending)/1e6:.1f} MPa")
    print(f"Buckling Margin of Safety: {min_margin:.2f}")
    
    if min_margin < 0:
        print(">> WARNING: SKIN BUCKLING PREDICTED! <<")
        print(f"   Reduce Rib Spacing (Current: {wing_config['Rib_Spacing']} m) or Increase Skin Thickness.")
    else:
        print("Skin Design is Safe.")

    # --- PLOTTING ---
    
    # 3D Visualization
    y_local_path = np.insert(np.cumsum(Le), 0, 0.0)
    node_aero_loads = np.interp(y_local_path, y_elem_centers, load_w_z_aero)
    res = {
        'y_local': y_local_path, 
        'chord': chord_nodes, 
        'X0': X_nodes,                                         # Undeformed Baseline
        'Y0': Y_nodes, 
        'Z0': Z_nodes,
        'deflection': w_global,
        'twist_geo': np.radians(twist_nodes_deg),              # Geometric Washout
        'twist_elas': np.radians(twist_local_deg),             # Twist due to loading
        'w_z_load': node_aero_loads,
        'spar_f': spar_f_nodes, 
        'spar_r': spar_r_nodes,
        'params': { 
            'tc': wing_config['t_c'], 
            'Rib_Spacing': wing_config['Rib_Spacing'], 
            'Span': total_span,
            'Front_Spar': wing_config['Front_Spar'], 
            'Rear_Spar': wing_config['Rear_Spar']
        }
    }
    
    # Call the 3D Plotter
    plot_wingbox(res, scale=20.0)
    
    # Plot 2: Buckling Analysis
    plt.figure(figsize=(14, 6))
    plt.plot(Y_elems, sigma_crit/1e6, 'g--', label='Critical Stress')
    plt.plot(Y_elems, sigma_bending/1e6, 'r-', label='Actual Stress')
    plt.title(f'Skin Buckling Analysis (Rib Spacing {wing_config['Rib_Spacing']*1000:.0f}mm)')
    plt.ylabel('Stress (MPa)')
    plt.xlabel('Span (m)')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    aerostructures()