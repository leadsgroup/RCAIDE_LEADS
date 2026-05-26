# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/FEA.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# package imports 

# Import Supporting Functions
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_material_properties          import compute_material_properties
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_wingbox_properties           import compute_wingbox_properties
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_loads                        import compute_loads
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_3d_transformation_matrix     import compute_3d_transformation_matrix
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_element_stiffness_arrays     import compute_element_stiffness_arrays
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_force_vector                 import compute_force_vector
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.plot_wingbox                         import plot_wingbox
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing                      import discretize_wing
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing                      import map_panel_forces_to_fea

# Python Imports
import numpy as np
import matplotlib.pyplot as plt
import time
import pyvista as pv

# --- Global PyVista Theme ---
pv.global_theme.font.family             = 'times'
pv.global_theme.font.label_size         = 14
pv.global_theme.font.title_size         = 16

# ----------------------------------------------------------------------
#  Finite Element Analysis
# ----------------------------------------------------------------------
def FEA(conditions,VLM_results,VD,settings,geometry):
    """
    Main Aero-Structural integration bridge.
    """ 
    n_cpts = len(VLM_results.CLift) # Number of timesteps/flight conditions
    # 1. Discretize Geometry
    num_elements       = settings.discretiation
    num_nodes          = num_elements + 1
    structural_results = Data()
    
    # generate structural node distribution  
    structural_results = Data()
    for wing in geometry.wings.values():
        structural_results[wing.tag]                      = Data()
        structural_results[wing.tag].structural_node_data = discretize_wing(wing, num_elements)     
        structural_results[wing.tag].load          = np.zeros((n_cpts,num_nodes,3))   # load x,y,z (formally w_z_load)
        structural_results[wing.tag].deflection    = np.zeros((n_cpts,num_nodes,3))   # deflection x,y,z
        structural_results[wing.tag].elastic_twist = np.zeros((n_cpts,num_nodes,3))     # twist x,y,z 
        
  
    # Loop over control points 
    for ti in range(n_cpts):
        
        # Array indices for panel flattening (Logic from plot_surface_pressures.py)
        # VD.n_sw[ti] * VD.n_cw[ti] gives the total panels for each wing segment
        panels_per_wing = VD.n_sw[ti] * VD.n_cw[ti]
        b_pts = np.concatenate(([0], np.cumsum(panels_per_wing))) 
        
        vd_idx = 0
        
        # Calculate Dynamic Pressure for this timestep
        rho  = conditions.freestream.density
        V    = conditions.freestream.velocity 
        
        rho[rho==0.0] =  1.225
        
        q_dyn = 0.5 * rho * (V ** 2)
        
        # LOOP OVER WINGS
        for wing in geometry.wings.values():
            VD_structural_wing = structural_results[wing.tag].structural_node_data

            sym = wing.xz_plane_symmetric 
            # 1. GENERATE FEA GEOMETRY (Optimize this to run once later)
            fea_pts = np.column_stack((VD_structural_wing.X_nodes[:-1], VD_structural_wing.Y_elems, VD_structural_wing.Z_nodes[:-1]))
            
            # 2. EXTRACT VLM CONTROL POINTS (For this specific wing and timestep)
            start_idx = int(b_pts[vd_idx])
            end_idx   = int(b_pts[vd_idx+1])
            
            XC = VD.XC[ti, start_idx:end_idx]
            YC = VD.YC[ti, start_idx:end_idx]
            # If ZC doesn't exist, assume flat wake (z=0)
            if hasattr(VD, 'ZC'):
                ZC = VD.ZC[ti, start_idx:end_idx]
            else:
                ZC = np.zeros_like(XC)
                
            vlm_pts = np.column_stack((XC, YC, ZC))
            
            Delta_CP      = VLM_results.CP[ti, start_idx:end_idx] 
            CP = np.tile(Delta_CP[:, np.newaxis], (1, 3)) # Expand to 3D for force calculation
            Normals = VD.normals[ti, start_idx:end_idx]
            
            # 3. EXTRACT VLM FORCES 
            F_vec =  CP * Normals * q_dyn
            Fx = - F_vec[:,1] # The normal is swaped in the VLM code, so Fx is actually the negative of the Y component of the force vector
            Fy = F_vec[:,0]
            Fz = F_vec[:,2]

            vlm_F = np.column_stack((Fx, Fy, Fz)) 
            
            # 4. MAP AERO TO STRUCTURE
            fea_forces, fea_moments = map_panel_forces_to_fea(vlm_pts, vlm_F, fea_pts)
            
            load_w_x_aero = fea_forces[:, 0]  # Drag
            load_w_y_aero = fea_forces[:, 1]  # Spanwise Force (Sideslip)
            load_w_z_aero = fea_forces[:, 2]  # Lift
            load_t_y_aero = fea_moments[:, 1] # Pitching Moment
            
            # 5. RUN STRUCTURAL SOLVER
            # Material & Wingbox
            E, G, Rho, Yield_Stress, Nu = compute_material_properties("CFRP_uCRM")
            
            # Pass the VD_struct object to your properties calculator
            A_arr, Ixx_arr, Izz_arr, J_arr, w_box_arr, h_arr = compute_wingbox_properties(wing, VD_structural_wing)
            
            # COMMENTED OUT TEMPORARY UNIFORM LOAD FOR BENCHMARKING
            # # Gravity & Mass Loads
            # w_z_struct  = -A_arr * Rho * 9.81
            
            # Airfoil_Area = 0.7 * VD_struct.chord_elems * h_arr
            # Rib_Mass_Per_Meter = (Airfoil_Area * VD_struct.wing_config['Rib_Thick'] * Rho) / VD_struct.wing_config['Rib_Spacing']
            # w_z_ribs = -Rib_Mass_Per_Meter * 9.81
            
            # REMOVE LATER---------------------------- 
            # Inertial Gravity Load (2.5g)
            g_load = 2.5 * 9.81
            # Gravity (Ribs + Wing Structure)
            # Wing Structure Weight
            w_z_struct = -A_arr * Rho * g_load
            mass_struct_total = np.sum(A_arr * Rho * VD_structural_wing.Le)
            # REMOVE UNTIL HERE ---------------------------- 
            
            # Total Loads
            load_w_z_total = load_w_z_aero + w_z_struct # + w_z_ribs
            
            # Matrices Assembly
            twist_elems_rad = ((VD_structural_wing.twist_nodes[:-1] + VD_structural_wing.twist_nodes[1:]) / 2)
            T_all = compute_3d_transformation_matrix(VD_structural_wing.sweep_elems_rad, VD_structural_wing.dihedral_elems_rad, twist_elems_rad, num_elements)
            K_local = compute_element_stiffness_arrays(E, G, A_arr, J_arr, Ixx_arr, Izz_arr, VD_structural_wing.Le, num_elements)
            
            K_temp = np.matmul(K_local, T_all)
            K_global_elem = np.matmul(np.transpose(T_all, (0, 2, 1)), K_temp)
            F_global_elem = compute_force_vector(load_w_x_aero, load_w_y_aero, load_w_z_total, load_t_y_aero, VD_structural_wing.Le, num_elements, T_all)
            
            # Global Assembly
            num_nodes = num_elements + 1
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
            
            # Solve boundary conditions (Cantilever)
            constrained_dof = np.arange(0, 6)
            all_dofs = np.arange(total_dof)
            free_dof = np.setdiff1d(all_dofs, constrained_dof)
            K_reduced = K_global[np.ix_(free_dof, free_dof)]
            F_reduced = F_global[free_dof]
            
            u_reduced = np.linalg.solve(K_reduced, F_reduced)
            u_full = np.zeros(total_dof)
            u_full[free_dof] = u_reduced
             
            theta_x = u_full[3::6]
            theta_y = u_full[4::6]
            
            twist_local = (theta_x * np.sin(VD_structural_wing.sweep_nodes) + theta_y * np.cos(VD_structural_wing.sweep_nodes))
            
            # store results  
            node_aero_loads = np.interp(VD_structural_wing.y_local, VD_structural_wing.Y_elems, load_w_z_aero)

 
            structural_results[wing.tag].load[ti,:,2]           =  node_aero_loads
            structural_results[wing.tag].elastic_twist[ti,:,1]  = u_full[4::6] # twist y 
            structural_results[wing.tag].deflection[ti,:,2]     = u_full[2::6] # deflection z
            structural_results[wing.tag].deflection[ti,:,0]     = u_full[0::6] # deflection x
 
            vd_idx += 1
            if sym:
                vd_idx += 1
            
    # Call the 3D Plotter  - TO REMOVE and CHANGE SO THAT ALL WINGS ARE PLOTTED 
    plot_wingbox(structural_results, wing, 0 , scale=1.0)
            
    return structural_results