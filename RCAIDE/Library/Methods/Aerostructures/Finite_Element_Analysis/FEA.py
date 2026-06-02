# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/FEA.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# Import Supporting Functions
from RCAIDE.Framework.Core                                                                              import Data 
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_wingbox_properties           import compute_wingbox_properties 
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_3d_transformation_matrix     import compute_3d_transformation_matrix
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_element_stiffness_arrays     import compute_element_stiffness_arrays
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_force_vector                 import compute_force_vector
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing                      import discretize_wing
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing                      import map_panel_forces_to_fea

# Python Imports
import numpy as np
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
    # compute freestream properties 
    rho           = conditions.freestream.density
    V             = conditions.freestream.velocity  
    rho[rho==0.0] =  1.225 
    q_dyn         = 0.5 * rho * (V ** 2)
    n             = settings.load_factor  
    
    n_cpts = len(VLM_results.CLift)
    
    # Discretize Geometry
    num_elements       = settings.discretiation
    num_nodes          = num_elements + 1 
    
    # generate structural node distribution  
    structural_results = Data()
    for wing in geometry.wings.values():
        structural_results[wing.tag]                      = Data()
        structural_results[wing.tag].structural_node_data = discretize_wing(wing, num_elements)     
        structural_results[wing.tag].load                 = np.zeros((n_cpts,num_nodes,3))   # load x,y,z (formally w_z_load)
        structural_results[wing.tag].deflection           = np.zeros((n_cpts,num_nodes,3))   # deflection x,y,z
        structural_results[wing.tag].elastic_twist        = np.zeros((n_cpts,num_nodes,1))   # twist x,y,z 
        
    # Loop over control points 
    for ti in range(n_cpts):
         
        panels_per_wing = VD.n_sw[ti] * VD.n_cw[ti]
        b_pts = np.concatenate(([0], np.cumsum(panels_per_wing)))  
        vd_idx = 0 
         
        for wing in geometry.wings.values():
            VD_structural_wing = structural_results[wing.tag].structural_node_data
            
            # Geernate FEA geometry 
            fea_pts = np.column_stack((VD_structural_wing.X_elems, VD_structural_wing.Y_elems, VD_structural_wing.Z_elems))
            
            # Extract VLM control points 
            start_idx = int(b_pts[vd_idx])
            end_idx   = int(b_pts[vd_idx+1]) 
            
            # Extract VLM forces 
            Delta_CP    = VLM_results.CP[ti, start_idx:end_idx]  
            Normals     = VD.normals[ti, start_idx:end_idx]
            Panel_Areas = VD.panel_areas[ti, start_idx:end_idx]
            
            # Force (N) = Cp * Normal_Vector * q_dyn * Area
            F_vec       = np.tile(Delta_CP[:, np.newaxis], (1, 3))  * Normals * q_dyn * Panel_Areas[:, np.newaxis]
            Fx          = - F_vec[:,1] # The normal is swaped in the VLM code, so Fx is actually the negative of the Y component of the force vector
            Fy          = F_vec[:,0]
            Fz          = F_vec[:,2]  
            total_loads = np.column_stack((Fx, Fy, Fz)) 
            total_pts   = np.column_stack((VD.XC[ti, start_idx:end_idx], VD.YC[ti, start_idx:end_idx], VD.ZC[ti, start_idx:end_idx]))
      
            # FUTURE: ADD FUNCTION HERE TO APPEND THE LOADS AND POINTS DATA STRUCTURE WIHT ADDITIONAL LOADS (BATTERIES, MOTORS/PROPELLERS,TANKS ETC)
            # FUTURE WORK 

            # Map aerodynamic loads to structure
            fea_forces, fea_moments = map_panel_forces_to_fea(total_pts, total_loads, fea_pts)
            
            # Extract panel forces (N)
            load_w_x_aero = fea_forces[:, 0]  # Drag
            load_w_y_aero = fea_forces[:, 1]  # Spanwise Force (Sideslip)
            load_w_z_aero = fea_forces[:, 2]  # Lift
            
            # Extract Global Moments (N-m)
            M_x = fea_moments[:, 0]
            M_y = fea_moments[:, 1]
            M_z = fea_moments[:, 2]
            
            # Project global moments onto the local swept/dihedral elastic axis for True Torsion
            load_t_y_aero = (M_x * np.sin(VD_structural_wing.sweep_mid_elems) * np.cos(VD_structural_wing.dihedral_elems) + 
                 M_y * np.cos(VD_structural_wing.sweep_mid_elems) * np.cos(VD_structural_wing.dihedral_elems) + 
                 M_z * np.sin(VD_structural_wing.dihedral_elems)) # Pitching Moment
            
            # Run structural solver 
            E    = wing.structural.material.youngs_modulus      
            G    = wing.structural.material.shear_modulus     
            Rho  = wing.structural.material.density       
            
            # Pass the VD_struct object to your properties calculator
            A_arr, Ixx_arr, Izz_arr, J_arr, w_box_arr = compute_wingbox_properties(wing, VD_structural_wing)
           
            # Gravity & Mass Loads
            w_z_struct         = -A_arr * Rho * 9.81 * n 
            airfoil_Area       = 0.7 * VD_structural_wing.chord_elems * VD_structural_wing.t_elems # approximation 
            Rib_Mass_Per_Meter = (airfoil_Area * wing.structural.rib_thickness  * Rho) / wing.structural.rib_spacing
            w_z_ribs           = -Rib_Mass_Per_Meter * 9.81 * n 

            # Total Distributed Loads
            load_w_z_distributed = w_z_struct + w_z_ribs
            
            # Matrices Assembly
            T_all         = compute_3d_transformation_matrix(VD_structural_wing.sweep_mid_elems, VD_structural_wing.dihedral_elems, VD_structural_wing.twist_elems, num_elements)
            K_local       = compute_element_stiffness_arrays(E, G, A_arr, J_arr, Ixx_arr, Izz_arr, VD_structural_wing.Le, num_elements)
            
            K_temp        = np.matmul(K_local, T_all)
            K_global_elem = np.matmul(np.transpose(T_all, (0, 2, 1)), K_temp)
            
            # Compute distributed force vector (Aero loads set to 0.0)
            F_global_elem = compute_force_vector(w_x=np.zeros_like(load_w_x_aero), w_y=np.zeros_like(load_w_y_aero), w_z=load_w_z_distributed, t_y=np.zeros_like(load_t_y_aero), Le=VD_structural_wing.Le, num_elem=num_elements, T=T_all)
            
            # Global Assembly
            num_nodes     = num_elements + 1
            dofs_per_node = 6
            total_dof     = dofs_per_node * num_nodes
            K_global      = np.zeros((total_dof, total_dof))
            F_global      = np.zeros(total_dof)
            
            global_indices        = np.zeros((num_elements, 12), dtype=int)
            node_indices          = np.arange(num_nodes)
            dof_indices           = np.arange(dofs_per_node)
            global_indices[:, :6] = dofs_per_node * node_indices[:-1, np.newaxis] + dof_indices
            global_indices[:, 6:] = dofs_per_node * node_indices[1:, np.newaxis] + dof_indices
            
            rows = global_indices[:, :, np.newaxis]
            cols = global_indices[:, np.newaxis, :]
            np.add.at(K_global, (rows, cols), K_global_elem)
            np.add.at(F_global, global_indices, F_global_elem)
            
            # Direct Nodal Injection of Discrete Aero Loads
            # Split the mapped element force 50/50 to its left and right nodes
            node1_idx = 6 * np.arange(num_elements)
            node2_idx = 6 * (np.arange(num_elements) + 1)
            
            # Inject Forces (Fx, Fy, Fz in Newtons)
            F_global[node1_idx + 0] += load_w_x_aero / 2
            F_global[node2_idx + 0] += load_w_x_aero / 2
            F_global[node1_idx + 1] += load_w_y_aero / 2
            F_global[node2_idx + 1] += load_w_y_aero / 2
            F_global[node1_idx + 2] += load_w_z_aero / 2
            F_global[node2_idx + 2] += load_w_z_aero / 2
            
            # Inject Global Moments (Mx, My, Mz in Newton-meters)
            F_global[node1_idx + 3] += M_x / 2
            F_global[node2_idx + 3] += M_x / 2
            F_global[node1_idx + 4] += M_y / 2
            F_global[node2_idx + 4] += M_y / 2
            F_global[node1_idx + 5] += M_z / 2
            F_global[node2_idx + 5] += M_z / 2
            
            # Solve boundary conditions (Cantilever)
            constrained_dof  = np.arange(0, 6)
            all_dofs         = np.arange(total_dof)
            free_dof         = np.setdiff1d(all_dofs, constrained_dof)
            K_reduced        = K_global[np.ix_(free_dof, free_dof)]
            F_reduced        = F_global[free_dof]
            
            u_reduced        = np.linalg.solve(K_reduced, F_reduced)
            u_full           = np.zeros(total_dof)
            u_full[free_dof] = u_reduced  
            twist_local      = ( u_full[3::6] * np.sin(VD_structural_wing.sweep_nodes) + u_full[4::6] * np.cos(VD_structural_wing.sweep_nodes))
            
            # store results
            # Create an arc-length coordinate array for the center of each element
            y_elem_centers = (VD_structural_wing.y_local[:-1] + VD_structural_wing.y_local[1:]) / 2  
            node_aero_loads = np.interp(VD_structural_wing.y_local, y_elem_centers, load_w_z_aero)
            
            structural_results[wing.tag].load[ti,:,2]           = node_aero_loads
            structural_results[wing.tag].elastic_twist[ti,:,0]  = twist_local
            structural_results[wing.tag].deflection[ti,:,0]     = u_full[0::6]  # X deflection (Chordwise)
            structural_results[wing.tag].deflection[ti,:,1]     = u_full[1::6]  # Y deflection (Spanwise)
            structural_results[wing.tag].deflection[ti,:,2]     = u_full[2::6]  # Z deflection (Vertical)
 
            vd_idx += 1
            if wing.xz_plane_symmetric:
                vd_idx += 1
            
    return structural_results