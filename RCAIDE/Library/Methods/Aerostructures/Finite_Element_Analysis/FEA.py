# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/FEA.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# Import Supporting Functions
import RCAIDE
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
from scipy.sparse         import coo_matrix
from scipy.sparse.linalg  import spsolve

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
    q_dyn         = conditions.freestream.dynamic_pressure 
    n             = settings.load_factor  
    
    n_cpts = len(VLM_results.CLift)
    
    # Discretize Geometry
    num_elements       = settings.discretiation
    num_nodes          = num_elements + 1 
    
    # generate structural node distribution  
    structural_results = conditions.aerostructures
    for wing in geometry.wings.values():
        structural_results[wing.tag]                      = Data()
        structural_results[wing.tag].structural_node_data = discretize_wing(wing, num_elements)     
        structural_results[wing.tag].load                 = np.zeros((n_cpts,num_nodes,3))   # load x,y,z (formally w_z_load)
        structural_results[wing.tag].deflection           = np.zeros((n_cpts,num_nodes,3))   # deflection x,y,z
        structural_results[wing.tag].elastic_twist        = np.zeros((n_cpts,num_nodes,1))   # twist x,y,z
        structural_results[wing.tag].normal_stress        = np.zeros((n_cpts,num_elements))  # combined axial + bending, per element
        structural_results[wing.tag].shear_stress         = np.zeros((n_cpts,num_elements))  # torsional shear (thinnest wall), per element
        structural_results[wing.tag].margin_of_safety     = np.zeros((n_cpts,num_elements))  # min(yield/normal, yield_shear/shear) - 1
 
    # Precompute structural point loads; fuel loads computed per control point (time-varying)
    source_pts       = np.zeros((0, 3))
    source_loads_cst = np.zeros((0, 3))
    propulsive_pts   = np.zeros((0, 3))
    propulsive_loads = np.zeros((0, 3))
    fuel_tank_info   = []  # list of (global_CG, fuel_tag, tank_static_mass)

    for network in geometry.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                if source.wing_tag is not None:
                    global_CG = np.array(source.origin[0]) + np.array(source.mass_properties.center_of_gravity)
                    fuel_tank_info.append((global_CG, source.fuel.tag, source.mass_properties.mass))

            elif isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                for battery_module in source.modules:
                    if battery_module.wing_tag is not None:
                        battery_load = float(np.mean(battery_module.mass_properties.mass * conditions.freestream.gravitational_acceleration)) * n
                        global_CG    = np.array(battery_module.origin[0]) + np.array(battery_module.mass_properties.center_of_gravity)
                        source_pts       = np.vstack([source_pts,       global_CG.reshape(1, 3)])
                        source_loads_cst = np.vstack([source_loads_cst, [[0.0, 0.0, -battery_load]]])

        for propulsor in network.propulsors:
            if propulsor.wing_mounted:
                prop_load = float(np.mean(propulsor.mass_properties.mass * conditions.freestream.gravitational_acceleration)) * n
                global_CG = np.array(propulsor.origin[0]) + np.array(propulsor.mass_properties.center_of_gravity)
                propulsive_pts   = np.vstack([propulsive_pts,   global_CG.reshape(1, 3)])
                propulsive_loads = np.vstack([propulsive_loads, [[0.0, 0.0, -prop_load]]])


    # Loop over control points
    for ti in range(n_cpts):

        # Time-varying fuel loads for this control point
        ti_source_pts   = source_pts.copy()
        ti_source_loads = source_loads_cst.copy()
        for (global_CG, fuel_tag, tank_mass) in fuel_tank_info:
            fuel_mass = conditions.weights.components.mass[fuel_tag][ti, 0] + tank_mass
            fuel_load = fuel_mass * float(conditions.freestream.gravitational_acceleration[ti, 0]) * n
            ti_source_pts   = np.vstack([ti_source_pts,   global_CG.reshape(1, 3)])
            ti_source_loads = np.vstack([ti_source_loads, [[0.0, 0.0, -fuel_load]]])

        vd_ti           = min(ti, len(VD.n_sw) - 1)   # n_sw/n_cw stored once when mesh is fixed
        panels_per_wing = VD.n_sw[vd_ti] * VD.n_cw[vd_ti]
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
            Normals     = VD.normals[vd_ti, start_idx:end_idx]
            Panel_Areas = VD.panel_areas[vd_ti, start_idx:end_idx]

            # Force (N) = Cp * Normal_Vector * q_dyn * Area
            F_vec      = np.tile(Delta_CP[:, np.newaxis], (1, 3)) * Normals * q_dyn[ti,0] * Panel_Areas[:, np.newaxis]
            Fx         = -F_vec[:, 1]
            Fy         =  F_vec[:, 0]
            Fz         =  F_vec[:, 2]
            aero_loads = np.column_stack((Fx, Fy, Fz))
            aero_pts   = np.column_stack((VD.XC[vd_ti, start_idx:end_idx], VD.YC[vd_ti, start_idx:end_idx], VD.ZC[vd_ti, start_idx:end_idx]))

            total_loads = np.concatenate((aero_loads, propulsive_loads, ti_source_loads), axis=0)
            total_pts   = np.concatenate((aero_pts,   propulsive_pts,   ti_source_pts),   axis=0)

            # Map aerodynamic loads to structure and convert to distributed (N/m)
            fea_forces, fea_moments = map_panel_forces_to_fea(total_pts, total_loads, fea_pts)
            w_x_aero = fea_forces[:, 0] / VD_structural_wing.Le
            w_y_aero = fea_forces[:, 1] / VD_structural_wing.Le
            w_z_aero = fea_forces[:, 2] / VD_structural_wing.Le

            # Project eccentricity moments onto the elastic axis for torsion
            M_x = fea_moments[:, 0]
            M_y = fea_moments[:, 1]
            M_z = fea_moments[:, 2]
            load_t_y_aero = (M_x * np.sin(VD_structural_wing.sweep_mid_elems) * np.cos(VD_structural_wing.dihedral_elems) +
                             M_y * np.cos(VD_structural_wing.sweep_mid_elems) * np.cos(VD_structural_wing.dihedral_elems) +
                             M_z * np.sin(VD_structural_wing.dihedral_elems))

            # Run structural solver 
            E    = wing.structural.material.youngs_modulus      
            G    = wing.structural.material.shear_modulus     
            Rho  = wing.structural.material.density       
            
            # Pass the VD_struct object to your properties calculator
            A_arr, Ixx_arr, Izz_arr, J_arr, w_box_arr = compute_wingbox_properties(wing, VD_structural_wing)

            # Verification-only: override the VLM aero load with a uniform pressure
            # field so the beam solver can be checked against an independent
            # reference implementation on identical loads (settings.verification_pressure).
            if getattr(settings, 'verification_pressure', None) is not None:
                w_x_aero      = np.zeros_like(w_box_arr)
                w_y_aero      = np.zeros_like(w_box_arr)
                w_z_aero      = settings.verification_pressure * w_box_arr * np.cos(VD_structural_wing.sweep_mid_elems)
                load_t_y_aero = np.zeros_like(w_box_arr)

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

            # K_global = T @ K_local @ T^T, consistent with the u_global = T @ u_local
            # / F_global = T @ F_local convention used throughout this module
            # (see compute_force_vector.py). Verified against the closed-form
            # decoupled-axis Euler-Bernoulli cantilever solution (local bending +
            # axial stretch, rotated to global) for an arbitrarily swept/dihedral
            # single element under a uniform global load: matches to machine
            # precision, whereas the previous T^T @ K_local @ T order did not.
            K_temp        = np.matmul(K_local, np.transpose(T_all, (0, 2, 1)))
            K_global_elem = np.matmul(T_all, K_temp)
            
            # All loads routed through T for correct bending-torsion coupling
            F_global_elem = compute_force_vector(w_x=w_x_aero, w_y=w_y_aero, w_z=w_z_aero + load_w_z_distributed, t_y=load_t_y_aero, Le=VD_structural_wing.Le, num_elem=num_elements, T=T_all)
            
            # Global Assembly (sparse). This beam-chain stiffness matrix is
            # banded -- each element only couples to its immediate neighbor,
            # so a dense (total_dof x total_dof) matrix and O(n^3) dense solve
            # spend nearly all their effort on structural zeros. COO sums
            # duplicate (row,col) entries on conversion to CSC, which is
            # exactly what np.add.at did for nodes shared between adjacent
            # elements, so the assembled matrix is identical to the dense one.
            num_nodes     = num_elements + 1
            dofs_per_node = 6
            total_dof     = dofs_per_node * num_nodes
            F_global      = np.zeros(total_dof)

            global_indices        = np.zeros((num_elements, 12), dtype=int)
            node_indices          = np.arange(num_nodes)
            dof_indices           = np.arange(dofs_per_node)
            global_indices[:, :6] = dofs_per_node * node_indices[:-1, np.newaxis] + dof_indices
            global_indices[:, 6:] = dofs_per_node * node_indices[1:, np.newaxis] + dof_indices

            rows     = np.repeat(global_indices, 12, axis=1).ravel()
            cols     = np.tile(global_indices, 12).ravel()
            K_global = coo_matrix((K_global_elem.ravel(), (rows, cols)),
                                   shape=(total_dof, total_dof)).tocsc()
            np.add.at(F_global, global_indices, F_global_elem)

            # Solve boundary conditions (Cantilever): node 0's 6 DOFs are
            # always the fixed root, so the free block is a contiguous slice
            # -- no fancy indexing needed, sparse or dense.
            K_reduced        = K_global[6:, 6:]
            F_reduced        = F_global[6:]

            u_reduced        = spsolve(K_reduced, F_reduced)
            u_full           = np.zeros(total_dof)
            u_full[6:]       = u_reduced
            twist_local      = ( u_full[3::6] * np.sin(VD_structural_wing.sweep_nodes) + u_full[4::6] * np.cos(VD_structural_wing.sweep_nodes))

            # ------------------------------------------------------------
            # Internal loads and stress recovery (per element)
            # ------------------------------------------------------------
            # Member-end actions (local coords) = K_local @ u_local - F_local, where
            # F_local is the local equivalent-nodal-load vector for the distributed
            # load on this element (same formula as compute_force_vector.py, kept
            # local here rather than exposed from that function since it is only
            # needed for this internal-force recovery). Validated numerically
            # against the closed-form cantilever-beam UDL solution (tip deflection
            # wL^4/(8EI) and root moment wL^2/2) to machine precision.
            Le               = VD_structural_wing.Le
            loads_global_e   = np.column_stack((w_x_aero, w_y_aero, w_z_aero + load_w_z_distributed))
            R_blocks         = T_all[:, 0:3, 0:3]
            loads_local_e    = np.einsum('eji,ej->ei', R_blocks, loads_global_e)
            w_u_l, w_v_l, w_w_l = loads_local_e[:, 0], loads_local_e[:, 1], loads_local_e[:, 2]

            F_local_fe        = np.zeros((num_elements, 12))
            F_local_fe[:, 0]  = w_u_l * Le / 2;         F_local_fe[:, 6]  = w_u_l * Le / 2
            F_local_fe[:, 5]  = -w_u_l * Le**2 / 12;    F_local_fe[:, 11] = w_u_l * Le**2 / 12
            F_local_fe[:, 1]  = w_v_l * Le / 2;         F_local_fe[:, 7]  = w_v_l * Le / 2
            F_local_fe[:, 2]  = w_w_l * Le / 2;         F_local_fe[:, 8]  = w_w_l * Le / 2
            F_local_fe[:, 3]  = w_w_l * Le**2 / 12;     F_local_fe[:, 9]  = -w_w_l * Le**2 / 12
            F_local_fe[:, 4]  = load_t_y_aero * Le / 2; F_local_fe[:, 10] = load_t_y_aero * Le / 2

            u_elem_global    = u_full[global_indices]
            u_elem_local     = np.einsum('eij,ej->ei', np.transpose(T_all, (0, 2, 1)), u_elem_global)
            q_local          = np.einsum('eij,ej->ei', K_local, u_elem_local) - F_local_fe

            N_axial   = q_local[:, 1]  # axial force, local spanwise axis
            M_bend_x  = q_local[:, 3]  # bending moment about local x -> vertical (Ixx) bending
            M_bend_z  = q_local[:, 5]  # bending moment about local z -> chordwise (Izz) bending
            M_torsion = q_local[:, 4]  # torsion moment about local spanwise axis

            h_arr = VD_structural_wing.chord_elems * VD_structural_wing.t_c_elems  # wingbox height, matches compute_wingbox_properties
            sigma_axial  = N_axial / A_arr
            sigma_bend_x = np.abs(M_bend_x) * (h_arr / 2.0) / Ixx_arr
            sigma_bend_z = np.abs(M_bend_z) * (w_box_arr / 2.0) / Izz_arr
            # Conservative: assumes the worst bending corner coincides with peak axial stress.
            normal_stress = np.abs(sigma_axial) + sigma_bend_x + sigma_bend_z

            # Torsional shear (Bredt-Batho closed single-cell): shear flow is
            # uniform around the perimeter, so peak stress occurs at the thinnest wall.
            skin_t_top = wing.structural.top_skin_thickness    + wing.structural.stringer_smeared_thickness
            skin_t_bot = wing.structural.bottom_skin_thickness + wing.structural.stringer_smeared_thickness
            t_min = np.minimum(np.minimum(skin_t_top, skin_t_bot),
                                np.minimum(wing.structural.front_spar.t_web, wing.structural.rear_spar.t_web))
            Am_enclosed  = w_box_arr * h_arr
            shear_stress = np.abs(M_torsion) / (2.0 * Am_enclosed * t_min)

            margin_normal = wing.structural.material.yield_tensile_strength / np.maximum(normal_stress, 1e-6) - 1.0
            margin_shear  = wing.structural.material.yield_shear_strength   / np.maximum(shear_stress,  1e-6) - 1.0
            margin_of_safety = np.minimum(margin_normal, margin_shear)

            # store results
            # Build spanwise loading directly from VLM strips (sum over chordwise panels per strip)
            n_sw_wing     = int(VD.n_sw[vd_ti][vd_idx])
            n_cw_wing     = int(VD.n_cw[vd_ti][vd_idx])
            aero_Fz_2d    = aero_loads[:, 2].reshape(n_sw_wing, n_cw_wing)
            aero_Y_2d     = aero_pts[:, 1].reshape(n_sw_wing, n_cw_wing)
            vlm_strip_Fz  = aero_Fz_2d.sum(axis=1)
            vlm_strip_Y   = aero_Y_2d.mean(axis=1)
            vlm_dy        = np.empty(n_sw_wing)
            vlm_dy[1:-1]  = (vlm_strip_Y[2:] - vlm_strip_Y[:-2]) / 2.0
            vlm_dy[0]     =  vlm_strip_Y[1]  - vlm_strip_Y[0]
            vlm_dy[-1]    =  vlm_strip_Y[-1] - vlm_strip_Y[-2]
            vlm_strip_wz  = vlm_strip_Fz / np.maximum(vlm_dy, 1e-6)
            node_aero_loads = np.interp(VD_structural_wing.y_local, vlm_strip_Y, vlm_strip_wz)

            structural_results[wing.tag].load[ti,:,2]           = node_aero_loads
            structural_results[wing.tag].elastic_twist[ti,:,0]  = twist_local
            structural_results[wing.tag].deflection[ti,:,0]     = u_full[0::6]  # X deflection (Chordwise)
            structural_results[wing.tag].deflection[ti,:,1]     = u_full[1::6]  # Y deflection (Spanwise)
            structural_results[wing.tag].deflection[ti,:,2]     = u_full[2::6]  # Z deflection (Vertical)
            structural_results[wing.tag].normal_stress[ti,:]    = normal_stress
            structural_results[wing.tag].shear_stress[ti,:]     = shear_stress
            structural_results[wing.tag].margin_of_safety[ti,:] = margin_of_safety
 
            vd_idx += 1
            if wing.xz_plane_symmetric:
                vd_idx += 1
            
    return structural_results