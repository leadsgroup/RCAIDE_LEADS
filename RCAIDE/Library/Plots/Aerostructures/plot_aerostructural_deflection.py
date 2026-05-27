import numpy as np
import pyvista as pv

# --- Set Global PyVista Theme ---
pv.global_theme.font.family = 'times'
pv.global_theme.font.label_size = 14
pv.global_theme.font.title_size = 16


def plot_aerostructural_deflection(vehicle,structural_results, cpt = 0, scale=1.0):
    
    for wing in vehicle.wings:
        plot_wing_deflection(structural_results,wing,cpt,scale)

def plot_wing_deflection(structural_results,wing,cpt,scale):
    
    # 1. Build Geometry
    st, sb, sf, sr, caps, ribs, arrows = build_components(structural_results,wing,cpt, scale, undeformed=False)
    st0, sb0, sf0, sr0, caps0, _, _    = build_components(structural_results, wing,cpt, scale, undeformed=True)
    ghost_mesh = st0.merge([sb0, sf0, sr0, caps0])
    
    plot = pv.Plotter()
    plot.set_background('white')
    
    # Set dynamic limits for the color bar
    min_def = 0.0
    max_def = np.max(structural_results[wing.tag].deflection[cpt,:,2])
    cmap = 'turbo'
    
    sbar_args = {
        'title': "Vertical Deflection (m)",
        'n_labels': 5,
        'interactive': False,
        'label_font_size': 14,
        'title_font_size': 16
    }
    
    # 2. Add Meshes
    # Ghost
    plot.add_mesh(ghost_mesh, color='grey', opacity=0.1, style='wireframe')
    
    # Skins
    plot.add_mesh(st, cmap=cmap, clim=[min_def, max_def], opacity=0.6, show_edges=False, show_scalar_bar=True, scalar_bar_args=sbar_args)
    plot.add_mesh(sb, cmap=cmap, clim=[min_def, max_def], opacity=0.6, show_edges=False, show_scalar_bar=False)
    
    # Spar Webs 
    plot.add_mesh(sf, color="#444444", opacity=0.8)
    plot.add_mesh(sr, color='#444444', opacity=0.8)
    
    # Spar Caps 
    if caps.n_points > 0:
        plot.add_mesh(caps, color='black', opacity=1.0)
    
    # Ribs 
    plot.add_mesh(ribs, color='orange', opacity=1.0, show_edges=True, line_width=2)
    
    # Load Vectors (Arrows)
    plot.add_mesh(arrows, color='cyan', opacity=0.5, label='Applied Lift') 
    
    plot.view_isometric()
     
    plot.show_axes()
    plot.show() 
    
    return 
    
    
def build_components(structural_results,wing,cpt, scale=1.0, undeformed=False):
    
    res = structural_results[wing.tag]
    
    corners = get_cross_section_corners(res,cpt,scale, undeformed)
    n       = corners.shape[1]
    
    def make_strip(pts1, pts2):
        grid = pv.StructuredGrid()
        grid.points = np.vstack((pts1, pts2))
        grid.dimensions = [n, 2, 1]
        if not undeformed: 
            # Map deflection value to the points (not absolute Z)
            deflection_array = np.concatenate([res.deflection[cpt, :, 2], res.deflection[cpt, :, 2]])
            grid.point_data["Deflection"] = deflection_array
        return grid

    # 1. Main Surfaces
    skin_top   = make_strip(corners[0], corners[1])
    skin_bot   = make_strip(corners[3], corners[2])
    spar_f_web = make_strip(corners[0], corners[3]) 
    spar_r_web = make_strip(corners[1], corners[2]) 
    
    # 2. Dynamic Spar Caps
    caps_list = []
    mesh_caps = pv.PolyData() # Initialize an empty mesh for caps
    
    def make_cap_strip(line_pts, width, offset_type='center'):
        pts_L, pts_R = line_pts.copy(), line_pts.copy()
        if offset_type == 'center': 
            pts_L[:, 0] -= width/2; pts_R[:, 0] += width/2
        elif offset_type == 'inward_front': 
            pts_R[:, 0] += width
        elif offset_type == 'inward_rear':  
            pts_L[:, 0] -= width
        return make_strip(pts_L, pts_R)  
    

    f_type, f_w = wing.structural.front_spar.type, wing.structural.front_spar.w_cap 
    if f_type == 'I_Beam':
        caps_list.extend([make_cap_strip(corners[0], f_w, 'center'), make_cap_strip(corners[3], f_w, 'center')])
    elif f_type == 'C_Channel':
        caps_list.extend([make_cap_strip(corners[0], f_w, 'inward_front'), make_cap_strip(corners[3], f_w, 'inward_front')])

    r_type, r_w = wing.structural.rear_spar.type ,wing.structural.rear_spar.w_cap
    if r_type == 'I_Beam':
        caps_list.extend([make_cap_strip(corners[1], r_w, 'center'), make_cap_strip(corners[2], r_w, 'center')])
    elif r_type == 'C_Channel':
        caps_list.extend([make_cap_strip(corners[1], r_w, 'inward_rear'), make_cap_strip(corners[2], r_w, 'inward_rear')])

    # Merge only if we actually generated caps
    if len(caps_list) > 0:
        mesh_caps = caps_list[0]
        for c in caps_list[1:]: 
            mesh_caps = mesh_caps.merge(c)

    # 3. Ribs
    ribs = pv.PolyData()
    if not undeformed:
        rib_locs = np.arange(0, res.structural_node_data.y_local[-1] + 0.01, wing.structural.rib_spacing)
        for y_r in rib_locs:
            rib_pts = np.zeros((4, 3))
            for k in range(4): 
                rib_pts[k, 0] = np.interp(y_r, res.structural_node_data.y_local, corners[k, :, 0])
                rib_pts[k, 1] = np.interp(y_r, res.structural_node_data.y_local, corners[k, :, 1])
                rib_pts[k, 2] = np.interp(y_r, res.structural_node_data.y_local, corners[k, :, 2])
            face = [4, 0, 1, 2, 3]
            poly = pv.PolyData(rib_pts, [face])
            ribs = ribs.merge(poly)
            
    # 4. Load Vectors (Quarter Chord)
    arrows_mesh = pv.PolyData()
    if not undeformed:
        num_arrows = 100    # Change this to add/remove arrows
        spar_path_len = res.structural_node_data.y_local[-1]
        arrow_y_locs = np.linspace(0, spar_path_len, num_arrows)
        
        qc_pts = np.zeros((num_arrows, 3))
        vectors = np.zeros((num_arrows, 3))
        
        for i, y_a in enumerate(arrow_y_locs):
            # Interpolate the physics at this arrow location
            c_a    = np.interp(y_a, res.structural_node_data.y_local, res.structural_node_data.chord_nodes )
            tw_g_a = np.interp(y_a, res.structural_node_data.y_local, res.structural_node_data.twist_nodes)
            tw_e_a = np.interp(y_a, res.structural_node_data.y_local, res.elastic_twist[cpt, :, 0])
            x0_a   = np.interp(y_a, res.structural_node_data.y_local, res.structural_node_data.X_nodes)
            y0_a   = np.interp(y_a, res.structural_node_data.y_local, res.structural_node_data.Y_nodes)
            z0_a   = np.interp(y_a, res.structural_node_data.y_local, res.structural_node_data.Z_nodes)
            t_c_a  = np.interp(y_a, res.structural_node_data.y_local, res.structural_node_data.t_c_nodes ) 
            def_a  = np.interp(y_a, res.structural_node_data.y_local, res.deflection[cpt, :, 2])
            load_a = np.interp(y_a, res.structural_node_data.y_local, res.load[cpt,:, 2])
            
            # Interpolate spar locations to find the local box center
            sf_a             = np.interp(y_a, res.structural_node_data.y_local, res.structural_node_data.spar_f_nodes)
            sr_a             = np.interp(y_a, res.structural_node_data.y_local,  res.structural_node_data.spar_r_nodes)
            box_center_a     = (sf_a + sr_a) / 2
            load_center_frac = box_center_a # Change to 0.25 For Quarter Chord
            offset_frac      = load_center_frac - box_center_a
            
            # Calculate local coordinates
            local_x_qc = c_a * offset_frac
            local_z_qc = (c_a * t_c_a) / 2  
            
            
            # Apply rotation
            theta = tw_g_a + (tw_e_a * scale)
            co, si = np.cos(theta), np.sin(theta)
            rot_x = local_x_qc * co + local_z_qc * si
            rot_z = -local_x_qc * si + local_z_qc * co
            
            # Map to Global 3D space
            qc_pts[i, 0] = x0_a + rot_x
            qc_pts[i, 1] = y0_a
            qc_pts[i, 2] = z0_a + rot_z + (def_a * scale)
            
            # Vector magnitude based on load
            vectors[i, 2] = load_a
            
        point_cloud = pv.PolyData(qc_pts)
        point_cloud['forces'] = vectors
        arrows_mesh = point_cloud.glyph(orient='forces', scale='forces', factor=0.00002)

    return skin_top, skin_bot, spar_f_web, spar_r_web, mesh_caps, ribs, arrows_mesh

    
def get_cross_section_corners(res, cpt, scale=1.0, undeformed=False):
    y_loc  = res.structural_node_data.y_local 
    c      = res.structural_node_data.chord_nodes 
    X0     = res.structural_node_data.X_nodes
    Y0     = res.structural_node_data.Y_nodes
    Z0     = res.structural_node_data.Z_nodes 
    Tw_geo = res.structural_node_data.twist_nodes
    spar_f = res.structural_node_data.spar_f_nodes
    spar_r = res.structural_node_data.spar_r_nodes
    t_c    = res.structural_node_data.t_c_nodes
     
    if undeformed:
        W         = np.zeros_like(res.deflection[cpt,:,2])
        Tw_elas   = np.zeros_like(res.elastic_twist[cpt,:,0])
    else:
        W         = res.deflection[cpt, :, 2]
        Tw_elas   = res.elastic_twist[cpt,:,0]
          
    n = len(y_loc)
    corners = np.zeros((4, n, 3))
    
    for i in range(n):
        h = c[i] * t_c[i]
        w = c[i] * (spar_r[i] - spar_f[i])
        
        local_x = np.array([-w/2,  w/2,  w/2, -w/2])
        local_z = np.array([ h/2,  h/2, -h/2, -h/2])
        
        theta = Tw_geo[i] + (Tw_elas[i] * scale)
        co, si = np.cos(theta), np.sin(theta)
        
        rot_x = local_x * co + local_z * si
        rot_z = -local_x * si + local_z * co
        
        corners[:, i, 0] = X0[i] + rot_x
        corners[:, i, 1] = Y0[i]
        corners[:, i, 2] = Z0[i] + rot_z + (W[i] * scale)
        
    return corners

