import numpy as np
import pyvista as pv

# --- Set Global PyVista Theme ---
pv.global_theme.font.family = 'times'
pv.global_theme.font.label_size = 14
pv.global_theme.font.title_size = 16

def get_cross_section_corners(res, scale=1.0, undeformed=False):
    y_loc, c = res['y_local'], res['chord']
    X0, Y0, Z0 = res['X0'], res['Y0'], res['Z0']
    Tw_geo = res['twist_geo']
    spar_f = res['spar_f']
    spar_r = res['spar_r']
    
    if undeformed:
        W, Tw_elas = np.zeros_like(res['deflection']), np.zeros_like(res['twist_elas'])
    else:
        W, Tw_elas = res['deflection'], res['twist_elas']
        
    p = res['params']
    n = len(y_loc)
    corners = np.zeros((4, n, 3))
    
    for i in range(n):
        h = c[i] * p['tc']
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

def build_components(res, scale=1.0, undeformed=False):
    corners = get_cross_section_corners(res, scale, undeformed)
    n = corners.shape[1]
    
    def make_strip(pts1, pts2):
        grid = pv.StructuredGrid()
        grid.points = np.vstack((pts1, pts2))
        grid.dimensions = [n, 2, 1]
        if not undeformed: 
            # Map deflection value to the points (not absolute Z)
            deflection_array = np.concatenate([res['deflection'], res['deflection']])
            grid.point_data["Deflection"] = deflection_array
        return grid

    # 1. Main Surfaces
    skin_top = make_strip(corners[0], corners[1])
    skin_bot = make_strip(corners[3], corners[2])
    spar_f_web = make_strip(corners[0], corners[3]) 
    spar_r_web = make_strip(corners[1], corners[2]) 
    
    # 2. Dynamic Spar Caps
    caps_list = []
    def make_cap_strip(line_pts, width, offset_type='center'):
        pts_L, pts_R = line_pts.copy(), line_pts.copy()
        if offset_type == 'center': 
            pts_L[:, 0] -= width/2; pts_R[:, 0] += width/2
        elif offset_type == 'inward_front': 
            pts_R[:, 0] += width
        elif offset_type == 'inward_rear':  
            pts_L[:, 0] -= width
        return make_strip(pts_L, pts_R)

    f_type, f_w = res['params']['Front_Spar']['type'], res['params']['Front_Spar']['w_cap']
    if f_type == 'I_Beam':
        caps_list.extend([make_cap_strip(corners[0], f_w, 'center'), make_cap_strip(corners[3], f_w, 'center')])
    elif f_type == 'C_Channel':
        caps_list.extend([make_cap_strip(corners[0], f_w, 'inward_front'), make_cap_strip(corners[3], f_w, 'inward_front')])

    r_type, r_w = res['params']['Rear_Spar']['type'], res['params']['Rear_Spar']['w_cap']
    if r_type == 'I_Beam':
        caps_list.extend([make_cap_strip(corners[1], r_w, 'center'), make_cap_strip(corners[2], r_w, 'center')])
    elif r_type == 'C_Channel':
        caps_list.extend([make_cap_strip(corners[1], r_w, 'inward_rear'), make_cap_strip(corners[2], r_w, 'inward_rear')])

    mesh_caps = caps_list[0]
    for c in caps_list[1:]: mesh_caps = mesh_caps.merge(c)

    # 3. Ribs
    ribs = pv.PolyData()
    if not undeformed:
        rib_locs = np.arange(0, res['y_local'][-1] + 0.01, res['params']['Rib_Spacing'])
        for y_r in rib_locs:
            rib_pts = np.zeros((4, 3))
            for k in range(4): 
                rib_pts[k, 0] = np.interp(y_r, res['y_local'], corners[k, :, 0])
                rib_pts[k, 1] = np.interp(y_r, res['y_local'], corners[k, :, 1])
                rib_pts[k, 2] = np.interp(y_r, res['y_local'], corners[k, :, 2])
            face = [4, 0, 1, 2, 3]
            poly = pv.PolyData(rib_pts, [face])
            ribs = ribs.merge(poly)
            
    # 4. Load Vectors (Quarter Chord)
    arrows_mesh = pv.PolyData()
    if not undeformed:
        num_arrows = 100    # Change this to add/remove arrows
        spar_path_len = res['y_local'][-1]
        arrow_y_locs = np.linspace(0, spar_path_len, num_arrows)
        
        qc_pts = np.zeros((num_arrows, 3))
        vectors = np.zeros((num_arrows, 3))
        
        for i, y_a in enumerate(arrow_y_locs):
            # Interpolate the physics at this arrow location
            c_a = np.interp(y_a, res['y_local'], res['chord'])
            tw_g_a = np.interp(y_a, res['y_local'], res['twist_geo'])
            tw_e_a = np.interp(y_a, res['y_local'], res['twist_elas'])
            x0_a = np.interp(y_a, res['y_local'], res['X0'])
            y0_a = np.interp(y_a, res['y_local'], res['Y0'])
            z0_a = np.interp(y_a, res['y_local'], res['Z0'])
            def_a = np.interp(y_a, res['y_local'], res['deflection'])
            load_a = np.interp(y_a, res['y_local'], res['w_z_load'])
            
            # Interpolate spar locations to find the local box center
            sf_a = np.interp(y_a, res['y_local'], res['spar_f'])
            sr_a = np.interp(y_a, res['y_local'], res['spar_r'])
            box_center_a = (sf_a + sr_a) / 2
            load_center_frac = box_center_a # Change to 0.25 For Quarter Chord
            offset_frac = load_center_frac - box_center_a
            
            # Calculate local coordinates
            local_x_qc = c_a * offset_frac
            local_z_qc = (c_a * res['params']['tc']) / 2 
            
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
        arrows_mesh = point_cloud.glyph(orient='forces', scale='forces', factor=0.002)

    return skin_top, skin_bot, spar_f_web, spar_r_web, mesh_caps, ribs, arrows_mesh

# Main Plotter

def plot_wingbox(res, scale=1.0):
    
    # 1. Build Geometry
    st, sb, sf, sr, caps, ribs, arrows = build_components(res, scale, undeformed=False)
    st0, sb0, sf0, sr0, caps0, _, _ = build_components(res, scale, undeformed=True)
    ghost_mesh = st0.merge([sb0, sf0, sr0, caps0])
    
    p = pv.Plotter()
    p.set_background('white')
    
    # Set dynamic limits for the color bar
    min_def = 0.0
    max_def = np.max(res['deflection'])
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
    p.add_mesh(ghost_mesh, color='grey', opacity=0.1, style='wireframe')
    
    # Skins
    p.add_mesh(st, cmap=cmap, clim=[min_def, max_def], opacity=0.6, show_edges=False, show_scalar_bar=True, scalar_bar_args=sbar_args)
    p.add_mesh(sb, cmap=cmap, clim=[min_def, max_def], opacity=0.6, show_edges=False, show_scalar_bar=False)
    
    # Spar Webs 
    p.add_mesh(sf, color="#444444", opacity=0.8)
    p.add_mesh(sr, color='#444444', opacity=0.8)
    
    # Spar Caps 
    p.add_mesh(caps, color='black', opacity=1.0)
    
    # Ribs 
    p.add_mesh(ribs, color='orange', opacity=1.0, show_edges=True, line_width=2)
    
    # Load Vectors (Arrows)
    p.add_mesh(arrows, color='cyan', opacity=0.5, label='Applied Lift')
    
    # Title
    p.add_text(f"Version 10: Wingbox Deflection Model\nFront: {res['params']['Front_Spar']['type']}\nRear: {res['params']['Rear_Spar']['type']}\nRib Spacing: {res['params']['Rib_Spacing']}m", 
               font_size=11, font='times', color='grey', position='upper_left')
    
    p.view_isometric()
    
    # p.show_grid(color='black', fmt='%.1f', n_zlabels=3)
    p.show_axes()
    p.show()