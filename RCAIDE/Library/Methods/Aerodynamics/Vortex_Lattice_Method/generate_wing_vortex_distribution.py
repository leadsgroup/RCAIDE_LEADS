# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/generate_wing_vortex_distribution.py
# 
# Created:  May 2018, M. Clarke
# Modified: Apr 2020, M. Clarke
#           Jun 2021, A. Blaufox
#           Apr 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                  import Data 
from RCAIDE.Library.Methods.Geometry.Airfoil                import compute_naca_4series, import_airfoil_geometry
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments
  
# package imports 
import numpy as np


# ----------------------------------------------------------------------
#  Discretize Wings
# ----------------------------------------------------------------------
def generate_wing_vortex_distribution(VD,wing,n_cw,n_sw,spc,precision):
    """ This generates vortex distribution points for the given wing 

    Assumptions:  
    
    For control surfaces, "positve" deflection corresponds to the RH rule where the axis of rotation is the OUTBOARD-pointing hinge vector
    symmetry: the LH rule is applied to the reflected surface for non-ailerons. Ailerons follow a RH rule for both sides
    
    The hinge_vector will only ever be calcualted on the first strip of any control/all-moving surface. It is assumed that all control
    surfaces are trapezoids, thus needing only one hinge, and that all all-moving surfaces have exactly one point of rotation.

    Source:   
    None
    
    Inputs:   
    VD                   - vortex distribution 
    
    Properties Used:
    N/A
    """  
    # get geometry of wing  
    span          = wing.spans.projected
    xz_sym        = wing.xz_plane_symmetric 
    xy_sym        = wing.xy_plane_symmetric 
    yz_sym        = wing.yz_plane_symmetric  
    wing_origin   = wing.origin[0]
    VD.vortex_lift.append(wing.vortex_lift)

    # determine if vehicle has symmetry 
    if xz_sym is True :
        span = span/2
        VD.vortex_lift.append(wing.vortex_lift)
        
    VD.counter  +=1
    wing.surface_ID = VD.counter*1

    # ---------------------------------------------------------------------------------------
    # STEP 3: Get discretization control variables  
    # --------------------------------------------------------------------------------------- 
    # get y_coordinates (y-locations of the edges of each strip in wing-local coords)
    if spc == True: # discretize wing using cosine spacing     
        n               = np.linspace(n_sw+1,0,n_sw+1)         # vectorize
        thetan          = n*(np.pi/2)/(n_sw+1)                 # angular stations 
        y_coordinates   = span*np.cos(thetan)   # y locations based on the angular spacing           
    else:    
        y_coordinates  = np.linspace(0,span,n_sw+1)  # y locations based on the angular spacing
 
    # ---------------------------------------------------------------------------------------
    # STEP 4: Setup span_break and section arrays. A 'section' is the trapezoid between two 
    # ---------------------------------------------------------------------------------------  
    del_y = y_coordinates[1:] - y_coordinates[:-1]   
    
    # -------------------------------------------------------------------------------------------------------------
    # Run the strip contruction loop again if wing is symmetric. 
    # Reflection plane = x-y plane for vertical wings. Otherwise, reflection plane = x-z plane 
    symmetry_mask = np.array([[0,0,0],[yz_sym,xz_sym,xy_sym]])
    signs         = np.array([1, -1])  
    side_idx = 0
    for sym_sign in signs[[True,xz_sym]]:

        yz_sym_sign   = 1 if symmetry_mask[side_idx,0] == 0 else -1
        xz_sym_sign   = 1 if symmetry_mask[side_idx,1] == 0 else -1
        xy_sym_sign   = 1 if symmetry_mask[side_idx,2] == 0 else -1
        
        # create empty vectors for coordinates 
        xah_w  = np.zeros((n_sw,n_cw))
        yah_w  = np.zeros((n_sw,n_cw))
        zah_w  = np.zeros((n_sw,n_cw))
        xbh_w  = np.zeros((n_sw,n_cw))
        ybh_w  = np.zeros((n_sw,n_cw))
        zbh_w  = np.zeros((n_sw,n_cw))   
        xch_w  = np.zeros((n_sw,n_cw))
        ych_w  = np.zeros((n_sw,n_cw))
        zch_w  = np.zeros((n_sw,n_cw))   
        xa1_w  = np.zeros((n_sw,n_cw))
        ya1_w  = np.zeros((n_sw,n_cw))
        za1_w  = np.zeros((n_sw,n_cw))
        xa2_w  = np.zeros((n_sw,n_cw))
        ya2_w  = np.zeros((n_sw,n_cw))
        za2_w  = np.zeros((n_sw,n_cw))   
        xb1_w  = np.zeros((n_sw,n_cw))
        yb1_w  = np.zeros((n_sw,n_cw))
        zb1_w  = np.zeros((n_sw,n_cw))
        xb2_w  = np.zeros((n_sw,n_cw))
        yb2_w  = np.zeros((n_sw,n_cw))
        zb2_w  = np.zeros((n_sw,n_cw))   
        xac_w  = np.zeros((n_sw,n_cw))
        yac_w  = np.zeros((n_sw,n_cw))
        zac_w  = np.zeros((n_sw,n_cw))   
        xbc_w  = np.zeros((n_sw,n_cw))
        ybc_w  = np.zeros((n_sw,n_cw))
        zbc_w  = np.zeros((n_sw,n_cw)) 
        xc_w   = np.zeros((n_sw,n_cw))
        yc_w   = np.zeros((n_sw,n_cw))
        zc_w   = np.zeros((n_sw,n_cw))
        x_w    = np.zeros((n_sw+1,n_cw+1)) # may have to change to make space for split if control surfaces are allowed to have more than two Segments
        y_w    = np.zeros((n_sw+1,n_cw+1)) 
        z_w    = np.zeros((n_sw+1,n_cw+1)) 
        x_c    = np.zeros((n_sw+1,n_cw)) 
        y_c    = np.zeros((n_sw+1,n_cw)) 
        z_c    = np.zeros((n_sw+1,n_cw))
        x_h    = np.zeros((n_sw+1,n_cw)) 
        y_h    = np.zeros((n_sw+1,n_cw)) 
        z_h    = np.zeros((n_sw+1,n_cw))           
        cs_ws  = np.zeros(n_sw+1) 
             
        seg_idx       = 0
        wing_seg_tags = list(wing.segments.keys())
        for y_i in range(len(y_coordinates)):
            delta_y = y_coordinates[y_i]
            eta_y   = delta_y / span
            
            # Step 1. check to see if y-location of horseshoe vortex is within current wing segment  
            inboard_segment  = wing.segments[wing_seg_tags[seg_idx]]
            outboard_segment = wing.segments[wing_seg_tags[seg_idx+1]]
            
            try: 
                if delta_y == outboard_segment.percent_span_location * span:
                    pass
                else:
                    while delta_y >= outboard_segment.percent_span_location * span: 
                        seg_idx += 1
                        inboard_segment = wing.segments[wing_seg_tags[seg_idx]]
                        outboard_segment = wing.segments[wing_seg_tags[seg_idx+1]]
            except:
                raise AssertionError("Insufficient surface panelization discretization. Increase number of spanwise and chordwise vortices.")
                
            # Step 2. get segment properties  
            segment_root_chord    = wing.chords.root*inboard_segment.root_chord_percent
            segment_tip_chord     = wing.chords.root*outboard_segment.root_chord_percent 
            segment_root_twist    = inboard_segment.twist
            segment_tip_twist     = outboard_segment.twist 
            inboard_segment_y     = inboard_segment.percent_span_location * span   
            outboard_segment_y    = outboard_segment.percent_span_location * span 
            segment_span          = outboard_segment_y -  inboard_segment_y   
            local_y_val           = delta_y - inboard_segment_y
            local_percent_y_val   = local_y_val /segment_span 
            local_chord           = segment_root_chord -  (segment_root_chord - segment_tip_chord) *  local_y_val / segment_span
            local_twist           = segment_root_twist -  (segment_root_twist - segment_tip_twist) *  local_y_val / segment_span
            
            # Step 3. get global shift 
            # guarantee that all segments have leading edge sweep
            if inboard_segment.sweeps.leading_edge is None:
                old_sweep                 = inboard_segment.sweeps.quarter_chord
                new_sweep                 = convert_sweep_segments(old_sweep, inboard_segment, outboard_segment, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=0.0)
                inboard_segment.sweeps.leading_edge = new_sweep 
                
            dihedral = inboard_segment.dihedral_outboard
            LE_sweep = inboard_segment.sweeps.leading_edge
            delta_z  = inboard_segment.origin[0][2] +  np.tan(dihedral) * local_y_val
            delta_x  = inboard_segment.origin[0][0] +  np.tan(LE_sweep) * local_y_val 
             
            # Step 4. get points of airfoil
            airfoil_x_pts_0 , airfoil_z_pts_0 = generate_interplated_airfoil_points(inboard_segment, outboard_segment,local_percent_y_val, ncpts=100)  
             

            airfoil_x_pts_1   = np.linspace(0, 1, n_cw+1) 
            del_x         = np.diff(airfoil_x_pts_1)
            airfoil_x_pts_1_h = (airfoil_x_pts_1[:-1] + del_x*0.25) 
            airfoil_x_pts_1_c = (airfoil_x_pts_1[:-1] + del_x*0.75) 
            
            # lines 701-710 (dimensionalizing x)
            airfoil_x_pts_2   = delta_x + airfoil_x_pts_1*local_chord 
            airfoil_x_pts_2_h = delta_x + airfoil_x_pts_1_h*local_chord 
            airfoil_x_pts_2_c = delta_x + airfoil_x_pts_1_c*local_chord  
            
            # lines 731 - 740 (dimensionalizing z)
            airfoil_z_pts_2   =  np.interp(airfoil_x_pts_1, airfoil_x_pts_0, airfoil_z_pts_0)*local_chord
            airfoil_z_pts_2_h =  np.interp(airfoil_x_pts_1_h, airfoil_x_pts_0, airfoil_z_pts_0)*local_chord
            airfoil_z_pts_2_c =  np.interp(airfoil_x_pts_1_c, airfoil_x_pts_0, airfoil_z_pts_0)*local_chord

            # lines 743 to 753 
            airfoil_z_pts_3 = airfoil_z_pts_2 + delta_z
            airfoil_z_pts_3_h = airfoil_z_pts_2_h + delta_z
            airfoil_z_pts_3_c = airfoil_z_pts_2_c + delta_z
            
            # lines 755 to 758 
            pivot_x = delta_x
            pivot_z = delta_z 


            # lines 776 to 785 
            airfoil_x_pts_4  = pivot_x + np.cos(local_twist)*(airfoil_x_pts_2-pivot_x) + np.sin(local_twist)*( airfoil_z_pts_3- pivot_z) 
            airfoil_x_pts_4_h  = pivot_x + np.cos(local_twist)*(airfoil_x_pts_2_h-pivot_x) + np.sin(local_twist)*( airfoil_z_pts_3_h- pivot_z) 
            airfoil_x_pts_4_c  = pivot_x + np.cos(local_twist)*(airfoil_x_pts_2_c-pivot_x) + np.sin(local_twist)*( airfoil_z_pts_3_c- pivot_z) 

            # lines 787 - lines 796  
            airfoil_z_pts_4   = pivot_z - np.sin(local_twist)*(airfoil_x_pts_2-pivot_x) + np.cos(local_twist)*( airfoil_z_pts_3- pivot_z)
            airfoil_z_pts_4_h = pivot_z - np.sin(local_twist)*(airfoil_x_pts_2_h-pivot_x) + np.cos(local_twist)*( airfoil_z_pts_3_h- pivot_z)
            airfoil_z_pts_4_c = pivot_z - np.sin(local_twist)*(airfoil_x_pts_2_c-pivot_x) + np.cos(local_twist)*( airfoil_z_pts_3_c- pivot_z)       
              
 
            # Step 6. get control surface deflection and modify airfoil 
            LE_angle          = 0.0
            LE_chord_fraction = 0.0
            TE_angle          = 0.0 
            TE_chord_fraction = 0.0
            
            for cs in wing.control_surfaces: 
                if (eta_y >= cs.span_fraction_start) and  (eta_y <= cs.span_fraction_end):
                    if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler:
                        continue
                    elif type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat: 
                        LE_angle          = cs.deflection  
                        LE_chord_fraction = cs.chord_fraction
                    elif type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron: 
                        TE_angle          = -cs.deflection * sym_sign 
                        TE_chord_fraction = cs.chord_fraction 
                    elif type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder: 
                        TE_angle          = -cs.deflection * xz_sym_sign 
                        TE_chord_fraction = cs.chord_fraction                        
                    else:
                        TE_angle          = cs.deflection 
                        TE_chord_fraction = cs.chord_fraction 
                
            airfoil_x_pts_5, airfoil_z_pts_5 = apply_control_surface_deflections(airfoil_x_pts_4, airfoil_z_pts_4,LE_angle, LE_chord_fraction, TE_angle, TE_chord_fraction)
            airfoil_x_pts_5_c, airfoil_z_pts_5_c = apply_control_surface_deflections(airfoil_x_pts_4_c, airfoil_z_pts_4_c,LE_angle, LE_chord_fraction, TE_angle, TE_chord_fraction)
            airfoil_x_pts_5_h, airfoil_z_pts_5_h = apply_control_surface_deflections(airfoil_x_pts_4_h, airfoil_z_pts_4_h,LE_angle, LE_chord_fraction, TE_angle, TE_chord_fraction)      
             
            # store coordinates of panels, horseshoeces vortices and control points relative to wing root----------
            if wing.vertical:
                x_w[y_i,:] = airfoil_x_pts_5 
                y_w[y_i,:] = airfoil_z_pts_5
                z_w[y_i,:] = np.ones(len(airfoil_x_pts_5))*delta_y 
                x_c[y_i,:] = airfoil_x_pts_5_c 
                y_c[y_i,:] = airfoil_z_pts_5_c 
                z_c[y_i,:] = np.ones(len(airfoil_x_pts_5_c))*delta_y 
                x_h[y_i,:] = airfoil_x_pts_5_h 
                y_h[y_i,:] = airfoil_z_pts_5_h 
                z_h[y_i,:] = np.ones(len(airfoil_x_pts_5_h))*delta_y           
                
            else:
                x_w[y_i,:] = airfoil_x_pts_5 
                y_w[y_i,:] = np.ones(len(airfoil_x_pts_5))*delta_y 
                z_w[y_i,:] = airfoil_z_pts_5
                x_c[y_i,:] = airfoil_x_pts_5_c 
                y_c[y_i,:] = np.ones(len(airfoil_x_pts_5_c))*delta_y 
                z_c[y_i,:] = airfoil_z_pts_5_c 
                x_h[y_i,:] = airfoil_x_pts_5_h 
                y_h[y_i,:] = np.ones(len(airfoil_x_pts_5_h))*delta_y 
                z_h[y_i,:] = airfoil_z_pts_5_h           

            # # handle flips for symmetric wings 
            # if side_idx == 1:
            #     x_w[y_i,:] = x_w[y_i,:] * yz_sym_sign
            #     y_w[y_i,:] = y_w[y_i,:] * xz_sym_sign
            #     z_w[y_i,:] = z_w[y_i,:] * xy_sym_sign  
            
            cs_ws[y_i] = local_chord
            
            if delta_y != outboard_segment.percent_span_location * span:
                # store this strip's discretization information--------------------------------------------------------
                LE_inds        = np.full(n_cw, False)
                TE_inds        = np.full(n_cw, False)
                LE_inds[0]     = True
                TE_inds[-1]    = True
                
                RNMAX                      = np.ones(n_cw, np.int16)*n_cw
                panel_numbers              = np.linspace(1,n_cw,n_cw, dtype=np.int16)     
                exposed_leading_edge_flag  = 1 
                            
                VD.panels_per_strip          = np.append(VD.panels_per_strip         , RNMAX                    )
                VD.chordwise_panel_number    = np.append(VD.chordwise_panel_number   , panel_numbers            )  
                VD.leading_edge_indices      = np.append(VD.leading_edge_indices     , LE_inds                  ) 
                VD.trailing_edge_indices     = np.append(VD.trailing_edge_indices    , TE_inds                  )  
                VD.exposed_leading_edge_flag = np.append(VD.exposed_leading_edge_flag, exposed_leading_edge_flag)            
            
        cs_w  = (cs_ws[:1] +  cs_ws[1:]) / 2
        
        # adjust origin for symmetry with special case for vertical symmetry 
        if side_idx == 1: 


            xa1_w = x_w[:-1,:-1]* yz_sym_sign
            ya1_w = y_w[:-1,:-1]* xz_sym_sign
            za1_w = z_w[:-1,:-1]* xy_sym_sign
            xa2_w = x_w[:-1,1:]* yz_sym_sign
            ya2_w = y_w[:-1,1:]* xz_sym_sign
            za2_w = z_w[:-1,1:]* xy_sym_sign
            xb1_w = x_w[1:,:-1]* yz_sym_sign
            yb1_w = y_w[1:,:-1]* xz_sym_sign
            zb1_w = z_w[1:,:-1]* xy_sym_sign
            xb2_w = x_w[1:,1:]* yz_sym_sign
            yb2_w = y_w[1:,1:]* xz_sym_sign
            zb2_w = z_w[1:,1:]* xy_sym_sign
            xac_w = x_c[:-1]* yz_sym_sign
            yac_w = y_c[:-1]* xz_sym_sign  
            zac_w = z_c[:-1]* xy_sym_sign
            xbc_w = x_c[1:]* yz_sym_sign
            ybc_w = y_c[1:]* xz_sym_sign 
            zbc_w = z_c[1:]* xy_sym_sign
            xah_w = x_h[:-1]* yz_sym_sign
            yah_w = y_h[:-1]* xz_sym_sign
            zah_w = z_h[:-1]* xy_sym_sign
            xbh_w = x_h[1:]* yz_sym_sign
            ybh_w = y_h[1:]* xz_sym_sign
            zbh_w = z_h[1:]* xy_sym_sign
            xch_w = (xah_w + xbh_w)/2
            ych_w = (yah_w + ybh_w)/2
            zch_w = (zah_w + zbh_w)/2  
            xc_w  = (xac_w + xbc_w)/2  
            yc_w  = (yac_w + ybc_w)/2  
            zc_w  = (zac_w + zbc_w)/2  

            wing_origin_x = wing_origin[0]* yz_sym_sign
            wing_origin_y = wing_origin[1]* xz_sym_sign
            wing_origin_z = wing_origin[2]* xy_sym_sign
        else: 

            xa1_w = x_w[:-1,:-1]
            ya1_w = y_w[:-1,:-1]
            za1_w = z_w[:-1,:-1]
            xa2_w = x_w[:-1,1:]
            ya2_w = y_w[:-1,1:]
            za2_w = z_w[:-1,1:]
            xb1_w = x_w[1:,:-1]
            yb1_w = y_w[1:,:-1]
            zb1_w = z_w[1:,:-1]
            xb2_w = x_w[1:,1:]
            yb2_w = y_w[1:,1:]
            zb2_w = z_w[1:,1:]  
            xac_w = x_c[:-1] 
            yac_w = y_c[:-1]   
            zac_w = z_c[:-1]
            xbc_w = x_c[1:] 
            ybc_w = y_c[1:]  
            zbc_w = z_c[1:] 
            xah_w = x_h[:-1]
            yah_w = y_h[:-1]
            zah_w = z_h[:-1]
            xbh_w = x_h[1:] 
            ybh_w = y_h[1:] 
            zbh_w = z_h[1:] 
            xch_w = (xah_w + xbh_w)/2
            ych_w = (yah_w + ybh_w)/2
            zch_w = (zah_w + zbh_w)/2  
            xc_w  = (xac_w + xbc_w)/2  
            yc_w  = (yac_w + ybc_w)/2  
            zc_w  = (zac_w + zbc_w)/2  
        
            wing_origin_x = wing_origin[0]  
            wing_origin_y = wing_origin[1]  
            wing_origin_z = wing_origin[2] 
            
        # adjusting coordinate axis so reference point is at the nose of the aircraft------------------------------
        xah = xah_w.flatten() + wing_origin_x # x coordinate of left corner of bound vortex 
        yah = yah_w.flatten() + wing_origin_y # y coordinate of left corner of bound vortex 
        zah = zah_w.flatten() + wing_origin_z # z coordinate of left corner of bound vortex 
        xbh = xbh_w.flatten() + wing_origin_x # x coordinate of right corner of bound vortex 
        ybh = ybh_w.flatten() + wing_origin_y # y coordinate of right corner of bound vortex 
        zbh = zbh_w.flatten() + wing_origin_z # z coordinate of right corner of bound vortex 
        xch = xch_w.flatten() + wing_origin_x # x coordinate of center of bound vortex on panel
        ych = ych_w.flatten() + wing_origin_y # y coordinate of center of bound vortex on panel
        zch = zch_w.flatten() + wing_origin_z # z coordinate of center of bound vortex on panel   
        xa1 = xa1_w.flatten() + wing_origin_x # x coordinate of top left corner of panel
        ya1 = ya1_w.flatten() + wing_origin_y # y coordinate of bottom left corner of panel
        za1 = za1_w.flatten() + wing_origin_z # z coordinate of top left corner of panel
        xa2 = xa2_w.flatten() + wing_origin_x # x coordinate of bottom left corner of panel
        ya2 = ya2_w.flatten() + wing_origin_y # y coordinate of bottom left corner of panel
        za2 = za2_w.flatten() + wing_origin_z # z coordinate of bottom left corner of panel   
        xb1 = xb1_w.flatten() + wing_origin_x # x coordinate of top right corner of panel  
        yb1 = yb1_w.flatten() + wing_origin_y # y coordinate of top right corner of panel 
        zb1 = zb1_w.flatten() + wing_origin_z # z coordinate of top right corner of panel 
        xb2 = xb2_w.flatten() + wing_origin_x # x coordinate of bottom rightcorner of panel 
        yb2 = yb2_w.flatten() + wing_origin_y # y coordinate of bottom rightcorner of panel 
        zb2 = zb2_w.flatten() + wing_origin_z # z coordinate of bottom right corner of panel  
        xac = xac_w.flatten() + wing_origin_x  # x coordinate of control points on panel
        yac = yac_w.flatten() + wing_origin_y  # y coordinate of control points on panel
        zac = zac_w.flatten() + wing_origin_z  # z coordinate of control points on panel
        xbc = xbc_w.flatten() + wing_origin_x  # x coordinate of control points on panel
        ybc = ybc_w.flatten() + wing_origin_y  # y coordinate of control points on panel
        zbc = zbc_w.flatten() + wing_origin_z  # z coordinate of control points on panel 
        xc  =  xc_w.flatten()  + wing_origin_x  # x coordinate of control points on panel
        yc  =  yc_w.flatten()  + wing_origin_y  # y coordinate of control points on panel
        zc  =  zc_w.flatten()  + wing_origin_z  # y coordinate of control points on panel
        x   =   x_w.flatten()   + wing_origin_x  # x coordinate of control points on panel
        y   =   y_w.flatten()   + wing_origin_y  # y coordinate of control points on panel
        z   =   z_w.flatten()   + wing_origin_z  # y coordinate of control points on panel
        
        # VD discretization information----------------------------------------------------------------------------
        
        # increment number of wings and panels
        n_panels = len(xch)
        VD.n_w  += 1             
        VD.n_cp += n_panels 
        
        # store this wing's discretization information  
        first_panel_ind  = VD.XAH.size
        first_strip_ind  = VD.chordwise_breaks.size
        chordwise_breaks = first_panel_ind + np.arange(n_panels)[0::n_cw]
        ID               = VD.counter*1
        
        VD.chordwise_breaks = np.append(VD.chordwise_breaks, np.int32(chordwise_breaks))
        VD.spanwise_breaks  = np.append(VD.spanwise_breaks , np.int32(first_strip_ind ))            
        VD.n_sw             = np.append(VD.n_sw            , np.int16(n_sw)            )
        VD.n_cw             = np.append(VD.n_cw            , np.int16(n_cw)            )
        VD.surface_ID       = np.append(VD.surface_ID      , np.ones(n_cw*n_sw)*ID*xz_sym_sign) # Update me when the loop is gone
        VD.surface_ID_full  = np.append(VD.surface_ID_full , np.ones((n_cw+1)*(n_sw+1))*ID*xz_sym_sign) # Update me when the loop is gone    
                
        # ---------------------------------------------------------------------------------------
        # STEP 7: Store wing in vehicle vector
        # --------------------------------------------------------------------------------------- 
        VD.XAH    = np.append(VD.XAH  , np.array(xah  , dtype=precision))
        VD.YAH    = np.append(VD.YAH  , np.array(yah  , dtype=precision))
        VD.ZAH    = np.append(VD.ZAH  , np.array(zah  , dtype=precision))
        VD.XBH    = np.append(VD.XBH  , np.array(xbh  , dtype=precision))
        VD.YBH    = np.append(VD.YBH  , np.array(ybh  , dtype=precision))
        VD.ZBH    = np.append(VD.ZBH  , np.array(zbh  , dtype=precision))
        VD.XCH    = np.append(VD.XCH  , np.array(xch  , dtype=precision))
        VD.YCH    = np.append(VD.YCH  , np.array(ych  , dtype=precision))
        VD.ZCH    = np.append(VD.ZCH  , np.array(zch  , dtype=precision))            
        VD.XA1    = np.append(VD.XA1  , np.array(xa1  , dtype=precision))
        VD.YA1    = np.append(VD.YA1  , np.array(ya1  , dtype=precision))
        VD.ZA1    = np.append(VD.ZA1  , np.array(za1  , dtype=precision))
        VD.XA2    = np.append(VD.XA2  , np.array(xa2  , dtype=precision))
        VD.YA2    = np.append(VD.YA2  , np.array(ya2  , dtype=precision))
        VD.ZA2    = np.append(VD.ZA2  , np.array(za2  , dtype=precision))        
        VD.XB1    = np.append(VD.XB1  , np.array(xb1  , dtype=precision))
        VD.YB1    = np.append(VD.YB1  , np.array(yb1  , dtype=precision))
        VD.ZB1    = np.append(VD.ZB1  , np.array(zb1  , dtype=precision))
        VD.XB2    = np.append(VD.XB2  , np.array(xb2  , dtype=precision))                
        VD.YB2    = np.append(VD.YB2  , np.array(yb2  , dtype=precision))        
        VD.ZB2    = np.append(VD.ZB2  , np.array(zb2  , dtype=precision)) 
        VD.XAC    = np.append(VD.XAC  , np.array(xac  , dtype=precision))
        VD.YAC    = np.append(VD.YAC  , np.array(yac  , dtype=precision)) 
        VD.ZAC    = np.append(VD.ZAC  , np.array(zac  , dtype=precision)) 
        VD.XBC    = np.append(VD.XBC  , np.array(xbc  , dtype=precision))
        VD.YBC    = np.append(VD.YBC  , np.array(ybc  , dtype=precision)) 
        VD.ZBC    = np.append(VD.ZBC  , np.array(zbc  , dtype=precision))  
        VD.XC     = np.append(VD.XC   , np.array(xc   , dtype=precision))
        VD.YC     = np.append(VD.YC   , np.array(yc   , dtype=precision))
        VD.ZC     = np.append(VD.ZC   , np.array(zc   , dtype=precision))  
        VD.X      = np.append(VD.X    , np.array(x    , dtype=precision))
        VD.Y      = np.append(VD.Y    , np.array(y    , dtype=precision))
        VD.Z      = np.append(VD.Z    , np.array(z    , dtype=precision))         
        VD.CS     = np.append(VD.CS   , np.array(cs_w , dtype=precision)) 
        VD.DY     = np.append(VD.DY   , np.array(del_y, dtype=precision))
        
        side_idx += 1
    
    if ( xz_sym == True) or (yz_sym == True)  or (xy_sym == True ):
        sym = 1
    else:
        sym = 0
        
    VD.symmetric_wings = np.append(VD.symmetric_wings, int(sym))
    
    if wing.vertical:
        VD.vertical_wing =  np.append(VD.vertical_wing, int(1))
    else:
        VD.vertical_wing =  np.append(VD.vertical_wing, int(0))
    
    area = wing.areas.projected
    VD.wing_areas.append(area/(sym+1))
    if sym:
        VD.wing_areas.append(area/(sym+1))
        
    # Pack wing data
    wing.n_sw = n_sw
    wing.n_cw = n_cw    
    
    return VD

def generate_interplated_airfoil_points(inboard_segment,outboard_segment,local_percent_y_val,ncpts): 
        """ Takes in two airfoils, interpolates between their coordinates to generate new
        airfoil geometries and saves new airfoil files.
        
        Assumptions: Linear geometric transition between airfoils
        
        Source: None
        
        Inputs:
        a1                 first airfoil                                [ airfoil ]
        a2                 second airfoil                               [ airfoil ]
        nairfoils          number of airfoils                           [ unitless ]
        
        """
        
        # Get points of inboard segment airfoil 
        if inboard_segment.airfoil: 
            if type(inboard_segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil: 
                a_geo_1 = compute_naca_4series(inboard_segment.airfoil.NACA_4_Series_code,ncpts*2-1)
            else:
                a_geo_1 = import_airfoil_geometry(inboard_segment.airfoil.coordinate_file,ncpts*2+1)   
        else:
            a_geo_1 =  Data()
            a_geo_1.camber_coordinates = np.zeros(ncpts)              
            a_geo_1.x_upper_surface    = np.linspace(0,1,ncpts)
    
    
        # Get points of outboard segment airfoil     
        if outboard_segment.airfoil: 
            if type(outboard_segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil: 
                a_geo_2 = compute_naca_4series(outboard_segment.airfoil.NACA_4_Series_code,ncpts*2-1)
            else:
                a_geo_2 = import_airfoil_geometry(outboard_segment.airfoil.coordinate_file,ncpts*2+1)   
        else:
            a_geo_2 =  Data()
            a_geo_2.camber_coordinates = np.zeros(ncpts)              
            a_geo_2.x_upper_surface    = np.linspace(0,1,ncpts)
                
        
        # for each point around the airfoil, interpolate between the two given airfoil coordinates
        z = np.linspace(0,1,ncpts)
        
        y_u_lb = a_geo_1.camber_coordinates 
        y_u_ub = a_geo_2.camber_coordinates  
        x_u_lb = a_geo_1.x_upper_surface 
        x_u_ub = a_geo_2.x_upper_surface    
        
        # broadcasting interpolation
        cambers = (z[None,...] * (y_u_ub[...,None] - y_u_lb[...,None]) + (y_u_lb[...,None])).T 
        x_vals  = (z[None,...] * (x_u_ub[...,None] - x_u_lb[...,None]) + (x_u_lb[...,None])).T  
           
        idx = int(local_percent_y_val *ncpts)
        
        if idx == len(cambers):
            idx -= 1
        airfoil_z_coords = cambers[idx]
        airfoil_x_coords = x_vals[idx]
         
        return airfoil_x_coords, airfoil_z_coords
    
    
def apply_control_surface_deflections(airfoil_x_pts_2, airfoil_z_pts_2,LE_angle, LE_chord_fraction, TE_angle, TE_chord_fraction):

    chord = airfoil_x_pts_2[-1]
    LE_chord_loc = chord*LE_chord_fraction
    TE_chord_loc = chord*(1-TE_chord_fraction)
 
    LE_hinge_distance = LE_chord_loc - airfoil_x_pts_2
    LE_hinge_distance[airfoil_x_pts_2>LE_chord_loc] = 0

    TE_hinge_distance = airfoil_x_pts_2 - TE_chord_loc 
    TE_hinge_distance[airfoil_x_pts_2<TE_chord_loc] = 0

    LE_x_deflection = LE_chord_loc - np.tan(LE_angle)*LE_hinge_distance
    LE_x_deflection[airfoil_x_pts_2>LE_chord_loc] = 0
    TE_x_deflection = - np.sin(TE_angle)*TE_hinge_distance 
    TE_x_deflection[airfoil_x_pts_2<TE_chord_loc] = 0

    LE_z_deflection = -np.sin(LE_angle)*LE_hinge_distance
    LE_z_deflection[airfoil_x_pts_2>LE_chord_loc] = 0
    TE_z_deflection = -np.sin(TE_angle)*TE_hinge_distance
    TE_z_deflection[airfoil_x_pts_2<TE_chord_loc] = 0
    
    Total_CS_z_deflection = LE_z_deflection + TE_z_deflection
    Total_CS_x_deflection = LE_x_deflection - TE_x_deflection

    # shift points of airfoil by control surface deflection 
    airfoil_x_pts_3 = airfoil_x_pts_2 + Total_CS_x_deflection
    airfoil_z_pts_3 = airfoil_z_pts_2 + Total_CS_z_deflection        

    return airfoil_x_pts_3, airfoil_z_pts_3