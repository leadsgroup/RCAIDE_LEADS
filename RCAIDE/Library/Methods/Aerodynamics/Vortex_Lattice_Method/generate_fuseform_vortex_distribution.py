# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/generate_fuseform_vortex_distribution.py
# 
# Created:  Apr 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                  import Data  
  
# package imports 
import numpy as np
# ----------------------------------------------------------------------
#  generate_fuseform_vortex_distribution
# ----------------------------------------------------------------------
def generate_fuseform_vortex_distribution(VD,fus,n_cw,n_sw,spc,precision):
    """ This generates the vortex distribution points on a fuselage or nacelle component
    Assumptions: 
    If nacelle has segments defined, the mean width and height of the nacelle is used
    Source:   
    None
    
    Inputs:   
    VD                   - vortex distribution
    
    Properties Used:
    N/A
    """    
    # geometry values 
    xz_sym     = True
    xy_sym     = False
    yz_sym     = False
 
    VD.vortex_lift.append(False)
    VD.vortex_lift.append(False)  
    VD.counter  +=1 
 
    # ------------------------------------------------
    # Horizontal Fuselage Sections 
    # ------------------------------------------------
    fhs        = Data()        
    fhs.origin = np.zeros((n_sw+1,3))        
    fhs.chord  = np.zeros((n_sw+1))         
    fhs.sweep  = np.zeros((n_sw+1))

    fhs_xa1 = np.zeros(n_cw*n_sw)
    fhs_ya1 = np.zeros(n_cw*n_sw)
    fhs_za1 = np.zeros(n_cw*n_sw)
    fhs_xa2 = np.zeros(n_cw*n_sw)
    fhs_ya2 = np.zeros(n_cw*n_sw)
    fhs_za2 = np.zeros(n_cw*n_sw)
    fhs_xb1 = np.zeros(n_cw*n_sw)
    fhs_yb1 = np.zeros(n_cw*n_sw)
    fhs_zb1 = np.zeros(n_cw*n_sw)
    fhs_yb2 = np.zeros(n_cw*n_sw)
    fhs_xb2 = np.zeros(n_cw*n_sw)
    fhs_zb2 = np.zeros(n_cw*n_sw)
    fhs_xah = np.zeros(n_cw*n_sw)
    fhs_yah = np.zeros(n_cw*n_sw)
    fhs_zah = np.zeros(n_cw*n_sw)
    fhs_xbh = np.zeros(n_cw*n_sw)
    fhs_ybh = np.zeros(n_cw*n_sw)
    fhs_zbh = np.zeros(n_cw*n_sw)
    fhs_xch = np.zeros(n_cw*n_sw)
    fhs_ych = np.zeros(n_cw*n_sw)
    fhs_zch = np.zeros(n_cw*n_sw)
    fhs_xc  = np.zeros(n_cw*n_sw)
    fhs_yc  = np.zeros(n_cw*n_sw)
    fhs_zc  = np.zeros(n_cw*n_sw)
    fhs_xac = np.zeros(n_cw*n_sw)
    fhs_yac = np.zeros(n_cw*n_sw)
    fhs_zac = np.zeros(n_cw*n_sw)
    fhs_xbc = np.zeros(n_cw*n_sw)
    fhs_ybc = np.zeros(n_cw*n_sw)
    fhs_zbc = np.zeros(n_cw*n_sw)    
    fhs_x   = np.zeros((n_cw+1)*(n_sw+1))
    fhs_y   = np.zeros((n_cw+1)*(n_sw+1))
    fhs_z   = np.zeros((n_cw+1)*(n_sw+1))      
    

    # ------------------------------------------------
    # Vertical Fuselage Sections 
    # ------------------------------------------------         
    fvs        = Data() 
    fvs.origin = np.zeros((n_sw+1,3))
    fvs.chord  = np.zeros((n_sw+1)) 
    fvs.sweep  = np.zeros((n_sw+1))   

    # Compute the curvature of the nose/tail given fineness ratio. Curvature is derived from general quadratic equation
    # This method relates the fineness ratio to the quadratic curve formula via a spline fit interpolation
    if isinstance(fus, RCAIDE.Library.Components.Fuselages.Fuselage): 
        vec1               = [2 , 1.5, 1.2 , 1]
        vec2               = [1  ,1.57 , 3.2,  8]
        x_sec                 = np.linspace(0,1,4)
        fus_nose_curvature =  np.interp(np.interp(fus.fineness.nose,vec2,x_sec), x_sec , vec1)
        fus_tail_curvature =  np.interp(np.interp(fus.fineness.tail,vec2,x_sec), x_sec , vec1)
        semispan_h = fus.width * 0.5
        si         = np.arange(1,((n_sw*2)+2))
        spacing    = np.cos((2*si - 1)/(2*len(si))*np.pi)
        h_array    = semispan_h*spacing[0:int((len(si)+1)/2)][::-1] 
        
        if spc == True: # discretize wing using cosine spacing     
            n               = np.linspace(n_sw+1,0,n_sw+1)         # vectorize
            thetan          = n*(np.pi/2)/(n_sw+1)                 # angular stations 
            h_array   = semispan_h*np.cos(thetan)       
        else:    
            h_array  = np.linspace(0,semispan_h,n_sw+1)   

        for i in range(n_sw+1): 
            fhs.nose_length   = ((1 - ((abs(h_array[i]/semispan_h))**fus_nose_curvature ))**(1/fus_nose_curvature))*fus.lengths.nose
            fhs.tail_length   = ((1 - ((abs(h_array[i]/semispan_h))**fus_tail_curvature ))**(1/fus_tail_curvature))*fus.lengths.tail
            fhs.nose_origin   = fus.lengths.nose - fhs.nose_length
            fhs.origin[i][:]  =  np.array([0 , h_array[i], 0.])  
            fhs.chord[i]      = fus.lengths.total  
            
        fhs.sweep[:] = np.concatenate([np.arctan((fhs.origin[:,0][1:] - fhs.origin[:,0][:-1])/(fhs.origin[:,1][1:]  - fhs.origin[:,1][:-1])) ,np.zeros(1)]) 

    # ---------------------------------------------------------------------------------------
    # STEP 9: Define coordinates of panels horseshoe vortices and control points  
    # ---------------------------------------------------------------------------------------        
    fhs_eta_a = h_array[:-1] 
    fhs_eta_b = h_array[1:]            
    fhs_del_y = h_array[1:] - h_array[:-1]
    fhs_eta   = h_array[1:] - fhs_del_y/2  
    fhs_cs = np.concatenate([fhs.chord,fhs.chord]) 
    
    fus_h_area = 0 

    symmetry_mask = np.array([[0,0,0],[False,True,False]])
    signs         = np.array([1, -1])  
    side_idx = 0
    for sym_sign in signs[[True,True]]: 

        yz_sym_sign   = 1 if symmetry_mask[side_idx,0] == 0 else -1
        xz_sym_sign   = 1 if symmetry_mask[side_idx,1] == 0 else -1
        xy_sym_sign   = 1 if symmetry_mask[side_idx,2] == 0 else -1
        
        for idx_y in range(n_sw):  
            idx_x = np.arange(n_cw)
            
            # ------------------------------------------------
            # Horizontal Fuselage Sections 
            # ------------------------------------------------
            delta_x_a = fhs.chord[idx_y]/n_cw      
            delta_x_b = fhs.chord[idx_y + 1]/n_cw    
            delta_x   = (fhs.chord[idx_y]+fhs.chord[idx_y + 1])/(2*n_cw)

            fhs_xi_a1 = fhs.origin[idx_y][0] + delta_x_a*idx_x                    # x coordinate of top left corner of panel
            fhs_xi_ah = fhs.origin[idx_y][0] + delta_x_a*idx_x + delta_x_a*0.25   # x coordinate of left corner of panel
            fhs_xi_a2 = fhs.origin[idx_y][0] + delta_x_a*idx_x + delta_x_a        # x coordinate of bottom left corner of bound vortex 
            fhs_xi_ac = fhs.origin[idx_y][0] + delta_x_a*idx_x + delta_x_a*0.75   # x coordinate of bottom left corner of control point vortex  
            fhs_xi_b1 = fhs.origin[idx_y+1][0] + delta_x_b*idx_x                  # x coordinate of top right corner of panel      
            fhs_xi_bh = fhs.origin[idx_y+1][0] + delta_x_b*idx_x + delta_x_b*0.25 # x coordinate of right corner of bound vortex         
            fhs_xi_b2 = fhs.origin[idx_y+1][0] + delta_x_b*idx_x + delta_x_b      # x coordinate of bottom right corner of panel
            fhs_xi_bc = fhs.origin[idx_y+1][0] + delta_x_b*idx_x + delta_x_b*0.75 # x coordinate of bottom right corner of control point vortex         
            fhs_xi_c  = (fhs.origin[idx_y][0] + fhs.origin[idx_y+1][0])/2  + delta_x*idx_x + delta_x*0.75   # x coordinate three-quarter chord control point for each panel
            fhs_xi_ch = (fhs.origin[idx_y][0] + fhs.origin[idx_y+1][0])/2  + delta_x*idx_x + delta_x*0.25   # x coordinate center of bound vortex of each panel 
    
            fhs_xa1[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_a1                       + fus.origin[0][0]  
            fhs_ya1[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta_a[idx_y]  + fus.origin[0][1]  
            fhs_za1[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]
            fhs_xa2[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_a2                       + fus.origin[0][0]  
            fhs_ya2[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta_a[idx_y]  + fus.origin[0][1] 
            fhs_za2[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]      
            fhs_xb1[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_b1                       + fus.origin[0][0]  
            fhs_yb1[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta_b[idx_y]  + fus.origin[0][1] 
            fhs_zb1[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]
            fhs_xb2[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_b2                       + fus.origin[0][0] 
            fhs_yb2[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta_b[idx_y]  + fus.origin[0][1] 
            fhs_zb2[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]       
            fhs_xah[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_ah                       + fus.origin[0][0]   
            fhs_yah[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta_a[idx_y]  + fus.origin[0][1]  
            fhs_zah[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]             
            fhs_xbh[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_bh                       + fus.origin[0][0] 
            fhs_ybh[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta_b[idx_y]  + fus.origin[0][1]  
            fhs_zbh[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]    
            fhs_xch[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_ch                       + fus.origin[0][0]  
            fhs_ych[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta[idx_y]    + fus.origin[0][1]                
            fhs_zch[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]     
            fhs_xc [idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_c                        + fus.origin[0][0]  
            fhs_yc [idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta[idx_y]    + fus.origin[0][1]  
            fhs_zc [idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]       
            fhs_xac[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_ac                       + fus.origin[0][0]  
            fhs_yac[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta_a[idx_y]  + fus.origin[0][1]
            fhs_zac[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]
            fhs_xbc[idx_y*n_cw:(idx_y+1)*n_cw]       = fhs_xi_bc                       + fus.origin[0][0]  
            fhs_ybc[idx_y*n_cw:(idx_y+1)*n_cw]       = np.ones(n_cw)*fhs_eta_b[idx_y]  + fus.origin[0][1]                             
            fhs_zbc[idx_y*n_cw:(idx_y+1)*n_cw]       = np.zeros(n_cw)                  + fus.origin[0][2]              
            fhs_x[idx_y*(n_cw+1):(idx_y+1)*(n_cw+1)] = np.concatenate([fhs_xi_a1,np.array([fhs_xi_a2[-1]])]) + fus.origin[0][0]  
            fhs_y[idx_y*(n_cw+1):(idx_y+1)*(n_cw+1)] = np.ones(n_cw+1)*fhs_eta_a[idx_y]  + fus.origin[0][1]                             
            fhs_z[idx_y*(n_cw+1):(idx_y+1)*(n_cw+1)] = np.zeros(n_cw+1)                  + fus.origin[0][2]

            fus_h_area += ((fhs.chord[idx_y]+fhs.chord[idx_y + 1])/2)*(fhs_eta_b[idx_y] - fhs_eta_a[idx_y])

            # store this strip's discretization information
            LE_inds        = np.full(n_cw, False)
            TE_inds        = np.full(n_cw, False)
            LE_inds[0]     = True
            TE_inds[-1]    = True
            
            RNMAX          = np.ones(n_cw, np.int16)*n_cw
            panel_numbers  = np.linspace(1,n_cw,n_cw, dtype=np.int16)
            exposed_leading_edge_flag = 1
            
            VD.leading_edge_indices      = np.append(VD.leading_edge_indices   , LE_inds       ) 
            VD.trailing_edge_indices     = np.append(VD.trailing_edge_indices  , TE_inds       )            
            VD.panels_per_strip          = np.append(VD.panels_per_strip       , RNMAX         )
            VD.chordwise_panel_number    = np.append(VD.chordwise_panel_number , panel_numbers )   
            VD.exposed_leading_edge_flag = np.append(VD.exposed_leading_edge_flag, exposed_leading_edge_flag)
            
        
        # ------------------------------------------------     
        # Horizontal Fuselage Sections 
        # ------------------------------------------------  
        # xyz positions for the right side of this fuselage's outermost panels
        fhs_x[-(n_cw+1):] = np.concatenate([fhs_xi_b1,np.array([fhs_xi_b2[-1]])]) 
        fhs_y[-(n_cw+1):] = np.ones(n_cw+1)*fhs_eta_b[idx_y]                  
        fhs_z[-(n_cw+1):] = np.zeros(n_cw+1)                  
        fhs_cs = (fhs.chord[:-1]+fhs.chord[1:])/2 
        
        VD.wing_areas.append(np.array(fus_h_area, dtype=precision)) 
        VD.symmetric_wings  = np.append(VD.symmetric_wings,True)   
        VD.vertical_wing    =  np.append(VD.vertical_wing, int(0)) 
        
        # store points of horizontal section of fuselage
        # adjust origin for symmetry with special case for vertical symmetry 
        if side_idx == 1:
    
            fhs_xa1 *= yz_sym_sign 
            fhs_ya1 *= xz_sym_sign 
            fhs_za1 *= xy_sym_sign
            fhs_xa2 *= yz_sym_sign 
            fhs_ya2 *= xz_sym_sign
            fhs_za2 *= xy_sym_sign     
            fhs_xb1 *= yz_sym_sign 
            fhs_yb1 *= xz_sym_sign
            fhs_zb1 *= xy_sym_sign
            fhs_xb2 *= yz_sym_sign
            fhs_yb2 *= xz_sym_sign
            fhs_zb2 *= xy_sym_sign      
            fhs_xah *= yz_sym_sign  
            fhs_yah *= xz_sym_sign 
            fhs_zah *= xy_sym_sign            
            fhs_xbh *= yz_sym_sign
            fhs_ybh *= xz_sym_sign 
            fhs_zbh *= xy_sym_sign   
            fhs_xch *= yz_sym_sign 
            fhs_ych *= xz_sym_sign               
            fhs_zch *= xy_sym_sign    
            fhs_xc  *= yz_sym_sign 
            fhs_yc  *= xz_sym_sign 
            fhs_zc  *= xy_sym_sign      
            fhs_xac *= yz_sym_sign 
            fhs_yac *= xz_sym_sign
            fhs_zac *= xy_sym_sign
            fhs_xbc *= yz_sym_sign 
            fhs_ybc *= xz_sym_sign                            
            fhs_zbc *= xy_sym_sign             
            fhs_x   *= yz_sym_sign 
            fhs_y   *= xz_sym_sign                            
            fhs_z   *= xy_sym_sign 
         
        # shift points to fuseform origin             
        fhs_xa1 += fus.origin[0][0]  
        fhs_ya1 += fus.origin[0][1]  
        fhs_za1 += fus.origin[0][2]
        fhs_xa2 += fus.origin[0][0]  
        fhs_ya2 += fus.origin[0][1] 
        fhs_za2 += fus.origin[0][2]      
        fhs_xb1 += fus.origin[0][0]  
        fhs_yb1 += fus.origin[0][1] 
        fhs_zb1 += fus.origin[0][2]
        fhs_xb2 += fus.origin[0][0] 
        fhs_yb2 += fus.origin[0][1] 
        fhs_zb2 += fus.origin[0][2]       
        fhs_xah += fus.origin[0][0]   
        fhs_yah += fus.origin[0][1]  
        fhs_zah += fus.origin[0][2]             
        fhs_xbh += fus.origin[0][0] 
        fhs_ybh += fus.origin[0][1]  
        fhs_zbh += fus.origin[0][2]    
        fhs_xch += fus.origin[0][0]  
        fhs_ych += fus.origin[0][1]                
        fhs_zch += fus.origin[0][2]     
        fhs_xc  += fus.origin[0][0]  
        fhs_yc  += fus.origin[0][1]  
        fhs_zc  += fus.origin[0][2]       
        fhs_xac += fus.origin[0][0]  
        fhs_yac += fus.origin[0][1]
        fhs_zac += fus.origin[0][2]
        fhs_xbc += fus.origin[0][0]  
        fhs_ybc += fus.origin[0][1]                             
        fhs_zbc += fus.origin[0][2]              
        fhs_x   += fus.origin[0][0]  
        fhs_y   += fus.origin[0][1]                             
        fhs_z   += fus.origin[0][2] 

        # increment fuslage lifting surface sections  
        VD.n_f    += 1    
        n_panels   = len(fhs_xch)
        VD.n_w    += 1             
        VD.n_cp   += n_panels        
        
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
        VD.XAH    = np.append(VD.XAH  , np.array(fhs_xah  , dtype=precision))
        VD.YAH    = np.append(VD.YAH  , np.array(fhs_yah  , dtype=precision))
        VD.ZAH    = np.append(VD.ZAH  , np.array(fhs_zah  , dtype=precision))
        VD.XBH    = np.append(VD.XBH  , np.array(fhs_xbh  , dtype=precision))
        VD.YBH    = np.append(VD.YBH  , np.array(fhs_ybh  , dtype=precision))
        VD.ZBH    = np.append(VD.ZBH  , np.array(fhs_zbh  , dtype=precision))
        VD.XCH    = np.append(VD.XCH  , np.array(fhs_xch  , dtype=precision))
        VD.YCH    = np.append(VD.YCH  , np.array(fhs_ych  , dtype=precision))
        VD.ZCH    = np.append(VD.ZCH  , np.array(fhs_zch  , dtype=precision))            
        VD.XA1    = np.append(VD.XA1  , np.array(fhs_xa1  , dtype=precision))
        VD.YA1    = np.append(VD.YA1  , np.array(fhs_ya1  , dtype=precision))
        VD.ZA1    = np.append(VD.ZA1  , np.array(fhs_za1  , dtype=precision))
        VD.XA2    = np.append(VD.XA2  , np.array(fhs_xa2  , dtype=precision))
        VD.YA2    = np.append(VD.YA2  , np.array(fhs_ya2  , dtype=precision))
        VD.ZA2    = np.append(VD.ZA2  , np.array(fhs_za2  , dtype=precision))        
        VD.XB1    = np.append(VD.XB1  , np.array(fhs_xb1  , dtype=precision))
        VD.YB1    = np.append(VD.YB1  , np.array(fhs_yb1  , dtype=precision))
        VD.ZB1    = np.append(VD.ZB1  , np.array(fhs_zb1  , dtype=precision))
        VD.XB2    = np.append(VD.XB2  , np.array(fhs_xb2  , dtype=precision))                
        VD.YB2    = np.append(VD.YB2  , np.array(fhs_yb2  , dtype=precision))        
        VD.ZB2    = np.append(VD.ZB2  , np.array(fhs_zb2  , dtype=precision)) 
        VD.XAC    = np.append(VD.XAC  , np.array(fhs_xac  , dtype=precision))
        VD.YAC    = np.append(VD.YAC  , np.array(fhs_yac  , dtype=precision)) 
        VD.ZAC    = np.append(VD.ZAC  , np.array(fhs_zac  , dtype=precision)) 
        VD.XBC    = np.append(VD.XBC  , np.array(fhs_xbc  , dtype=precision))
        VD.YBC    = np.append(VD.YBC  , np.array(fhs_ybc  , dtype=precision)) 
        VD.ZBC    = np.append(VD.ZBC  , np.array(fhs_zbc  , dtype=precision))  
        VD.XC     = np.append(VD.XC   , np.array(fhs_xc   , dtype=precision))
        VD.YC     = np.append(VD.YC   , np.array(fhs_yc   , dtype=precision))
        VD.ZC     = np.append(VD.ZC   , np.array(fhs_zc   , dtype=precision))  
        VD.X      = np.append(VD.X    , np.array(fhs_x    , dtype=precision))
        VD.Y      = np.append(VD.Y    , np.array(fhs_y    , dtype=precision))
        VD.Z      = np.append(VD.Z    , np.array(fhs_z    , dtype=precision))         
        VD.CS     = np.append(VD.CS   , np.array(fhs_cs , dtype=precision)) 
        VD.DY     = np.append(VD.DY   , np.array(fhs_del_y, dtype=precision))
        
        side_idx += 1 
     
    VD.symmetric_wings = np.append(VD.symmetric_wings, int(1)) 
    VD.vertical_wing   = np.append(VD.vertical_wing, int(0))
        
    return VD