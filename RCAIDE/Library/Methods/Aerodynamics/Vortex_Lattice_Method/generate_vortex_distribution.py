# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/generate_vortex_distribution.py
# 
# Created:  May 2018, M. Clarke
# Modified: Apr 2020, M. Clarke
#           Jun 2021, A. Blaufox

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core    import  Data 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.postprocess_vortex_distribution      import postprocess_vortex_distribution
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_wing_vortex_distribution         import generate_wing_vortex_distribution  
  
# package imports 
import numpy as np

# ----------------------------------------------------------------------
#  Generate Vortex Distribution
# ----------------------------------------------------------------------
def generate_vortex_distribution(conditions,settings,geometry): 
    precision      = settings.floating_point_precision    
    # ---------------------------------------------------------------------------------------
    # STEP 1: Define empty vectors for coordinates of panes, control points and bound vortices
    # ---------------------------------------------------------------------------------------
    VD_seg = Data()

    VD_seg.XAH    = np.empty(shape=[0,0], dtype=precision)
    VD_seg.YAH    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZAH    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.XBH    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YBH    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZBH    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.XCH    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YCH    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZCH    = np.empty(shape=[0,1], dtype=precision)     
    VD_seg.XA1    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YA1    = np.empty(shape=[0,1], dtype=precision)  
    VD_seg.ZA1    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.XA2    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YA2    = np.empty(shape=[0,1], dtype=precision)    
    VD_seg.ZA2    = np.empty(shape=[0,1], dtype=precision)    
    VD_seg.XB1    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YB1    = np.empty(shape=[0,1], dtype=precision)  
    VD_seg.ZB1    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.XB2    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YB2    = np.empty(shape=[0,1], dtype=precision)    
    VD_seg.ZB2    = np.empty(shape=[0,1], dtype=precision)     
    VD_seg.XAC    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YAC    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZAC    = np.empty(shape=[0,1], dtype=precision) 
    VD_seg.XBC    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YBC    = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZBC    = np.empty(shape=[0,1], dtype=precision) 
    VD_seg.XC_TE  = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YC_TE  = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZC_TE  = np.empty(shape=[0,1], dtype=precision)     
    VD_seg.XA_TE  = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YA_TE  = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZA_TE  = np.empty(shape=[0,1], dtype=precision) 
    VD_seg.XB_TE  = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YB_TE  = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZB_TE  = np.empty(shape=[0,1], dtype=precision)  
    VD_seg.XC     = np.empty(shape=[0,1], dtype=precision)
    VD_seg.YC     = np.empty(shape=[0,1], dtype=precision)
    VD_seg.ZC     = np.empty(shape=[0,1], dtype=precision)        
    VD_seg.CS     = np.empty(shape=[0,1], dtype=precision) 
    VD_seg.X      = np.empty(shape=[0,1], dtype=precision)
    VD_seg.Y      = np.empty(shape=[0,1], dtype=precision)
    VD_seg.Z      = np.empty(shape=[0,1], dtype=precision)
    VD_seg.Y_SW   = np.empty(shape=[0,1], dtype=precision)
    VD_seg.DY     = np.empty(shape=[0,1], dtype=precision)
     
    for i in range(len(conditions.aerodynamics.angles.alpha)):
        VD_i = generate_aircraft_vortex_distribution(geometry,settings)  
        if i ==  0: 
            VD_seg.XAH                       = np.atleast_2d(VD_i.XAH   ) 
            VD_seg.YAH                       = np.atleast_2d(VD_i.YAH   )
            VD_seg.ZAH                       = np.atleast_2d(VD_i.ZAH   )
            VD_seg.XBH                       = np.atleast_2d(VD_i.XBH   )
            VD_seg.YBH                       = np.atleast_2d(VD_i.YBH   )
            VD_seg.ZBH                       = np.atleast_2d(VD_i.ZBH   )
            VD_seg.XCH                       = np.atleast_2d(VD_i.XCH   )
            VD_seg.YCH                       = np.atleast_2d(VD_i.YCH   )
            VD_seg.ZCH                       = np.atleast_2d(VD_i.ZCH   )     
            VD_seg.XA1                       = np.atleast_2d(VD_i.XA1   )
            VD_seg.YA1                       = np.atleast_2d(VD_i.YA1   )  
            VD_seg.ZA1                       = np.atleast_2d(VD_i.ZA1   )
            VD_seg.XA2                       = np.atleast_2d(VD_i.XA2   )
            VD_seg.YA2                       = np.atleast_2d(VD_i.YA2   )    
            VD_seg.ZA2                       = np.atleast_2d(VD_i.ZA2   )    
            VD_seg.XB1                       = np.atleast_2d(VD_i.XB1   )
            VD_seg.YB1                       = np.atleast_2d(VD_i.YB1   )  
            VD_seg.ZB1                       = np.atleast_2d(VD_i.ZB1   )
            VD_seg.XB2                       = np.atleast_2d(VD_i.XB2   )
            VD_seg.YB2                       = np.atleast_2d(VD_i.YB2   )    
            VD_seg.ZB2                       = np.atleast_2d(VD_i.ZB2   )     
            VD_seg.XAC                       = np.atleast_2d(VD_i.XAC   )
            VD_seg.YAC                       = np.atleast_2d(VD_i.YAC   )
            VD_seg.ZAC                       = np.atleast_2d(VD_i.ZAC   ) 
            VD_seg.XBC                       = np.atleast_2d(VD_i.XBC   )
            VD_seg.YBC                       = np.atleast_2d(VD_i.YBC   )
            VD_seg.ZBC                       = np.atleast_2d(VD_i.ZBC   ) 
            VD_seg.XC_TE                     = np.atleast_2d(VD_i.XC_TE )
            VD_seg.YC_TE                     = np.atleast_2d(VD_i.YC_TE )
            VD_seg.ZC_TE                     = np.atleast_2d(VD_i.ZC_TE )     
            VD_seg.XA_TE                     = np.atleast_2d(VD_i.XA_TE )
            VD_seg.YA_TE                     = np.atleast_2d(VD_i.YA_TE )
            VD_seg.ZA_TE                     = np.atleast_2d(VD_i.ZA_TE ) 
            VD_seg.XB_TE                     = np.atleast_2d(VD_i.XB_TE )
            VD_seg.YB_TE                     = np.atleast_2d(VD_i.YB_TE )
            VD_seg.ZB_TE                     = np.atleast_2d(VD_i.ZB_TE )  
            VD_seg.XC                        = np.atleast_2d(VD_i.XC    )
            VD_seg.YC                        = np.atleast_2d(VD_i.YC    )
            VD_seg.ZC                        = np.atleast_2d(VD_i.ZC    )         
            VD_seg.CS                        = np.atleast_2d(VD_i.CS    ) 
            VD_seg.X                         = np.atleast_2d(VD_i.X     )
            VD_seg.Y                         = np.atleast_2d(VD_i.Y     )
            VD_seg.Z                         = np.atleast_2d(VD_i.Z     )
            VD_seg.Y_SW                      = np.atleast_2d(VD_i.Y_SW  )
            VD_seg.DY                        = np.atleast_2d(VD_i.DY    )  
            VD_seg.n_w                       = np.atleast_2d(VD_i.n_w                      )
            VD_seg.n_cp                      = np.atleast_2d(VD_i.n_cp                     )
            VD_seg.n_sw                      = np.atleast_2d(VD_i.n_sw                     )
            VD_seg.n_cw                      = np.atleast_2d(VD_i.n_cw                     )
            VD_seg.chordwise_breaks          = np.atleast_2d(VD_i.chordwise_breaks         )
            VD_seg.spanwise_breaks           = np.atleast_2d(VD_i.spanwise_breaks          )
            VD_seg.symmetric_wings           = np.atleast_2d(VD_i.symmetric_wings          )
            VD_seg.vertical_wing             = np.atleast_2d(VD_i.vertical_wing            )
            VD_seg.surface_ID                = np.atleast_2d(VD_i.surface_ID               )
            VD_seg.surface_ID_full           = np.atleast_2d(VD_i.surface_ID_full          )
            VD_seg.leading_edge_indices      = np.atleast_2d(VD_i.leading_edge_indices     )
            VD_seg.leading_edge_sweeps       = np.atleast_2d(VD_i.leading_edge_sweeps      )
            VD_seg.trailing_edge_indices     = np.atleast_2d(VD_i.trailing_edge_indices    )
            VD_seg.panels_per_strip          = np.atleast_2d(VD_i.panels_per_strip         )
            VD_seg.chordwise_panel_number    = np.atleast_2d(VD_i.chordwise_panel_number   )
            VD_seg.chord_lengths             = np.atleast_2d(VD_i.chord_lengths            )
            VD_seg.chord_widths              = np.atleast_2d(VD_i.chord_widths            )
            VD_seg.tangent_incidence_angle   = np.atleast_2d(VD_i.tangent_incidence_angle  )
            VD_seg.exposed_leading_edge_flag = np.atleast_2d(VD_i.exposed_leading_edge_flag) 
            VD_seg.wing_areas                = np.atleast_2d(VD_i.wing_areas               )
            VD_seg.vortex_lift               = np.atleast_2d(VD_i.vortex_lift              )
            VD_seg.counter                   = np.atleast_2d(VD_i.counter                  ) 
            VD_seg.panel_areas               = np.atleast_2d(VD_i.panel_areas              )
            VD_seg.normals                   = VD_i.normals[None,:, :]
            VD_seg.SLOPE                     = np.atleast_2d(VD_i.SLOPE                    )
            VD_seg.SLE                       = np.atleast_2d(VD_i.SLE                      )
            VD_seg.D                         = np.atleast_2d(VD_i.D                        )
            VD_seg.tangent_incidence_angle   = np.atleast_2d(VD_i.tangent_incidence_angle  )   
            
        else:

            VD_seg.XAH                       = np.vstack(( VD_seg.XAH  , np.atleast_2d(VD_i.XAH   )))
            VD_seg.YAH                       = np.vstack(( VD_seg.YAH  , np.atleast_2d(VD_i.YAH   )))
            VD_seg.ZAH                       = np.vstack(( VD_seg.ZAH  , np.atleast_2d(VD_i.ZAH   )))
            VD_seg.XBH                       = np.vstack(( VD_seg.XBH  , np.atleast_2d(VD_i.XBH   )))
            VD_seg.YBH                       = np.vstack(( VD_seg.YBH  , np.atleast_2d(VD_i.YBH   )))
            VD_seg.ZBH                       = np.vstack(( VD_seg.ZBH  , np.atleast_2d(VD_i.ZBH   )))
            VD_seg.XCH                       = np.vstack(( VD_seg.XCH  , np.atleast_2d(VD_i.XCH   )))
            VD_seg.YCH                       = np.vstack(( VD_seg.YCH  , np.atleast_2d(VD_i.YCH   )))
            VD_seg.ZCH                       = np.vstack(( VD_seg.ZCH  , np.atleast_2d(VD_i.ZCH   )))     
            VD_seg.XA1                       = np.vstack(( VD_seg.XA1  , np.atleast_2d(VD_i.XA1   )))
            VD_seg.YA1                       = np.vstack(( VD_seg.YA1  , np.atleast_2d(VD_i.YA1   )))  
            VD_seg.ZA1                       = np.vstack(( VD_seg.ZA1  , np.atleast_2d(VD_i.ZA1   )))
            VD_seg.XA2                       = np.vstack(( VD_seg.XA2  , np.atleast_2d(VD_i.XA2   )))
            VD_seg.YA2                       = np.vstack(( VD_seg.YA2  , np.atleast_2d(VD_i.YA2   )))    
            VD_seg.ZA2                       = np.vstack(( VD_seg.ZA2  , np.atleast_2d(VD_i.ZA2   )))    
            VD_seg.XB1                       = np.vstack(( VD_seg.XB1  , np.atleast_2d(VD_i.XB1   )))
            VD_seg.YB1                       = np.vstack(( VD_seg.YB1  , np.atleast_2d(VD_i.YB1   )))  
            VD_seg.ZB1                       = np.vstack(( VD_seg.ZB1  , np.atleast_2d(VD_i.ZB1   )))
            VD_seg.XB2                       = np.vstack(( VD_seg.XB2  , np.atleast_2d(VD_i.XB2   )))
            VD_seg.YB2                       = np.vstack(( VD_seg.YB2  , np.atleast_2d(VD_i.YB2   )))    
            VD_seg.ZB2                       = np.vstack(( VD_seg.ZB2  , np.atleast_2d(VD_i.ZB2   )))     
            VD_seg.XAC                       = np.vstack(( VD_seg.XAC  , np.atleast_2d(VD_i.XAC   )))
            VD_seg.YAC                       = np.vstack(( VD_seg.YAC  , np.atleast_2d(VD_i.YAC   )))
            VD_seg.ZAC                       = np.vstack(( VD_seg.ZAC  , np.atleast_2d(VD_i.ZAC   ))) 
            VD_seg.XBC                       = np.vstack(( VD_seg.XBC  , np.atleast_2d(VD_i.XBC   )))
            VD_seg.YBC                       = np.vstack(( VD_seg.YBC  , np.atleast_2d(VD_i.YBC   )))
            VD_seg.ZBC                       = np.vstack(( VD_seg.ZBC  , np.atleast_2d(VD_i.ZBC   ))) 
            VD_seg.XC_TE                     = np.vstack(( VD_seg.XC_TE, np.atleast_2d(VD_i.XC_TE )))
            VD_seg.YC_TE                     = np.vstack(( VD_seg.YC_TE, np.atleast_2d(VD_i.YC_TE )))
            VD_seg.ZC_TE                     = np.vstack(( VD_seg.ZC_TE, np.atleast_2d(VD_i.ZC_TE )))     
            VD_seg.XA_TE                     = np.vstack(( VD_seg.XA_TE, np.atleast_2d(VD_i.XA_TE )))
            VD_seg.YA_TE                     = np.vstack(( VD_seg.YA_TE, np.atleast_2d(VD_i.YA_TE )))
            VD_seg.ZA_TE                     = np.vstack(( VD_seg.ZA_TE, np.atleast_2d(VD_i.ZA_TE ))) 
            VD_seg.XB_TE                     = np.vstack(( VD_seg.XB_TE, np.atleast_2d(VD_i.XB_TE )))
            VD_seg.YB_TE                     = np.vstack(( VD_seg.YB_TE, np.atleast_2d(VD_i.YB_TE )))
            VD_seg.ZB_TE                     = np.vstack(( VD_seg.ZB_TE, np.atleast_2d(VD_i.ZB_TE )))  
            VD_seg.XC                        = np.vstack(( VD_seg.XC   , np.atleast_2d(VD_i.XC    )))
            VD_seg.YC                        = np.vstack(( VD_seg.YC   , np.atleast_2d(VD_i.YC    )))
            VD_seg.ZC                        = np.vstack(( VD_seg.ZC   , np.atleast_2d(VD_i.ZC    )))       
            VD_seg.CS                        = np.vstack(( VD_seg.CS   , np.atleast_2d(VD_i.CS    ))) 
            VD_seg.X                         = np.vstack(( VD_seg.X    , np.atleast_2d(VD_i.X     )))
            VD_seg.Y                         = np.vstack(( VD_seg.Y    , np.atleast_2d(VD_i.Y     )))
            VD_seg.Z                         = np.vstack(( VD_seg.Z    , np.atleast_2d(VD_i.Z     )))
            VD_seg.Y_SW                      = np.vstack(( VD_seg.Y_SW , np.atleast_2d(VD_i.Y_SW  )))
            VD_seg.DY                        = np.vstack(( VD_seg.DY   , np.atleast_2d(VD_i.DY    )))  
            VD_seg.n_w                       = np.vstack((VD_seg.n_w                        , np.atleast_2d(VD_i.n_w                      )))
            VD_seg.n_cp                      = np.vstack((VD_seg.n_cp                       , np.atleast_2d(VD_i.n_cp                     )))
            VD_seg.n_sw                      = np.vstack((VD_seg.n_sw                       , np.atleast_2d(VD_i.n_sw                     )))
            VD_seg.n_cw                      = np.vstack((VD_seg.n_cw                       , np.atleast_2d(VD_i.n_cw                     )))
            VD_seg.chordwise_breaks          = np.vstack((VD_seg.chordwise_breaks           , np.atleast_2d(VD_i.chordwise_breaks         )))
            VD_seg.spanwise_breaks           = np.vstack((VD_seg.spanwise_breaks            , np.atleast_2d(VD_i.spanwise_breaks          )))
            VD_seg.symmetric_wings           = np.vstack((VD_seg.symmetric_wings            , np.atleast_2d(VD_i.symmetric_wings          )))
            VD_seg.vertical_wing             = np.vstack((VD_seg.vertical_wing              , np.atleast_2d(VD_i.vertical_wing            )))
            VD_seg.surface_ID                = np.vstack((VD_seg.surface_ID                 , np.atleast_2d(VD_i.surface_ID               )))
            VD_seg.surface_ID_full           = np.vstack((VD_seg.surface_ID_full            , np.atleast_2d(VD_i.surface_ID_full          )))
            VD_seg.leading_edge_indices      = np.vstack((VD_seg.leading_edge_indices       , np.atleast_2d(VD_i.leading_edge_indices     )))
            VD_seg.leading_edge_sweeps       = np.vstack((VD_seg.leading_edge_sweeps        , np.atleast_2d(VD_i.leading_edge_sweeps      )))
            VD_seg.trailing_edge_indices     = np.vstack((VD_seg.trailing_edge_indices      , np.atleast_2d(VD_i.trailing_edge_indices    )))
            VD_seg.panels_per_strip          = np.vstack((VD_seg.panels_per_strip           , np.atleast_2d(VD_i.panels_per_strip         )))
            VD_seg.chordwise_panel_number    = np.vstack((VD_seg.chordwise_panel_number     , np.atleast_2d(VD_i.chordwise_panel_number   )))
            VD_seg.chord_lengths             = np.vstack((VD_seg.chord_lengths              , np.atleast_2d(VD_i.chord_lengths            )))
            VD_seg.chord_widths              = np.vstack((VD_seg.chord_widths              , np.atleast_2d(VD_i.chord_widths              )))
            VD_seg.tangent_incidence_angle   = np.vstack((VD_seg.tangent_incidence_angle    , np.atleast_2d(VD_i.tangent_incidence_angle  )))
            VD_seg.exposed_leading_edge_flag = np.vstack((VD_seg.exposed_leading_edge_flag  , np.atleast_2d(VD_i.exposed_leading_edge_flag)))
            VD_seg.wing_areas                = np.vstack((VD_seg.wing_areas                 , np.atleast_2d(VD_i.wing_areas               )))
            VD_seg.vortex_lift               = np.vstack((VD_seg.vortex_lift                , np.atleast_2d(VD_i.vortex_lift              )))
            VD_seg.counter                   = np.vstack((VD_seg.counter                    , np.atleast_2d(VD_i.counter                  ))) 
            VD_seg.panel_areas               = np.vstack((VD_seg.panel_areas                , np.atleast_2d(VD_i.panel_areas              )))
            VD_seg.normals                   = np.vstack((VD_seg.normals                    , VD_i.normals[None, :, :]                    ))
            VD_seg.SLOPE                     = np.vstack((VD_seg.SLOPE                      , np.atleast_2d(VD_i.SLOPE                    )))
            VD_seg.SLE                       = np.vstack((VD_seg.SLE                        , np.atleast_2d(VD_i.SLE                      )))
            VD_seg.D                         = np.vstack((VD_seg.D                          , np.atleast_2d(VD_i.D                        )))   
    
    return VD_seg 
    
    
def generate_aircraft_vortex_distribution(geometry,settings):
    ''' Compute the coordinates of panels, vortices , control points
    and geometry used to build the influence coefficient matrix. A 
    different discretization (n_sw and n_cw) may be defined for each type
    of major section (wings and fuselages). 
    
    Control surfaces are modelled as wings, but adapt their panel density 
    to that of the area in which they reside on their own wing.   

    Assumptions: 
    Below is a schematic of the coordinates of an arbitrary panel  
    
    XA1 ____________________________ XB1    
       |                            |
       |        bound vortex        |
    XAH|  ________________________  |XBH
       | |           XCH          | |
       | |                        | |
       | |                        | |     
       | |                        | |
       | |                        | |
       | |           0 <--control | |       
       | |          XC     point  | |  
       | |                        | |
   XA2 |_|________________________|_|XB2
         |                        |     
         |       trailing         |  
         |   <--  vortex   -->    |  
         |         legs           | 
             
    
    In addition, all control surfaces should be appended directly
       to the wing, not the wing segments    
    
    For control surfaces, "positve" deflection corresponds to the RH rule where the axis of rotation is the OUTBOARD-pointing hinge vector
    symmetry: the LH rule is applied to the reflected surface for non-ailerons. Ailerons follow a RH rule for both sides
    
    Source:  
    None

    Inputs:
    geometry.wings                                [Unitless]  
    settings.floating_point_precision             [np.dtype]
    
    Of the following settings, the user should define either the number_ atrributes or the wing_ and fuse_ attributes.
    settings.number_of_spanwise_vortices             - a base number of vortices to be applied to both wings and fuselages
    settings.number_of_chordwise_vortices            - a base number of vortices to be applied to both wings and fuselages 
       
    Outputs:                                   
    VD - vehicle vortex distribution              [Unitless] 

    Properties Used:
    N/A 
         
    '''
    # ---------------------------------------------------------------------------------------
    # STEP 0: Unpack settings
    # ---------------------------------------------------------------------------------------        
    #unpack other settings----------------------------------------------------
    spc            = settings.spanwise_cosine_spacing
    precision      = settings.floating_point_precision
    
    # unpack discretization settings------------------------------------------
    n_sw           = settings.number_of_spanwise_vortices
    n_cw           = settings.number_of_chordwise_vortices 
    # ---------------------------------------------------------------------------------------
    # STEP 1: Define empty vectors for coordinates of panes, control points and bound vortices
    # ---------------------------------------------------------------------------------------
    VD = Data()

    VD.XAH    = np.empty(shape=[0,1], dtype=precision)
    VD.YAH    = np.empty(shape=[0,1], dtype=precision)
    VD.ZAH    = np.empty(shape=[0,1], dtype=precision)
    VD.XBH    = np.empty(shape=[0,1], dtype=precision)
    VD.YBH    = np.empty(shape=[0,1], dtype=precision)
    VD.ZBH    = np.empty(shape=[0,1], dtype=precision)
    VD.XCH    = np.empty(shape=[0,1], dtype=precision)
    VD.YCH    = np.empty(shape=[0,1], dtype=precision)
    VD.ZCH    = np.empty(shape=[0,1], dtype=precision)     
    VD.XA1    = np.empty(shape=[0,1], dtype=precision)
    VD.YA1    = np.empty(shape=[0,1], dtype=precision)  
    VD.ZA1    = np.empty(shape=[0,1], dtype=precision)
    VD.XA2    = np.empty(shape=[0,1], dtype=precision)
    VD.YA2    = np.empty(shape=[0,1], dtype=precision)    
    VD.ZA2    = np.empty(shape=[0,1], dtype=precision)    
    VD.XB1    = np.empty(shape=[0,1], dtype=precision)
    VD.YB1    = np.empty(shape=[0,1], dtype=precision)  
    VD.ZB1    = np.empty(shape=[0,1], dtype=precision)
    VD.XB2    = np.empty(shape=[0,1], dtype=precision)
    VD.YB2    = np.empty(shape=[0,1], dtype=precision)    
    VD.ZB2    = np.empty(shape=[0,1], dtype=precision)     
    VD.XAC    = np.empty(shape=[0,1], dtype=precision)
    VD.YAC    = np.empty(shape=[0,1], dtype=precision)
    VD.ZAC    = np.empty(shape=[0,1], dtype=precision) 
    VD.XBC    = np.empty(shape=[0,1], dtype=precision)
    VD.YBC    = np.empty(shape=[0,1], dtype=precision)
    VD.ZBC    = np.empty(shape=[0,1], dtype=precision) 
    VD.XC_TE  = np.empty(shape=[0,1], dtype=precision)
    VD.YC_TE  = np.empty(shape=[0,1], dtype=precision)
    VD.ZC_TE  = np.empty(shape=[0,1], dtype=precision)     
    VD.XA_TE  = np.empty(shape=[0,1], dtype=precision)
    VD.YA_TE  = np.empty(shape=[0,1], dtype=precision)
    VD.ZA_TE  = np.empty(shape=[0,1], dtype=precision) 
    VD.XB_TE  = np.empty(shape=[0,1], dtype=precision)
    VD.YB_TE  = np.empty(shape=[0,1], dtype=precision)
    VD.ZB_TE  = np.empty(shape=[0,1], dtype=precision)  
    VD.XC     = np.empty(shape=[0,1], dtype=precision)
    VD.YC     = np.empty(shape=[0,1], dtype=precision)
    VD.ZC     = np.empty(shape=[0,1], dtype=precision)         
    VD.CS     = np.empty(shape=[0,1], dtype=precision) 
    VD.X      = np.empty(shape=[0,1], dtype=precision)
    VD.Y      = np.empty(shape=[0,1], dtype=precision)
    VD.Z      = np.empty(shape=[0,1], dtype=precision)
    VD.Y_SW   = np.empty(shape=[0,1], dtype=precision)
    VD.DY     = np.empty(shape=[0,1], dtype=precision) 

    # empty vectors necessary for arbitrary discretization dimensions
    VD.n_w              = 0                            # number of wings counter (refers to wings, fuselages or other structures)  
    VD.n_cp             = 0                            # number of bound vortices (panels) counter 
    VD.n_sw             = np.array([], dtype=np.int16) # array of the number of spanwise  strips in each wing
    VD.n_cw             = np.array([], dtype=np.int16) # array of the number of chordwise panels per strip in each wing
    VD.chordwise_breaks = np.array([], dtype=np.int32) # indices of the first panel in every strip      (given a list of all panels)
    VD.spanwise_breaks  = np.array([], dtype=np.int32) # indices of the first strip of panels in a wing (given chordwise_breaks)    
    VD.symmetric_wings  = np.array([], dtype=np.int32)
    VD.vertical_wing    = np.array([], dtype=np.int32)
    VD.surface_ID       = np.empty(shape=[0,1], dtype=np.int16) 
    VD.surface_ID_full  = np.empty(shape=[0,1], dtype=np.int16)   
    VD.leading_edge_indices      = np.array([], dtype=bool)      # bool array of leading  edge indices (all false except for panels at leading  edge)
    VD.leading_edge_sweeps       = np.array([], dtype=bool)      # bool array of leading  edge indices (all false except for panels at leading  edge)
    VD.trailing_edge_indices     = np.array([], dtype=bool)      # bool array of trailing edge indices (all false except for panels at trailing edge)    
    VD.panels_per_strip          = np.array([], dtype=np.int16)  # array of the number of panels per strip (RNMAX); this is assigned for all panels  
    VD.chordwise_panel_number    = np.array([], dtype=np.int16)  # array of panels' numbers in their strips.     
    VD.chord_lengths             = np.array([], dtype=precision) # Chord length, this is assigned for all panels.
    VD.tangent_incidence_angle   = np.array([], dtype=precision) # Tangent Incidence Angles of the chordwise strip. LE to TE, ZETA
    VD.chord_widths              = np.array([], dtype=precision) # Chord width, this is assigned for all panels.
    VD.exposed_leading_edge_flag = np.array([], dtype=np.int16)  # 0 or 1 per strip. 0 turns off leading edge suction for non-slat control surfaces
    
    # ---------------------------------------------------------------------------------------
    # Unpack aircraft wing geometry 
    # ---------------------------------------------------------------------------------------    
    VD.wing_areas  = [] # instantiate wing areas
    VD.vortex_lift = []
    VD.counter     = 0
            
    for wing in geometry.wings:         
        VD = generate_wing_vortex_distribution(VD,wing,n_cw,n_sw,spc,precision)  
                    
    # ---------------------------------------------------------------------------------------
    # Postprocess VD information
    # ---------------------------------------------------------------------------------------   
    VD = postprocess_vortex_distribution(VD, settings) 
    
    return VD 
