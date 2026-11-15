# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/generate_vortex_distribution.py
# 
# Created:  May 2018, M. Clarke
# Modified: Apr 2020, M. Clarke
#           Jun 2021, A. Blaufox

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core    import  Data, array_namespace
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.postprocess_vortex_distribution       import postprocess_vortex_distribution
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_wing_vortex_distribution     import generate_wing_vortex_distribution  
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_fuseform_vortex_distribution import generate_fuseform_vortex_distribution  
  
# package imports 
import numpy as np

# ----------------------------------------------------------------------
#  Generate Vortex Distribution
# ----------------------------------------------------------------------
def generate_vortex_distribution(conditions,settings,geometry):
    """ The vortex distribution of the aircraft, with a leading dimension for the flow cases.

    Inputs:
    conditions   flight conditions; the number of cases is that of the angles of attack
    settings     vortex lattice settings (number of vortices, spacing, fuselage model)
    geometry     vehicle

    Outputs:
    VD_seg       vortex distribution
    """
    n_cases = len(conditions.aerodynamics.angles.alpha)
    VD_i    = generate_aircraft_vortex_distribution(geometry,settings)
    return assemble_vortex_distribution(VD_i,n_cases)


def assemble_vortex_distribution(VD_i,n_cases):
    """ Gives every field of the aircraft's vortex distribution a leading dimension for the flow cases.

    Inputs:
    VD_i      vortex distribution of the aircraft (generate_aircraft_vortex_distribution)
    n_cases   number of flow cases

    Outputs:
    VD_seg    vortex distribution
    """
    VD_seg = Data()
    for key in VD_i.keys():
        value = VD_i[key]
        xp    = array_namespace(value)
        value = value[None,:,:] if key == 'normals' else xp.atleast_2d(value)
        if n_cases > 1:
            value = xp.tile(value, (n_cases,) + (1,) * (np.ndim(value) - 1))
        VD_seg[key] = value
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
    
    # unpack discretization settings------------------------------------------
    n_sw           = settings.number_of_spanwise_vortices
    n_cw           = settings.number_of_chordwise_vortices 
    n_sw_f         = settings.number_of_fuselage_spanwise_vortices
    n_cw_f         = settings.number_of_fuselage_chordwise_vortices 
    # ---------------------------------------------------------------------------------------
    # STEP 1: Define empty vectors for coordinates of panes, control points and bound vortices
    # ---------------------------------------------------------------------------------------
    VD = Data()

    # Panel-geometry fields below are accumulated as plain Python lists of
    # per-wing/per-fuselage chunks by generate_wing_vortex_distribution() and
    # generate_fuseform_vortex_distribution() (one list.append() per chunk,
    # O(1) amortized) and concatenated once at the end of this function --
    # rather than np.append()-ing onto a growing array on every chunk, which
    # is an O(total_size) copy each time (O(n^2) total across all chunks).
    CASE_FIELDS = [
        "XAH","YAH","ZAH","XBH","YBH","ZBH",
        "XCH","YCH","ZCH",
        "XA1","YA1","ZA1","XA2","YA2","ZA2",
        "XB1","YB1","ZB1","XB2","YB2","ZB2",
        "XAC","YAC","ZAC","XBC","YBC","ZBC",
        "XC","YC","ZC","X","Y","Z","CS","DY",
        "surface_ID","surface_ID_full",
        "panels_per_strip","chordwise_panel_number",
        "leading_edge_indices","trailing_edge_indices","exposed_leading_edge_flag",
    ]
    for field in CASE_FIELDS:
        VD[field] = []

    VD.XC_TE  = np.empty(shape=[0,1])
    VD.YC_TE  = np.empty(shape=[0,1])
    VD.ZC_TE  = np.empty(shape=[0,1])
    VD.XA_TE  = np.empty(shape=[0,1])
    VD.YA_TE  = np.empty(shape=[0,1])
    VD.ZA_TE  = np.empty(shape=[0,1])
    VD.XB_TE  = np.empty(shape=[0,1])
    VD.YB_TE  = np.empty(shape=[0,1])
    VD.ZB_TE  = np.empty(shape=[0,1])
    VD.Y_SW   = np.empty(shape=[0,1])

    # empty vectors necessary for arbitrary discretization dimensions
    VD.n_w              = 0                            # number of wings counter (refers to ALL surfaces)
    VD.n_f              = 0                            # number of fuselage counter (refers to ONLY fuselages)
    VD.n_cp             = 0                            # number of bound vortices (panels) counter
    VD.n_sw             = np.array([], dtype=np.int16) # array of the number of spanwise  strips in each wing
    VD.n_cw             = np.array([], dtype=np.int16) # array of the number of chordwise panels per strip in each wing
    VD.chordwise_breaks = np.array([], dtype=np.int32) # indices of the first panel in every strip      (given a list of all panels)
    VD.spanwise_breaks  = np.array([], dtype=np.int32) # indices of the first strip of panels in a wing (given chordwise_breaks)
    VD.symmetric_wings  = np.array([], dtype=np.int32)
    VD.vertical_wing    = np.array([], dtype=np.int32)
    VD.horizontal_tail  = np.array([], dtype=np.int32) # 1 for each surface side that belongs to a horizontal tail
    VD.leading_edge_sweeps       = np.array([], dtype=bool)      # bool array of leading  edge indices (all false except for panels at leading  edge)
    VD.chord_lengths             = np.array([]) # Chord length, this is assigned for all panels.
    VD.tangent_incidence_angle   = np.array([]) # Tangent Incidence Angles of the chordwise strip. LE to TE, ZETA
    VD.chord_widths              = np.array([]) # Chord width, this is assigned for all panels.

    # ---------------------------------------------------------------------------------------
    # Unpack aircraft wing geometry
    # ---------------------------------------------------------------------------------------
    VD.wing_areas  = [] # instantiate wing areas
    VD.vortex_lift = []
    VD.counter     = 0

    for wing in geometry.wings:
        VD = generate_wing_vortex_distribution(VD,wing,n_cw,n_sw,spc)

    if settings.model_fuselage:
        for fuselage in geometry.fuselages:
            VD = generate_fuseform_vortex_distribution(VD,fuselage,n_cw_f,n_sw_f,spc)

    # Concatenate the accumulated per-chunk lists into the final arrays.
    for field in CASE_FIELDS:
        VD[field] = np.concatenate(VD[field])

    # ---------------------------------------------------------------------------------------
    # Postprocess VD information
    # ---------------------------------------------------------------------------------------
    VD = postprocess_vortex_distribution(VD, settings)

    return VD
