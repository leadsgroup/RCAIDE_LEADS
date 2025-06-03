# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/create_avl_datastructures.py
#  
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import  Data ,  Units 
from RCAIDE.Library.Components.Wings.Control_Surfaces                                      import Aileron , Elevator , Slat , Flap , Rudder 
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.write_avl_airfoil_file      import write_avl_airfoil_file  
from .AVL_Objects.Wing                                                                     import Wing, Section, Control_Surface
from .AVL_Objects.Body                                                                     import Body

# package imports
import  numpy as  np


# ----------------------------------------------------------------------------------------------------------------------
#  create_avl_datastructures
# ----------------------------------------------------------------------------------------------------------------------  
def translate_avl_wing(rcaide_wing):
    """
    Translates wing geometry from RCAIDE format to AVL wing data structure.

    Parameters
    ----------
    rcaide_wing : Wing
        RCAIDE wing component containing geometric and aerodynamic properties
            - tag : str
                Wing identifier
            - symmetric : bool
                Flag indicating symmetric wing about centerline
            - vertical : bool
                Flag indicating vertical orientation (fin/rudder)
            - spans.projected : float
                Wing projected span in meters
            - origin : array_like
                Wing root origin coordinates [x, y, z] in meters
            - segments : Container
                Wing segment definitions with geometric properties

    Returns
    -------
    w : Wing
        AVL wing data structure with translated geometry and sections

    Notes
    -----
    This function serves as the primary interface for converting RCAIDE wing
    definitions into the AVL format required for vortex lattice analysis.
    The translation process handles segmented wings, airfoil definitions,
    and control surface configurations.
    """
    w                 = Wing()
    w.tag             = rcaide_wing.tag
    w.symmetric       = rcaide_wing.symmetric
    w.vertical        = rcaide_wing.vertical
    w                 = populate_wing_sections(w,rcaide_wing)

    return w

def translate_avl_body(rcaide_body):
    """
    Translates fuselage geometry from RCAIDE format to AVL body data structure.

    Parameters
    ----------
    rcaide_body : Body
        RCAIDE fuselage component containing geometric properties
            - tag : str
                Body identifier
            - lengths.total : float
                Total fuselage length in meters
            - lengths.nose : float
                Nose section length in meters
            - lengths.tail : float
                Tail section length in meters
            - width : float
                Maximum fuselage width in meters
            - heights.maximum : float
                Maximum fuselage height in meters

    Returns
    -------
    b : Body
        AVL body data structure with translated geometry and sections

    Notes
    -----
    Translates fuselage geometry for inclusion in AVL analysis. The body
    is represented by horizontal and vertical cross-sections that capture
    the three-dimensional shape for interference effects calculation.

    **Major Assumptions**
        * Fuselage has elliptical cross-sections
        * Symmetric body about vertical centerline
    """
    b                 = Body()
    b.tag             = rcaide_body.tag
    b.symmetric       = True
    b.lengths.total   = rcaide_body.lengths.total
    b.lengths.nose    = rcaide_body.lengths.nose
    b.lengths.tail    = rcaide_body.lengths.tail
    b.widths.maximum  = rcaide_body.width
    b.heights.maximum = rcaide_body.heights.maximum
    b                 = populate_body_sections(b,rcaide_body)

    return b

def populate_wing_sections(avl_wing,rcaide_wing): 
    """
    Creates wing sections and populates AVL wing data structure with geometric details.

    Parameters
    ----------
    avl_wing : Wing
        AVL wing object to be populated with section data
    rcaide_wing : Wing
        RCAIDE wing containing source geometry
            - spans.projected : float
                Wing projected span in meters
            - chords.root : float
                Root chord length in meters
            - origin : array_like
                Wing origin coordinates in meters
            - segments : Container
                Wing segments with sweep, twist, and chord definitions
            - control_surfaces : Container, optional
                Control surface definitions

    Returns
    -------
    avl_wing : Wing
        Populated AVL wing with all sections and control surfaces

    Notes
    -----
    This function handles the detailed conversion of wing geometry into
    AVL sections. It processes both segmented and non-segmented wings,
    converts sweep angles from quarter-chord to leading-edge definitions,
    and integrates control surface configurations.

    For segmented wings, each segment boundary creates a new section.
    Control surfaces create additional section breaks to properly model
    hinge lines and deflection regions.
    """           
        
    # obtain the geometry for each segment in a loop                                            
    symm                 = avl_wing.symmetric
    semispan             = rcaide_wing.spans.projected*0.5 * (2 - symm)
    avl_wing.semispan    = semispan   
    root_chord           = rcaide_wing.chords.root
    segments             = rcaide_wing.segments
    segment_names        = list(segments.keys())
    n_segments           = len(segment_names) 
    origin               = rcaide_wing.origin  
        
    if n_segments>0: 
        for i_segs in range(n_segments): 
            current_seg =  segment_names[i_segs]
            if (i_segs == n_segments-1):
                sweep = 0                                 
            else: # This converts all sweeps defined by the quarter chord to leading edge sweep since AVL needs the start of each wing section
                # from the leading edge coordinate and not the quarter chord coordinate 
                next_seg    =  segment_names[i_segs+1]
                
                if segments[current_seg].sweeps.leading_edge is not None: 
                    # If leading edge sweep is defined 
                    sweep       = segments[current_seg].sweeps.leading_edge  
                else:   
                    # If quarter chord sweep is defined, convert it to leading edge sweep
                    sweep_quarter_chord = segments[current_seg].sweeps.quarter_chord 
                    chord_fraction      = 0.25                          
                    segment_root_chord  = root_chord*segments[current_seg].root_chord_percent
                    segment_tip_chord   = root_chord*segments[next_seg].root_chord_percent
                    segment_span        = semispan*(segments[next_seg].percent_span_location - segments[current_seg].percent_span_location )
                    sweep               = np.arctan(((segment_root_chord*chord_fraction) + (np.tan(sweep_quarter_chord )*segment_span - chord_fraction*segment_tip_chord)) /segment_span) 
            dihedral       = segments[current_seg].dihedral_outboard   
    
            # append section 
            section        = Section() 
            section.tag    = segments[current_seg].tag
            section.chord  = root_chord*segments[current_seg].root_chord_percent 
            section.twist  = segments[current_seg].twist/Units.degrees    
            section.origin = origin # first origin in wing root, overwritten by section origin
            if isinstance(segments[current_seg].airfoil, RCAIDE.Library.Components.Airfoils.Airfoil):
                if type(segments[current_seg].airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                    section.naca_airfoil         = segments[current_seg].airfoil.NACA_4_Series_code
                else:
                    section.airfoil_coord_file   = write_avl_airfoil_file(segments[current_seg].airfoil.coordinate_file)
                    
            # append section to wing
            avl_wing.append_section(section)   
    
            if (i_segs == n_segments-1):
                return avl_wing 
            else:
                # condition for the presence of control surfaces in segment 
                if getattr(rcaide_wing,'control_surfaces',False):   
                    root_chord_percent = segments[current_seg].root_chord_percent
                    tip_chord_percent  = segments[next_seg].root_chord_percent
                    tip_percent_span   = segments[next_seg].percent_span_location
                    root_percent_span  = segments[current_seg].percent_span_location
                    root_twist         = segments[current_seg].twist
                    tip_twist          = segments[next_seg].twist             
                    tip_airfoil        = segments[next_seg].airfoil 
                    seg_tag            = segments[next_seg].tag 
                    
                    # append control surfaces
                    append_avl_wing_control_surfaces(rcaide_wing,avl_wing,semispan,root_chord_percent,tip_chord_percent,tip_percent_span,
                                                     root_percent_span,root_twist,tip_twist,tip_airfoil,seg_tag,dihedral,origin,sweep) 
    
            # update origin for next segment 
            segment_percent_span =    segments[next_seg].percent_span_location - segments[current_seg].percent_span_location     
            if avl_wing.vertical:
                inverted_wing = -np.sign(abs(dihedral) - np.pi/2)
                if inverted_wing  == 0:
                    inverted_wing  = 1
                dz = inverted_wing*semispan*segment_percent_span
                dy = dz*np.tan(dihedral)
                l  = dz/np.cos(dihedral)
                dx = l*np.tan(sweep)
            else:
                inverted_wing = np.sign(dihedral)
                if inverted_wing  == 0:
                    inverted_wing  = 1
                dy = inverted_wing*semispan*segment_percent_span
                dz = dy*np.tan(dihedral)
                l  = dy/np.cos(dihedral)
                dx = l*np.tan(sweep)
            origin= [[origin[0][0] + dx , origin[0][1] + dy, origin[0][2] + dz]]  
        
    else:    
        dihedral              = rcaide_wing.dihedral
        if rcaide_wing.sweeps.leading_edge  is not None: 
            sweep      = rcaide_wing.sweeps.leading_edge
        else:  
            sweep_quarter_chord = rcaide_wing.sweeps.quarter_chord 
            chord_fraction      = 0.25                          
            segment_root_chord  = rcaide_wing.chords.root
            segment_tip_chord   = rcaide_wing.chords.tip
            segment_span        = semispan 
            sweep       = np.arctan(((segment_root_chord*chord_fraction) + (np.tan(sweep_quarter_chord )*segment_span - chord_fraction*segment_tip_chord)) /segment_span)  
        avl_wing.semispan     = semispan 
        
        # define root section 
        root_section          = Section()
        root_section.tag      = 'root_section'
        root_section.origin   = origin
        root_section.chord    = rcaide_wing.chords.root 
        root_section.twist    = rcaide_wing.twists.root/Units.degrees
        
        # append control surfaces 
        tip_airfoil       = rcaide_wing.airfoil 
        seg_tag            = 'section'   
        append_avl_wing_control_surfaces(rcaide_wing,avl_wing,semispan,1.0,rcaide_wing.taper,1.0,
                                         0.0,rcaide_wing.twists.root,rcaide_wing.twists.tip,tip_airfoil,seg_tag,dihedral,origin,sweep)  
        
        # define tip section
        tip_section           = Section()
        tip_section.tag       = 'tip_section'
        tip_section.chord     = rcaide_wing.chords.tip 
        tip_section.twist     = rcaide_wing.twists.tip/Units.degrees  

        # assign location of wing tip         
        if avl_wing.vertical:
            tip_section.origin    = [[origin[0][0]+semispan*np.tan(sweep), origin[0][1]+semispan*np.tan(dihedral), origin[0][2]+semispan]]
        else: 
            tip_section.origin    = [[origin[0][0]+semispan*np.tan(sweep), origin[0][1]+semispan,origin[0][2]+semispan*np.tan(dihedral)]]

        # assign wing airfoil
        if  (rcaide_wing.airfoil !=  None) and (isinstance(rcaide_wing.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil) :
            root_section.airfoil_coord_file  = rcaide_wing.airfoil.coordinate_file          
            tip_section.airfoil_coord_file   = rcaide_wing.airfoil.coordinate_file     

        avl_wing.append_section(root_section)
        avl_wing.append_section(tip_section)
        
    return avl_wing

def append_avl_wing_control_surfaces(rcaide_wing,avl_wing,semispan,root_chord_percent,tip_chord_percent,tip_percent_span,
                                     root_percent_span,root_twist,tip_twist,tip_airfoil,seg_tag,dihedral,origin,sweep):

    """
    Converts RCAIDE control surfaces to AVL wing sections with appropriate hinge and deflection properties.

    Parameters
    ----------
    rcaide_wing : Wing
        RCAIDE wing containing control surface definitions
    avl_wing : Wing
        AVL wing to receive control surface sections
    semispan : float
        Wing semispan length in meters
    root_chord_percent : float
        Root chord as percentage of wing root chord
    tip_chord_percent : float
        Tip chord as percentage of wing root chord
    tip_percent_span : float
        Tip location as percentage of wing span
    root_percent_span : float
        Root location as percentage of wing span
    root_twist : float
        Root section twist angle in radians
    tip_twist : float
        Tip section twist angle in radians
    tip_airfoil : Airfoil
        Airfoil definition for section
    seg_tag : str
        Segment identifier tag
    dihedral : float
        Wing dihedral angle in radians
    origin : array_like
        Wing section origin coordinates
    sweep : float
        Wing sweep angle in radians

    Returns
    -------
    None
        Control surfaces are appended directly to avl_wing sections

    Notes
    -----
    This function creates wing sections at control surface boundaries and
    assigns appropriate control surface properties. Different control surface
    types (aileron, elevator, flap, etc.) receive type-specific configurations
    including deflection symmetry and gain settings.

    Section breaks are created at the beginning and end of each control surface
    to properly model the hinge lines and deflection regions in AVL analysis.

    **Major Assumptions**
        * Control surfaces span complete wing sections
        * Hinge lines are straight and aligned with wing sections
        * Control surface deflections are small angle approximations

    **Definitions**

    'Hinge Line'
        Axis about which control surface rotates during deflection

    'Control Surface Gain'
        Scaling factor relating control input to surface deflection
    """         

    root_chord    = rcaide_wing.chords.root                    
    section_spans = []
    for cs in rcaide_wing.control_surfaces:     
        # Create a vectorof all the section breaks from control surfaces on wings.
        # Section breaks include beginning and end of control surfaces as well as the end of segment       
        control_surface_start = semispan*cs.span_fraction_start
        control_surface_end   = semispan*cs.span_fraction_end
        if (control_surface_start < semispan*tip_percent_span) and (control_surface_start >= semispan*root_percent_span) : 
            section_spans.append(control_surface_start) 
        if (control_surface_end  < semispan*tip_percent_span) and (control_surface_end  >= semispan*root_percent_span):                         
            section_spans.append(control_surface_end)                                
    ordered_section_spans = sorted(list(set(section_spans)))     # sort the section_spans in order to create sections in spanwise order
    num_sections = len(ordered_section_spans)                    # count the number of sections breaks that the segment will contain    \

    for section_count in range(num_sections):        
        # create and append sections onto avl wing structure  
        if ordered_section_spans[section_count] == semispan*root_percent_span:  
            # if control surface begins at beginning of segment, redundant section is removed
            section_tags = list(avl_wing.sections.keys())
            del avl_wing.sections[section_tags[-1]]

        # create section for each break in the wing        
        section                   = Section()              
        section.tag               = seg_tag + '_section_'+ str(ordered_section_spans[section_count]) + 'm'
        root_section_chord        = root_chord*root_chord_percent
        tip_section_chord         = root_chord*tip_chord_percent
        semispan_section_fraction = (ordered_section_spans[section_count] - semispan*root_percent_span)/(semispan*(tip_percent_span - root_percent_span ))   
        section.chord             = np.interp(semispan_section_fraction,[0.,1.],[root_section_chord,tip_section_chord])
        root_section_twist        = root_twist/Units.degrees 
        tip_section_twist         = root_chord*tip_twist/Units.degrees  
        section.twist             = np.interp(semispan_section_fraction,[0.,1.],[root_section_twist,tip_section_twist]) 

        # if wing is a vertical wing, the y and z coordinates are swapped 
        if avl_wing.vertical:
            inverted_wing = -np.sign(abs(dihedral) - np.pi/2)
            if inverted_wing  == 0: inverted_wing  = 1
            dz = ordered_section_spans[section_count] -  inverted_wing*semispan*root_percent_span
            dy = dz*np.tan(dihedral)
            l  = dz/np.cos(dihedral)
            dx = l*np.tan(sweep)                                                            
        else:
            inverted_wing = np.sign(dihedral)
            if inverted_wing  == 0: inverted_wing  = 1
            dy = ordered_section_spans[section_count] - inverted_wing*semispan*root_percent_span
            dz = dy*np.tan(dihedral)
            l  = dy/np.cos(dihedral)
            dx = l*np.tan(sweep)
        section.origin = [[origin[0][0] + dx , origin[0][1] + dy, origin[0][2] + dz]]              

        # this loop appends all the control surfaces within a particular wing section
        for index  , ctrl_surf in enumerate(rcaide_wing.control_surfaces):
            if  (semispan*ctrl_surf.span_fraction_start == ordered_section_spans[section_count]) or (ordered_section_spans[section_count] == semispan*ctrl_surf.span_fraction_end):
                c                     = Control_Surface()
                c.tag                 = ctrl_surf.tag                # name of control surface   
                c.sign_duplicate      = '+1'                         # this float indicates control surface deflection symmetry
                c.x_hinge             = 1 - ctrl_surf.chord_fraction # this float is the % location of the control surface hinge on the wing 
                c.deflection          = ctrl_surf.deflection / Units.degrees 
                c.order               = index

                # if control surface is an aileron, the deflection is asymmetric. This is standard convention from AVL
                if (type(ctrl_surf) ==  Aileron):
                    c.sign_duplicate = '-1'
                    c.function       = 'aileron'
                    c.gain           = -1.0
                # if control surface is a slat, the hinge is taken from the leading edge        
                elif (type(ctrl_surf) ==  Slat):
                    c.x_hinge   =  -ctrl_surf.chord_fraction
                    c.function  = 'slat'
                    c.gain      = -1.0
                elif (type(ctrl_surf) ==  Flap):
                    c.function  = 'flap'    
                    c.gain      = 1.0
                elif (type(ctrl_surf) ==  Elevator):
                    c.function  = 'elevator'
                    c.gain      = 1.0
                elif (type(ctrl_surf) ==  Rudder):
                    c.function  = 'rudder'
                    c.gain      = 1.0
                else:
                    raise AttributeError("Define control surface function as 'slat', 'flap', 'elevator' , 'aileron' or 'rudder'")
                section.append_control_surface(c) 

            elif  (semispan*ctrl_surf.span_fraction_start < ordered_section_spans[section_count]) and (ordered_section_spans[section_count] < semispan*ctrl_surf.span_fraction_end):
                c                     = Control_Surface()
                c.tag                 = ctrl_surf.tag                # name of control surface   
                c.sign_duplicate      = '+1'                         # this float indicates control surface deflection symmetry
                c.x_hinge             = 1 - ctrl_surf.chord_fraction # this float is the % location of the control surface hinge on the wing 
                c.deflection          = ctrl_surf.deflection / Units.degrees 
                c.order               = index

                # if control surface is an aileron, the deflection is asymmetric. This is standard convention from AVL
                if (type(ctrl_surf) ==  Aileron):
                    c.sign_duplicate = '-1'
                    c.function       = 'aileron'
                    c.gain           = -1.0
                # if control surface is a slat, the hinge is taken from the leading edge        
                elif (type(ctrl_surf) ==  Slat):
                    c.x_hinge   =  -ctrl_surf.chord_fraction
                    c.function  = 'slat'
                    c.gain      = -1.0
                elif (type(ctrl_surf) ==  Flap):
                    c.function  = 'flap'    
                    c.gain      = 1.0
                elif (type(ctrl_surf) ==  Elevator):
                    c.function  = 'elevator'
                    c.gain      = 1.0
                elif (type(ctrl_surf) ==  Rudder):
                    c.function  = 'rudder'
                    c.gain      = 1.0
                else:
                    raise AttributeError("Define control surface function as 'slat', 'flap', 'elevator' , 'aileron' or 'rudder'")
                section.append_control_surface(c)                                                  

        if isinstance(tip_airfoil,RCAIDE.Library.Components.Airfoils.Airfoil):  
            if type(tip_airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                section.naca_airfoil         = tip_airfoil.NACA_4_Series_code 
            else:
                section.airfoil_coord_file   = write_avl_airfoil_file(tip_airfoil.coordinate_file) 

        avl_wing.append_section(section)  
                        
    return 
def populate_body_sections(avl_body,rcaide_body):
    """
    Creates horizontal and vertical cross-sections for AVL body representation.

    Parameters
    ----------
    avl_body : Body
        AVL body object to be populated with section data
    rcaide_body : Body
        RCAIDE body containing source geometry
            - origin : array_like
                Body origin coordinates in meters
            - fineness.nose : float
                Nose fineness ratio (length/diameter)
            - fineness.tail : float
                Tail fineness ratio (length/diameter)

    Returns
    -------
    avl_body : Body
        Populated AVL body with horizontal and vertical sections

    Notes
    -----
    Creates cross-sectional representations of the fuselage for AVL analysis.
    The function generates both horizontal and vertical cutting planes through
    the body to capture three-dimensional shape effects on aerodynamic
    interference.

    Nose and tail curvatures are computed from fineness ratios using empirical
    relationships. The body is discretized into 11 sections in each direction
    for adequate geometric fidelity.

    **Major Assumptions**
        * Body has smooth, continuous curvature
        * Fineness ratios adequately characterize nose/tail shapes
        * Cross-sections are adequate for interference modeling

    **Definitions**

    'Fineness Ratio'
        Ratio of body length to maximum diameter, affecting shape curvature

    'Cross-Section'
        Two-dimensional slice through three-dimensional body geometry
    """  

    symm = avl_body.symmetric   
    semispan_h = avl_body.widths.maximum * 0.5 * (2 - symm)
    semispan_v = avl_body.heights.maximum * 0.5
    origin = rcaide_body.origin[0]

    # Compute the curvature of the nose/tail given fineness ratio. Curvature is derived from general quadratic equation
    # This method relates the fineness ratio to the quadratic curve formula via a spline fit interpolation
    vec1 = [2 , 1.5, 1.2 , 1]
    vec2 = [1  ,1.57 , 3.2,  8]
    x = np.linspace(0,1,4)
    fuselage_nose_curvature =  np.interp(np.interp(rcaide_body.fineness.nose,vec2,x), x , vec1)
    fuselage_tail_curvature =  np.interp(np.interp(rcaide_body.fineness.tail,vec2,x), x , vec1) 


    # Horizontal Sections of Fuselage
    if semispan_h != 0.0:                
        width_array = np.linspace(-semispan_h, semispan_h, num=11,endpoint=True)
        for section_width in width_array:
            fuselage_h_section               = Section()
            fuselage_h_section_cabin_length  = avl_body.lengths.total - (avl_body.lengths.nose + avl_body.lengths.tail)
            fuselage_h_section_nose_length   = ((1 - ((abs(section_width/semispan_h))**fuselage_nose_curvature ))**(1/fuselage_nose_curvature))*avl_body.lengths.nose
            fuselage_h_section_tail_length   = ((1 - ((abs(section_width/semispan_h))**fuselage_tail_curvature ))**(1/fuselage_tail_curvature))*avl_body.lengths.tail
            fuselage_h_section_nose_origin   = avl_body.lengths.nose - fuselage_h_section_nose_length
            fuselage_h_section.tag           =  'fuselage_horizontal_section_at_' +  str(section_width) + '_m'
            fuselage_h_section.origin        = [ origin[0] + fuselage_h_section_nose_origin , origin[1] + section_width, origin[2]]
            fuselage_h_section.chord         = fuselage_h_section_cabin_length + fuselage_h_section_nose_length + fuselage_h_section_tail_length
            avl_body.append_section(fuselage_h_section,'horizontal')

    # Vertical Sections of Fuselage 
    if semispan_v != 0:               
        height_array = np.linspace(-semispan_v, semispan_v, num=11,endpoint=True)
        for section_height in height_array :
            fuselage_v_section               = Section()
            fuselage_v_section_cabin_length  = avl_body.lengths.total - (avl_body.lengths.nose + avl_body.lengths.tail)
            fuselage_v_section_nose_length   = ((1 - ((abs(section_height/semispan_v))**fuselage_nose_curvature ))**(1/fuselage_nose_curvature))*avl_body.lengths.nose
            fuselage_v_section_tail_length   = ((1 - ((abs(section_height/semispan_v))**fuselage_tail_curvature ))**(1/fuselage_tail_curvature))*avl_body.lengths.tail
            fuselage_v_section_nose_origin   = avl_body.lengths.nose - fuselage_v_section_nose_length
            fuselage_v_section.tag           = 'fuselage_vertical_top_section_at_' +  str(section_height) + '_m'        
            fuselage_v_section.origin        = [ origin[0] + fuselage_v_section_nose_origin,  origin[1],  origin[2] + section_height ]
            fuselage_v_section.chord         = fuselage_v_section_cabin_length + fuselage_v_section_nose_length + fuselage_v_section_tail_length
            avl_body.append_section(fuselage_v_section,'vertical')

    return avl_body

