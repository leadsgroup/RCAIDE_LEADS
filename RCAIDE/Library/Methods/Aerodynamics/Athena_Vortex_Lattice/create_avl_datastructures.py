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
    """ Translates wing geometry from the vehicle setup to AVL format

    Assumptions:
        None

    Source:
        None

    Inputs:
        rcaide_wing.tag                                                          [-]
        rcaide_wing.xz_plane_symmetric                                                    [boolean]
        rcaide_wing.verical                                                      [boolean]
        rcaide_wing - passed into the populate_wing_sections function            [data stucture]

    Outputs:
        w - aircraft wing in AVL format                                         [data stucture] 

    Properties Used:
        N/A
    """         
    w                     = Wing()
    w.tag                 = rcaide_wing.tag
    w.xz_plane_symmetric  = rcaide_wing.xz_plane_symmetric
    w.vertical            = rcaide_wing.vertical
    w                     = populate_wing_sections(w,rcaide_wing)

    return w

def translate_avl_body(rcaide_body):
    """ Translates body geometry from the vehicle setup to AVL format

    Assumptions:
        None

    Source:
        None

    Inputs:
        body.tag                                                       [-]  
        rcaide_body.lengths.nose                                        [meters]
        rcaide_body.lengths.tail                                        [meters]
        rcaide_wing.verical                                             [meters]
        rcaide_body.width                                               [meters]
        rcaide_body.heights.maximum                                     [meters]
        rcaide_wing - passed into the populate_body_sections function   [data stucture]

    Outputs:
        b - aircraft body in AVL format                                [data stucture] 

    Properties Used:
        N/A
    """  
    b                          = Body()
    b.tag                      = rcaide_body.tag
    b.xz_plane_symmetric       = True
    b.lengths.total            = rcaide_body.lengths.total
    b.lengths.nose             = rcaide_body.lengths.nose
    b.lengths.tail             = rcaide_body.lengths.tail
    b.widths.maximum           = rcaide_body.width
    b.heights.maximum          = rcaide_body.heights.maximum
    b                          = populate_body_sections(b,rcaide_body)

    return b

def populate_wing_sections(avl_wing,rcaide_wing): 
    """ Creates sections of wing geometry and populates the AVL wing data structure

    Assumptions:
        None

    Source:
        None

    Inputs:
        avl_wing.xz_plane_symmetric                         [boolean]
        rcaide_wing.spans.projected                 [meters]
        rcaide_wing.origin                          [meters]
        rcaide_wing.dihedral                        [radians]
        rcaide_wing.segments.sweeps.leading_edge    [radians]
        rcaide_wing.segments.root_chord_percent     [-]
        rcaide_wing.segments.percent_span_location  [-]
        rcaide_wing.segments.sweeps.quarter_chord   [radians]
        rcaide_wing.segment.twist                   [radians]

    Outputs:
        avl_wing - aircraft wing in AVL format     [data stucture] 

    Properties Used:
        N/A
    """           
        
    # obtain the geometry for each segment in a loop                                            
    symm                 = avl_wing.xz_plane_symmetric
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

    """ Converts control surfaces on a rcaide wing to sections in avl wing

    Assumptions:
        None

    Source:
        None

    Inputs: 
        rcaide_wing           [-]
        avl_wing             [-]
        semispan             [meters]
        root_chord_percent   [unitless]
        tip_chord_percent    [unitless]
        tip_percent_span     [unitless]
        root_percent_span    [unitless]
        root_twist           [radians]
        tip_twist            [radians]
        tip_airfoil          [unitless]
        seg_tag              [unitless]
        dihedral             [radians]
        origin               [meters]
        sweep                [radians]
        
    Outputs: 
        None

    Properties Used:
        N/A
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
    """ Creates sections of body geometry and populates the AVL body data structure

    Assumptions:
        None

    Source:
        None

    Inputs:
        avl_wing.xz_plane_symmetric                       [boolean]
        avl_body.widths.maximum                  [meters]
        avl_body.heights.maximum                 [meters]
        rcaide_body.fineness.nose                 [meters]
        rcaide_body.fineness.tail                 [meters]
        avl_body.lengths.total                   [meters]
        avl_body.lengths.nose                    [meters] 
        avl_body.lengths.tail                    [meters]  

    Outputs:
        avl_body - aircraft body in AVL format   [data stucture] 

    Properties Used:
        N/A
    """  

    symm = avl_body.xz_plane_symmetric   
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

