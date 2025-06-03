# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/write_geometry.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.purge_files import purge_files
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.create_avl_datastructures import translate_avl_wing, translate_avl_body

# ----------------------------------------------------------------------------------------------------------------------
#  write_geometry
# ---------------------------------------------------------------------------------------------------------------------- 
def write_geometry(avl_object,run_script_path):
    """
    Writes translated aircraft geometry to AVL-compatible input file for vortex lattice analysis.

    Parameters
    ----------
    avl_object : Data
        AVL analysis object containing aircraft configuration and settings
            - vehicle : Vehicle
                Aircraft configuration with wings, fuselages, and mass properties
            - settings : Data
                Analysis configuration parameters
    run_script_path : str
        Directory path for analysis execution (currently unused but maintained for interface compatibility)

    Returns
    -------
    None
        Geometry file is written to disk at specified location
    """
    
    # unpack inputs
    aircraft                          = avl_object.vehicle
    geometry_file                     = avl_object.settings.filenames.features
    number_of_spanwise_vortices       = avl_object.settings.number_of_spanwise_vortices
    number_of_chordwise_vortices      = avl_object.settings.number_of_chordwise_vortices 
    
    avl_object.reference_values.S_ref        = avl_object.vehicle.wings['main_wing'].areas.reference
    avl_object.reference_values.c_ref        = avl_object.vehicle.wings['main_wing'].chords.mean_aerodynamic
    avl_object.reference_values.b_ref        = avl_object.vehicle.wings['main_wing'].spans.projected
    avl_object.reference_values.X_ref        = avl_object.vehicle.mass_properties.center_of_gravity[0][0]
    avl_object.reference_values.Y_ref        = avl_object.vehicle.mass_properties.center_of_gravity[0][1]
    avl_object.reference_values.Z_ref        = avl_object.vehicle.mass_properties.center_of_gravity[0][2]
    avl_object.reference_values.aspect_ratio = (avl_object.reference_values.b_ref ** 2) / avl_object.reference_values.S_ref 
    
    # Open the geometry file after purging if it already exists
    purge_files([geometry_file]) 
    geometry             = open(geometry_file,'w')

    with open(geometry_file,'w') as geometry:
        header_text       = make_header_text(avl_object)
        geometry.write(header_text)
        
        for w in aircraft.wings:
            avl_wing      = translate_avl_wing(w)
            wing_text     = make_surface_text(avl_wing,number_of_spanwise_vortices,number_of_chordwise_vortices)
            geometry.write(wing_text)  
        
        if avl_object.settings.model_fuselage:
            for b in aircraft.fuselages:
                avl_body  = translate_avl_body(b)
                body_text = make_body_text(avl_body,number_of_chordwise_vortices)
                geometry.write(body_text)
            
    return


def make_header_text(avl_object):  
    """
    Creates AVL geometry file header with reference values and symmetry settings.

    Parameters
    ----------
    avl_object : Data
        AVL analysis object containing aircraft configuration
            
    Returns
    -------
    header_text : str
        Formatted header text for AVL geometry file
    """
    header_base = \
'''{0}

#Mach
 {1}
 
#Iysym   IZsym   Zsym
  {2}      {3}     {4}
  
#Sref    Cref    Bref 	<meters>
{5}      {6}     {7}

#Xref    Yref    Zref   <meters>
{8}      {9}     {10}

'''

    # Unpack inputs
    Iysym = avl_object.settings.flow_symmetry.xz_plane
    Izsym = avl_object.settings.flow_symmetry.xy_parallel
    Zsym  = avl_object.settings.flow_symmetry.z_symmetry_plane
    Sref  = avl_object.vehicle.wings['main_wing'].areas.reference
    Cref  = avl_object.vehicle.wings['main_wing'].chords.mean_aerodynamic
    Bref  = avl_object.vehicle.wings['main_wing'].spans.projected
    Xref  = avl_object.vehicle.mass_properties.center_of_gravity[0][0]
    Yref  = avl_object.vehicle.mass_properties.center_of_gravity[0][1]
    Zref  = avl_object.vehicle.mass_properties.center_of_gravity[0][2]
    name  = avl_object.vehicle.tag     
    mach = 0.0

    # Insert inputs into the template
    header_text = header_base.format(name,mach,Iysym,Izsym,Zsym,Sref,Cref,Bref,Xref,Yref,Zref)

    return header_text


def make_surface_text(avl_wing,number_of_spanwise_vortices,number_of_chordwise_vortices):
    """
    Formats wing geometry data into AVL surface definition with sections and control surfaces.

    Parameters
    ----------
    avl_wing : Wing
        AVL wing data structure with translated geometry
    number_of_spanwise_vortices : int
        Number of spanwise vortex elements for discretization
    number_of_chordwise_vortices : int
        Number of chordwise vortex elements for discretization

    Returns
    -------
    surface_text : str
        Complete AVL surface definition text with all sections

    """
    ordered_tags = []         
    surface_base = \
        '''

#---------------------------------------------------------
SURFACE
{0}
#Nchordwise  Cspace   Nspanwise  Sspace
{1}         {2}         {3}      {4}{5}
'''        
    # Unpack inputs
    symm = avl_wing.symmetric
    name = avl_wing.tag

    if symm:
        ydup = '\n\nYDUPLICATE\n0.0\n' # Duplication of wing about xz plane
    else:
        ydup     = ' ' 
    
    # Vertical Wings
    if avl_wing.vertical:
        # Define precision of analysis. See AVL documentation for reference 
        chordwise_vortex_spacing = 1.0
        spanwise_vortex_spacing  = -1.1                              # cosine distribution i.e. || |   |    |    |  | ||
        ordered_tags = sorted(avl_wing.sections, key = lambda x: x.origin[0][2])
        
        # Write text 
        surface_text = surface_base.format(name,number_of_chordwise_vortices,chordwise_vortex_spacing,number_of_spanwise_vortices ,spanwise_vortex_spacing,ydup)
        for i in range(len(ordered_tags)):
            section_text    = make_wing_section_text(ordered_tags[i])
            surface_text    = surface_text + section_text
            
    # Horizontal Wings        
    else:        
        # Define precision of analysis. See AVL documentation for reference
        chordwise_vortex_spacing = 1.0        
        spanwise_vortex_spacing  = 1.0                              # cosine distribution i.e. || |   |    |    |  | ||
        ordered_tags = sorted(avl_wing.sections, key = lambda x: x.origin[0][1])
    
        # Write text  
        surface_text = surface_base.format(name,number_of_chordwise_vortices,chordwise_vortex_spacing,number_of_spanwise_vortices ,spanwise_vortex_spacing,ydup)
        for i in range(len(ordered_tags)):
            section_text    = make_wing_section_text(ordered_tags[i])
            surface_text    = surface_text + section_text

    return surface_text


def make_body_text(avl_body,number_of_chordwise_vortices):   
    """
    Formats fuselage geometry into AVL body representation using cross-sectional surfaces.

    Parameters
    ----------
    avl_body : Body
        AVL body data structure with translated fuselage geometry
    number_of_chordwise_vortices : int
        Number of chordwise vortex elements for body surface discretization

    Returns
    -------
    body_text : str
        Complete AVL body definition with horizontal and vertical surfaces

    """
    surface_base = \
'''

#---------------------------------------------------------
SURFACE
{0}
#Nchordwise  Cspace   Nspanwise  Sspace
{1}           {2}      
'''
    # Unpack inputs
    name = avl_body.tag
    
    # Define precision of analysis. See AVL documentation for reference 
    chordwise_vortex_spacing = 1.0 
    
    # Form the horizontal part of the + shaped fuselage    
    hname           = name + '_horizontal'
    horizontal_text = surface_base.format(hname,number_of_chordwise_vortices,chordwise_vortex_spacing)
       
    ordered_tags = []
    ordered_tags = sorted(avl_body.sections.horizontal, key = lambda x: x.origin[1])
    for i in range(len(ordered_tags)):
        section_text    = make_body_section_text(ordered_tags[i])
        horizontal_text = horizontal_text + section_text
        
    # Form the vertical part of the + shaped fuselage
    vname         = name + '_vertical'
    vertical_text = surface_base.format(vname,number_of_chordwise_vortices,chordwise_vortex_spacing)   
    ordered_tags = []
    ordered_tags = sorted(avl_body.sections.vertical, key = lambda x: x.origin[2])
    for i in range(len(ordered_tags)):
        section_text    = make_body_section_text(ordered_tags[i])
        vertical_text = vertical_text + section_text
        
    body_text = horizontal_text + vertical_text
    return body_text  


def make_wing_section_text(avl_section):
    """
    Formats individual wing section geometry including airfoil and control surface definitions.

    Parameters
    ----------
    avl_section : Section
        Wing section data structure

    Returns
    -------
    wing_section_text : str
        Formatted AVL section definition with airfoil and control data
    """
    section_base = \
'''
SECTION
#Xle    Yle      Zle      Chord     Ainc  Nspanwise  Sspace
{0}  {1}    {2}    {3}    {4}     
'''
    airfoil_base = \
'''AFILE
{}
'''
    naca_airfoil_base = \
'''NACA
{}
'''
    # Unpack inputs
    x_le          = avl_section.origin[0][0]
    y_le          = avl_section.origin[0][1]
    z_le          = avl_section.origin[0][2]
    chord         = avl_section.chord
    ainc          = avl_section.twist
    airfoil_coord = avl_section.airfoil_coord_file
    naca_airfoil  = avl_section.naca_airfoil 
     
    wing_section_text = section_base.format(round(x_le,4),round(y_le,4), round(z_le,4),round(chord,4),round(ainc,4))
    if airfoil_coord:
        wing_section_text = wing_section_text + airfoil_base.format(airfoil_coord)
    if naca_airfoil:
        wing_section_text = wing_section_text + naca_airfoil_base.format(naca_airfoil)        
    
    ordered_cs = []
    ordered_cs = sorted(avl_section.control_surfaces, key = lambda x: x.order)
    for i in range(len(ordered_cs)):
        control_text = make_controls_text(ordered_cs[i])
        wing_section_text = wing_section_text + control_text

    return wing_section_text

    
def make_body_section_text(avl_body_section):
    """
    Formats individual fuselage cross-section for AVL body surface definition.

    Parameters
    ----------
    avl_body_section : Section
        Body section data structure

    Returns
    -------
    body_section_text : str
        Formatted AVL body section definition
    """
    section_base = \
'''
SECTION
#Xle     Yle      Zle      Chord     Ainc  Nspanwise  Sspace
{0}    {1}     {2}     {3}     {4}      1        0
'''
    airfoil_base = \
'''AFILE
{}
'''

    # Unpack inputs
    x_le    = avl_body_section.origin[0]
    y_le    = avl_body_section.origin[1]
    z_le    = avl_body_section.origin[2]
    chord   = avl_body_section.chord
    ainc    = avl_body_section.twist
    airfoil = avl_body_section.airfoil_coord_file

    body_section_text = section_base.format(round(x_le,4),round(y_le,4), round(z_le,4),round(chord,4),round(ainc,4))
    if airfoil:
        body_section_text = body_section_text + airfoil_base.format(airfoil)
    
    return body_section_text

    
def make_controls_text(avl_control_surface):
    """
    Formats control surface definition for AVL analysis including hinge and deflection properties.

    Parameters
    ----------
    avl_control_surface : Control_Surface
        Control surface data structure

    Returns
    -------
    control_text : str
        Formatted AVL control surface definition
    """
    control_base = \
'''CONTROL
{0}    {1}   {2}   {3}  {4}
'''

    # Unpack inputs
    name     = avl_control_surface.tag
    gain     = avl_control_surface.gain
    xhinge   = avl_control_surface.x_hinge
    hv       = avl_control_surface.hinge_vector
    sign_dup = avl_control_surface.sign_duplicate

    control_text = control_base.format(name,gain,xhinge,hv,sign_dup)

    return control_text
