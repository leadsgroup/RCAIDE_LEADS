# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/write_mass_file.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.purge_files              import purge_files  

# ----------------------------------------------------------------------------------------------------------------------
#  write_mass_file
# ---------------------------------------------------------------------------------------------------------------------- 
def write_mass_file(avl_object,run_conditions):
    """
    Writes aircraft mass properties to AVL-compatible file for trimmed flight analysis.

    Parameters
    ----------
    avl_object : Data
        AVL analysis object containing aircraft configuration and file settings
    run_conditions : Data
        Flight conditions for analysis reference values

    Returns
    -------
    None
        Mass file is written to disk at specified location

    Notes
    -----
    Creates AVL mass properties file containing aircraft weight, center of gravity,
    and moments of inertia for accurate trim analysis. The function automatically
    selects between available mass values and formats data according to AVL
    input requirements.

    **Major Assumptions**
        * Aircraft moments and products of inertia are defined.
    """    
    
    # unpack inputs
    mass_file       = avl_object.settings.filenames.mass_file 
    aircraft        = avl_object.vehicle
    
    # Open the mass file after purging if it already exists
    purge_files([mass_file]) 
    mass_file_script  = open(mass_file,'w')

    with open(mass_file,'w') as mass_file_script:        
        #This function writes the header using the template required for the AVL executable to read
           
        # mass file template
        base_text = \
'''
#-------------------------------------------------
#  {0}
#
#  Dimensional unit and parameter data.
#  Mass & Inertia breakdown.
#-------------------------------------------------

#  Names and scalings for units to be used for trim and eigenmode calculations.
#  The Lunit and Munit values scale the mass, xyz, and inertia table data below.
#  Lunit value will also scale all lengths and areas in the AVL input file.
Lunit = 1.0 m
Munit = 1.0 kg
Tunit = 1.0 s

#------------------------- 
#  Gravity and density to be used as default values in trim setup.
#  Must be in the units given above.
g   = {1}
rho = {2}

#-------------------------
#  Mass & Inertia breakdown.
#  x y z  is location of item's own CG.
#  Ixx... are item's inertias about item's own CG.
#
#  x,y,z system here must be exactly the same one used in the AVL input file
#     (same orientation, same origin location, same length units)
#
#  mass     x     y     z    Ixx    Iyy    Izz   Component Name
#   
    {3}  {4}  {5}  {6}  {7}  {8}  {9} ! {0}
'''

        # Unpack inputs
        name    = avl_object.vehicle.tag
        density = run_conditions.freestream.density[0][0] 
        gravity = run_conditions.freestream.gravity[0][0] 
        
        if aircraft.mass_properties.mass == 0:
            mass = aircraft.mass_properties.max_takeoff
        elif aircraft.mass_properties.max_takeoff == 0:
            mass = aircraft.mass_properties.mass
        else:
            raise AttributeError("Specify Vehicle Mass")
         
        x       = aircraft.mass_properties.center_of_gravity[0][0]
        y       = aircraft.mass_properties.center_of_gravity[0][1]
        z       = aircraft.mass_properties.center_of_gravity[0][2]
        Ixx     = aircraft.mass_properties.moments_of_inertia.tensor[0][0]
        Iyy     = aircraft.mass_properties.moments_of_inertia.tensor[1][1]
        Izz     = aircraft.mass_properties.moments_of_inertia.tensor[2][2]
        
        # Insert inputs into the template
        text = base_text.format(name, gravity , density,mass, x,y,z,Ixx,Iyy,Izz)
        mass_file_script.write(text)  
            
    return