# RCAIDE/Framework/External_Interfaces/OpenVSP/vsp_propeller.py

# Created:  Sep 2021, R. Erhard
# Modified:

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units , Data
import numpy as np
import scipy as sp
import string
try:
    import vsp as vsp
except ImportError:
    try:
        import openvsp as vsp
    except ImportError:
        # This allows RCAIDE to build without OpenVSP
        pass

# This enforces lowercase names
chars = string.punctuation + string.whitespace
t_table = str.maketrans( chars          + string.ascii_uppercase ,
                         '_'*len(chars) + string.ascii_lowercase )

# ---------------------------------------------------------------------------------------------------------------------- 
#  vsp read rotor
# ---------------------------------------------------------------------------------------------------------------------- 
def read_vsp_rotor(prop_id, units_type='SI',write_airfoil_file=True):
    """This reads an OpenVSP rotor geometry and writes it into a RCAIDE rotor format.

    Assumptions:
    1. Written for OpenVSP 3.24.0

    Source:
    N/A

    Inputs:
    1. VSP 10-digit geom ID for rotor.
    2. units_type set to 'SI' (default) or 'Imperial'.
    3. write_airfoil_file set to True (default) or False
    4. number_of_radial_stations is the radial discretization used to extract the rotor design from OpenVSP

    Outputs:
    Writes RCAIDE rotor object, with these geometries, from VSP:
    	rotor.
    		origin                                  [m] in all three dimensions
    		orientation				[deg] in all three dimensions
    		number_of_blades			[-]
    		tip_radius				[m]
    		hub_radius				[m]
    		twist_distribution			[deg]
    		chord_distribution			[m]
    		radius_distribution			[m]
    		sweep_distribution			[deg]
    		mid_chord_alignment			[m]
    		max_thickness_distribution		[m]
    		thickness_to_chord			[-]
    		blade_solidity				[-]
    		rotation			        [-]
    		thickness_to_chord                      [-]
                beta34                                  [radians]
                pre_cone                                [radians]
                rake                                    [radians]
                skew                                    [radians]
                axial                                   [radians]
                tangential                              [radians]
    		dihedral                                [radians]
    		symmetric                               <boolean>
    		tag                                     <string>
    		Segments.
    		  tag                                   <string>
    		  twist                                 [radians]
    		  percent_span_location                 [-]  .1 is 10%
    		  root_chord_percent                    [-]  .1 is 10%
    		  dihedral_outboard                     [radians]
    		  sweeps.quarter_chord                  [radians]
    		  thickness_to_chord                    [-]
    		  airfoil                               <NACA 4-series, 6 series, or airfoil file>

    Properties Used:
    N/A
    """


    # Check if this is a rotor or a lift rotor 
    rotor 	= RCAIDE.Library.Components.Powertrain.Converters.Rotor()

    # Set the units
    if units_type == 'SI':
        units_factor = Units.meter * 1.
    elif units_type == 'imperial':
        units_factor = Units.foot * 1.
    elif units_type == 'inches':
        units_factor = Units.inch * 1.

    # Apply a tag to the rotor
    if vsp.GetGeomName(prop_id):
        tag = vsp.GetGeomName(prop_id)
        tag = tag.translate(t_table)
        rotor.tag = tag
    else:
        rotor.tag = 'propgeom'


    scaling           = vsp.GetParmVal(prop_id, 'Scale', 'XForm')
    units_factor      = units_factor*scaling

    # Propeller location (absolute)
    rotor.origin 	= [[0.0,0.0,0.0]]
    rotor.origin[0][0] 	= vsp.GetParmVal(prop_id, 'X_Location', 'XForm') * units_factor
    rotor.origin[0][1] 	= vsp.GetParmVal(prop_id, 'Y_Location', 'XForm') * units_factor
    rotor.origin[0][2] 	= vsp.GetParmVal(prop_id, 'Z_Location', 'XForm') * units_factor

    # Propeller orientation
    rotor.orientation_euler_angles 	= [0.0,0.0,0.0]
    rotor.orientation_euler_angles[0] 	= vsp.GetParmVal(prop_id, 'X_Rotation', 'XForm') * Units.degrees
    rotor.orientation_euler_angles[1] 	= vsp.GetParmVal(prop_id, 'Y_Rotation', 'XForm') * Units.degrees
    rotor.orientation_euler_angles[2] 	= vsp.GetParmVal(prop_id, 'Z_Rotation', 'XForm') * Units.degrees

    # Get the rotor parameter IDs
    parm_id    = vsp.GetGeomParmIDs(prop_id)
    parm_names = []
    for i in range(len(parm_id)):
        parm_name = vsp.GetParmName(parm_id[i])
        parm_names.append(parm_name)

    # Run the vsp Blade Element analysis
    vsp.SetStringAnalysisInput( "BladeElement" , "PropID" , (prop_id,) )
    rid = vsp.ExecAnalysis( "BladeElement" )
    Nc  = len(vsp.GetDoubleResults(rid,"YSection_000"))

    rotor.vtk_airfoil_points           = 2*Nc
    rotor.CLi                          = vsp.GetParmVal(parm_id[parm_names.index('CLi')])
    rotor.blade_solidity               = vsp.GetParmVal(parm_id[parm_names.index('Solidity')])
    rotor.number_of_blades             = int(vsp.GetParmVal(parm_id[parm_names.index('NumBlade')]))

    rotor.tip_radius                   = vsp.GetDoubleResults(rid, "Diameter" )[0] / 2 * units_factor
    rotor.radius_distribution          = np.array(vsp.GetDoubleResults(rid, "Radius" )) * rotor.tip_radius

    rotor.radius_distribution[-1]      = 0.99 * rotor.tip_radius # BEMT requires max nondimensional radius to be less than 1.0
    if rotor.radius_distribution[0] == 0.:
        start = 1
        rotor.radius_distribution = rotor.radius_distribution[start:]
    else:
        start = 0

    rotor.hub_radius                   = rotor.radius_distribution[0]

    rotor.chord_distribution           = np.array(vsp.GetDoubleResults(rid, "Chord" ))[start:]  * rotor.tip_radius # vsp gives c/R
    rotor.twist_distribution           = np.array(vsp.GetDoubleResults(rid, "Twist" ))[start:]  * Units.degrees
    rotor.sweep_distribution           = np.array(vsp.GetDoubleResults(rid, "Sweep" ))[start:]
    rotor.mid_chord_alignment          = np.tan(rotor.sweep_distribution*Units.degrees)  * rotor.radius_distribution
    rotor.thickness_to_chord           = np.array(vsp.GetDoubleResults(rid, "Thick" ))[start:]
    rotor.max_thickness_distribution   = rotor.thickness_to_chord*rotor.chord_distribution * units_factor
    rotor.Cl_distribution              = np.array(vsp.GetDoubleResults(rid, "CLi" ))[start:]

    # Extra data from VSP BEM for future use in BEVW
    rotor.beta34                       = vsp.GetDoubleResults(rid, "Beta34" )[0]  # pitch at 3/4 radius
    rotor.pre_cone                     = vsp.GetDoubleResults(rid, "Pre_Cone")[0]
    rotor.rake                         = np.array(vsp.GetDoubleResults(rid, "Rake"))[start:]
    rotor.skew                         = np.array(vsp.GetDoubleResults(rid, "Skew"))[start:]
    rotor.axial                        = np.array(vsp.GetDoubleResults(rid, "Axial"))[start:]
    rotor.tangential                   = np.array(vsp.GetDoubleResults(rid, "Tangential"))[start:]

    # Set rotor rotation
    rotor.rotation = 1

    # ---------------------------------------------
    # Rotor Airfoil
    # ---------------------------------------------
    if write_airfoil_file:
        print("Airfoil write not yet implemented. Defaulting to NACA 4412 airfoil for rotor cross section.")

    return rotor

# ---------------------------------------------------------------------------------------------------------------------- 
# write_vsp_rotor_bem
# ---------------------------------------------------------------------------------------------------------------------- 
def write_vsp_rotor_bem(vsp_bem_filename,rotor):
    """   This functions writes a .bem file for OpenVSP
    Assumptions:
        None

    Source:
        None
    Inputs:
        OpenVSP .bem filename
        RCAIDE Propeller Data Structure
    Outputs:
        OpenVSP .bem file
    Properties Used:
        N/A
    """
    vsp_bem = open(vsp_bem_filename,'w')
    with open(vsp_bem_filename,'w') as vsp_bem:
        make_header_text(vsp_bem, rotor)

        make_section_text(vsp_bem,rotor)

        make_airfoil_text(vsp_bem,rotor)

    # Now import this rotor
    vsp.ImportFile(vsp_bem_filename,vsp.IMPORT_BEM,'')

    return


# ---------------------------------------------------------------------------------------------------------------------- 
# make_header_text
# ---------------------------------------------------------------------------------------------------------------------- 
def make_header_text(vsp_bem,rotor):
    """This function writes the header of the OpenVSP .bem file
    Assumptions:
        None

    Source:
        None
    Inputs:
        vsp_bem - OpenVSP .bem file
        rotor    - RCAIDE rotor data structure

    Outputs:
        NA
    Properties Used:
        N/A
    """
    header_base = \
'''...{0}...
Num_Sections: {1}
Num_Blade: {2}
Diameter: {3}
Beta 3/4 (deg): {4}
Feather (deg): 0.00000000
Pre_Cone (deg): 0.00000000
Center: {5}, {6}, {7}
Normal: {8}, {9}, {10}
'''
    # Unpack inputs
    name      = rotor.tag
    N         = len(rotor.radius_distribution)
    B         = int(rotor.number_of_blades)
    D         = np.round(rotor.tip_radius*2,5)
    beta      = np.round(rotor.twist_distribution/Units.degrees,5)
    X         = np.round(rotor.origin[0][0],5)
    Y         = np.round(rotor.origin[0][1],5)
    Z         = np.round(rotor.origin[0][2],5) 
    Xn        = np.round(rotor.orientation_euler_angles[2],5) / Units.degrees
    Yn        = np.round(rotor.orientation_euler_angles[0],5) / Units.degrees 
    Zn        = np.round(rotor.orientation_euler_angles[1],5) / Units.degrees

    beta_3_4  = np.interp(rotor.tip_radius*0.75,rotor.radius_distribution,beta)

    # Insert inputs into the template
    header_text = header_base.format(name,N,B,D,beta_3_4,X,Y,Z,Xn,Yn,Zn)
    vsp_bem.write(header_text)

    return

# ---------------------------------------------------------------------------------------------------------------------- 
# make_section_text
# ---------------------------------------------------------------------------------------------------------------------- 
def make_section_text(vsp_bem,rotor):
    """This function writes the sectional information of the rotor
    Assumptions:
        None

    Source:
        None
    Inputs:
        vsp_bem - OpenVSP .bem file
        rotor    - RCAIDE rotor data structure

    Outputs:
        NA
    Properties Used:
        N/A
    """
    header = \
        '''Radius/R, Chord/R, Twist (deg), Rake/R, Skew/R, Sweep, t/c, CLi, Axial, Tangential\n'''

    N          = len(rotor.radius_distribution)
    r_R        = np.zeros(N)
    c_R        = np.zeros(N)
    r_R        = rotor.radius_distribution/rotor.tip_radius
    c_R        = rotor.chord_distribution/rotor.tip_radius
    beta_deg   = rotor.twist_distribution/Units.degrees
    Rake_R     = np.zeros(N)
    Skew_R     = np.zeros(N)
    Sweep      = np.arctan(rotor.mid_chord_alignment/rotor.radius_distribution)
    t_c        = rotor.thickness_to_chord
    
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor: 
        CLi        = np.ones(N)*rotor.hover.design_Cl  
    elif type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Propeller:
        CLi        = np.ones(N)*rotor.cruise.design_Cl   
    elif type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor: 
        CLi        = np.ones(N)*rotor.hover.design_Cl  
    
    Axial      = np.zeros(N)
    Tangential = np.zeros(N)

    # Write rotor station imformation
    vsp_bem.write(header)
    for i in range(N):
        section_text = format(r_R[i], '.7f')+ ", " + format(c_R[i], '.7f')+ ", " + format(beta_deg[i], '.7f')+ ", " +\
            format( Rake_R[i], '.7f')+ ", " + format(Skew_R[i], '.7f')+ ", " + format(Sweep[i], '.7f')+ ", " +\
            format(t_c[i], '.7f')+ ", " + format(CLi[i], '.7f') + ", "+ format(Axial[i], '.7f') + ", " +\
            format(Tangential[i], '.7f') + "\n"
        vsp_bem.write(section_text)

    return

# ---------------------------------------------------------------------------------------------------------------------- 
# make_airfoil_text
# ---------------------------------------------------------------------------------------------------------------------- 
def make_airfoil_text(vsp_bem,rotor):
    """This function writes the airfoil geometry into the vsp file
    Assumptions:
        None

    Source:
        None
    Inputs:
        vsp_bem - OpenVSP .bem file
        rotor    - RCAIDE rotor data structure

    Outputs:
        NA
    Properties Used:
        N/A
    """

    N             = len(rotor.radius_distribution)
    airfoils      = rotor.airfoils
    airfoil_list  = list(airfoils.keys())
    a_sec         = rotor.airfoil_polar_stations
    for i in range(N):
        airfoil_station_header = '\nSection ' + str(i) + ' X, Y\n'
        vsp_bem.write(airfoil_station_header)
        airfoil_index =  a_sec[i]
        airfoil       = airfoils[airfoil_list[airfoil_index]]
        airfoil_x     = airfoil.geometry.x_coordinates 
        airfoil_y     = airfoil.geometry.y_coordinates 

        for j in range(len(airfoil_x)):
            section_text = format(airfoil_x[j], '.7f')+ ", " + format(airfoil_y[j], '.7f') + "\n"
            vsp_bem.write(section_text)
    return
