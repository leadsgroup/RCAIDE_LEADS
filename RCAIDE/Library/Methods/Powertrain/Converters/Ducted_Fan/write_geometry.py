# RCAIDE/Library/Methods/Propulsor/Ducted_Fan_Propulsor/write_geometry.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units 
from RCAIDE.Library.Methods.Geometry.Airfoil import  import_airfoil_geometry , compute_naca_4series
from .purge_files import purge_files   
import numpy as  np
 
# ---------------------------------------------------------------------------------------------------------------------- 
# Write Geometry 
# ---------------------------------------------------------------------------------------------------------------------- 
def write_geometry(dfdc_object,run_script_path):
    """This function writes the translated aircraft geometry into text file read 
    by DFDC when it is called 
    """     
    # unpack inputs
    case_file   = dfdc_object.geometry.tag + '.case' 
    purge_files([case_file]) 
    geometry             = open(case_file,'w') 
    with open(case_file,'w') as geometry:
        make_header_text(dfdc_object,geometry)
        make_hub_text(dfdc_object,geometry)
        make_separator_text(dfdc_object,geometry)  
        make_duct_text(dfdc_object,geometry)
    return

def make_header_text(dfdc_object,geometry):  
    """This function writes the header using the template required for the DFDC executable to read
 
    """      
    header_1_text_template = \
'''DFDC Version  0.70 
Bladed rotor + actdisk stator test case                                             

OPER
!        Vinf         Vref          RPM          RPM
  {0}   {1}       {2}          0.0    
!         Rho          Vso          Rmu           Alt
   1.2260       340.00      0.17800E-04  0.00000E+00
!       XDwake        Nwake
  0.80000               20
!       Lwkrlx
            F
ENDOPER

AERO
!  #sections
     1
!   Xisection
  0.00000E+00
!       A0deg        dCLdA        CLmax         CLmin
  0.00000E+00   6.2800       1.5000      -1.0000    
!  dCLdAstall     dCLstall      Cmconst         Mcrit
  0.50000      0.20000      0.00000E+00  0.70000    
!       CDmin      CLCDmin     dCDdCL^2
  0.12000E-01  0.10000      0.50000E-02
!       REref        REexp
  0.20000E+06  0.35000    
ENDAERO

ROTOR
!       Xdisk        Nblds       NRsta
  {3}                {4}           {5}
!  #stations
    {6}
!           r        Chord         Beta
'''
    
    length                = dfdc_object.geometry.length
    x_disc_rotor          = dfdc_object.geometry.rotor.percent_x_location * length
    number_of_blades      = dfdc_object.geometry.number_of_rotor_blades 
    number_of_end_points  = dfdc_object.geometry.number_of_radial_stations + 1 
    number_of_stations    = dfdc_object.geometry.number_of_radial_stations  
    hub_radius            = dfdc_object.geometry.hub_radius
    clearance             = dfdc_object.geometry.blade_clearance
    design_RPM            = dfdc_object.geometry.cruise.design_angular_velocity /Units.rpm 
    tip_radius            = dfdc_object.geometry.tip_radius  
    V_inf                 = dfdc_object.geometry.cruise.design_freestream_velocity  
    V_ref                 = dfdc_object.geometry.cruise.design_reference_velocity  
    header_1_text         = header_1_text_template.format(V_inf,V_ref, design_RPM,x_disc_rotor,number_of_blades,number_of_end_points, number_of_stations)
   
    geometry.write(header_1_text)
    
    station_chords = np.linspace(0.4,0.3, number_of_stations) *tip_radius
    station_twists = np.linspace(77, 30, number_of_stations)
    station_radii  = np.linspace(hub_radius+clearance, tip_radius-clearance,number_of_stations )
    for i in range(number_of_stations): 
        station_radius = station_radii[i]
        station_chord  = station_chords[i] 
        station_twist  = station_twists[i]
        case_text = '  ' + format(station_radius, '.6f')+ "  " + format(station_chord, '.6f')+ "   " + format(station_twist, '.6f') + "\n" 
        geometry.write(case_text)
        
    header_2_text_template = \
'''ENDROTOR


AERO
!  #sections
     1
!   Xisection
  0.00000E+00
!       A0deg        dCLdA        CLmax         CLmin
  0.00000E+00   6.2800       1.0000      -1.5000    
!  dCLdAstall     dCLstall      Cmconst         Mcrit
  0.50000      0.20000      0.00000E+00  0.70000    
!       CDmin      CLCDmin     dCDdCL^2
  0.12000E-01 -0.10000      0.50000E-02
!       REref        REexp          TOC      dCDdCL^2
  0.20000E+06  0.35000      0.10000      0.20000E-01
ENDAERO

ACTDISK
! Xdisk   NRPdef
  {0}     15
! #stations
  3
! r     BGam
 0.02   -10.0
 0.06   -10.0
 0.10   -10.0
ENDACTDISK


DRAGOBJ
!  #pts
     4
!           x            r          CDA
  0.40000E-01  0.60000E-01  0.40000E-01
  0.40000E-01  0.80000E-01  0.35000E-01
  0.40000E-01  0.10000      0.30000E-01
  0.40000E-01  0.12000      0.30000E-01
ENDDRAGOBJ

'''
    x_disc_stator         = dfdc_object.geometry.stator.percent_x_location  * length  
    header_2_text         = header_2_text_template.format(x_disc_stator ) 
    geometry.write(header_2_text)     
    return  

def make_hub_text(dfdc_object,geometry):  
    """This function writes the rotor using the template required for the DFDC executable to read 
    """ 
    duct_header = \
'''GEOM
FatDuct + CB test case
''' 
    geometry.write(duct_header) 
    if len(dfdc_object.geometry.hub_airfoil) > 0:
        if type(dfdc_object.geometry.hub_airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
            NACA_Code =  dfdc_object.geometry.hub_airfoil.NACA_4_Series_code
            airfoil_geometry_data = compute_naca_4series(NACA_Code)    
        else: 
            airfoil_filename      = dfdc_object.geometry.hub_airfoil.coordinate_file
            airfoil_geometry_data = import_airfoil_geometry(airfoil_filename)
    else:  
        airfoil_geometry_data = compute_naca_4series('0015') 
    dim = len(airfoil_geometry_data.x_upper_surface)
    x_coords =  airfoil_geometry_data.x_upper_surface[::-1]
    y_coords =  airfoil_geometry_data.y_upper_surface[::-1]
     
    for i in range(dim):
        x_coord  = x_coords[i]* (dfdc_object.geometry.hub_radius / max(y_coords))
        y_coord  = y_coords[i]* (dfdc_object.geometry.hub_radius / max(y_coords))
        case_text = '     ' + format(x_coord, '.6f')+ "    " + format(y_coord, '.6f') + "\n" 
        geometry.write(case_text)
              
    return


def make_separator_text(dfdc_object,geometry):  
    """This function writes the operating conditions using the template required for the DFDC executable to read 
    """      
    separator_text = \
'''  999.0 999.0
'''
    geometry.write(separator_text) 
    return


def make_duct_text(dfdc_object,geometry):  
    """This function writes the operating conditions using the template required for the DFDC executable to read
 
    """      
    if len(dfdc_object.geometry.duct_airfoil) > 0: 
        if type(dfdc_object.geometry.duct_airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
            NACA_Code =  dfdc_object.geometry.duct_airfoil.NACA_4_Series_code
            airfoil_geometry_data = compute_naca_4series(NACA_Code)    
        else: 
            airfoil_filename      = dfdc_object.geometry.duct_airfoil.coordinate_file
            airfoil_geometry_data = import_airfoil_geometry(airfoil_filename)
    else:  
        airfoil_geometry_data= compute_naca_4series('2208')    
    dim = len(airfoil_geometry_data.x_coordinates)

    x_coords  = airfoil_geometry_data.x_coordinates*dfdc_object.geometry.length 
    y_coords  = airfoil_geometry_data.y_coordinates*dfdc_object.geometry.length     
    offset    = abs(min(y_coords)) + dfdc_object.geometry.tip_radius
    for i in range(dim):  
        case_text = '     ' + format(x_coords[i], '.6f')+ "    " + format(y_coords[i] + offset, '.6f') + "\n" 
        geometry.write(case_text) 
    end_text = \
'''ENDGEOM
'''               
    geometry.write(end_text) 
    return  
 
