# Test_Propeller.py
#
# Created:  Dec 2024 M. Clarke

# Imports
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor import design_propeller
import os 

# design propeller  
def Test_Propeller(): 
    
    prop                                     = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    prop.number_of_blades                    = 3
    prop.number_of_engines                   = 1
    prop.tip_radius                          = 1.0668
    prop.hub_radius                          = 0.21336
    prop.cruise.design_freestream_velocity   = 49.1744
    prop.cruise.design_tip_mach              = 0.65
    prop.cruise.design_angular_velocity      = 207.16160479940007
    prop.cruise.design_Cl                    = 0.7
    prop.cruise.design_altitude              = 1. * Units.km 
    prop.cruise.design_thrust                = 3054.4809132125697
    
    # define first airfoil    
    ospath                                     = os.path.abspath(__file__)
    separator                                  = os.path.sep
    rel_path                                   = os.path.dirname(ospath) + separator + '..'  + separator 
    airfoil_1                                  = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil_1.tag                              = 'NACA_4412' 
    airfoil_1.coordinate_file                  = rel_path + 'Airfoils' + separator + 'NACA_4412.txt'   # absolute path   
    airfoil_1.polar_files                      =[rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt',
                                                 rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt',
                                                 rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt',
                                                 rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt',
                                                 rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt'] 
    prop.append_airfoil(airfoil_1)           # append first airfoil 
    
    # define  second airfoil 
    airfoil_2                                = RCAIDE.Library.Components.Airfoils.Airfoil()  
    airfoil_2.tag                            = 'Clark_Y' 
    airfoil_2.coordinate_file                =   rel_path + 'Airfoils' + separator + 'Clark_y.txt' 
    airfoil_2.polar_files                    = [ rel_path + 'Airfoils' + separator + 'Polars' + separator + 'Clark_y_polar_Re_50000.txt',
                                                 rel_path + 'Airfoils' + separator + 'Polars' + separator + 'Clark_y_polar_Re_100000.txt',
                                                 rel_path + 'Airfoils' + separator + 'Polars' + separator + 'Clark_y_polar_Re_200000.txt',
                                                 rel_path + 'Airfoils' + separator + 'Polars' + separator + 'Clark_y_polar_Re_500000.txt',
                                                 rel_path + 'Airfoils' + separator + 'Polars' + separator + 'Clark_y_polar_Re_1000000.txt'] 
    prop.append_airfoil(airfoil_2)          # append second airfoil 
    
    # define polar stations on rotor 
    prop.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1]
    design_propeller(prop)
    
    return prop 