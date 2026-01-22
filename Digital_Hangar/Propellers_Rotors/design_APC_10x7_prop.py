# Imports
import RCAIDE
from RCAIDE.Core import Units, Data  
from RCAIDE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_properties import compute_airfoil_properties
from RCAIDE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry    import import_airfoil_geometry    
from scipy.interpolate import interp1d
import os
import numpy as np   

# design propeller  
def design_APC_10x7_prop():      
    prop                            = RCAIDE.Library.Components.Powertrain.Converters.Rotor()
    prop.inputs                     = Data() 
    prop.inputs.blade_pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'APC_10x7_Propeller'
    prop.tip_radius                 = (10/2)*Units.inches
    prop.hub_radius                 = prop.tip_radius*0.15 
    prop.number_of_blades           = 2  
    prop.thrust_angle               = 0.
    prop.VTOL_flag                  = True 
    r_R                             = np.array([0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50,0.55,
                                                0.60,0.65,0.70,0.75,0.80,0.85,0.90,0.95,0.99 ])
    b_R                             = np.array([0.138,0.154,0.175,0.190,0.198,0.202,0.200,0.195,
                                                0.186,0.174,0.161,0.145,0.129,0.112,0.096,0.081,0.061,0.040]) 
    beta                            = np.array([37.86,45.82,44.19,38.35,33.64,29.90,27.02,24.67,22.62,20.88,
                                                19.36,17.98,16.74,15.79,14.64,13.86,12.72,11.53  ]) 
       
    prop.twist_distribution         = beta*Units.degrees
    prop.chord_distribution         = b_R*prop.tip_radius    
    prop.radius_distribution        = r_R*prop.tip_radius     
    
    # estimate thickness 
    r_R_data                        = np.array([0.15,0.275,0.367,0.449,0.5,0.55,
                                                0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,0.99])
    t_b_data                        = np.array([0.122,0.105,0.077,0.061,0.055,0.049,0.045,0.041,0.038
                                                ,0.035,0.033,0.031,0.029,0.027,0.026]) 
    b_D_data                        = np.array([0.14485,0.14587,0.1481,
                                                0.1499,0.15061,0.15058,0.14981,0.14831,0.1468,0.14529,0.14268,
                                                0.13764,0.12896,0.11304,0.085])    
    func_max_thickness_distribution = interp1d(r_R_data, t_b_data*b_D_data*2*prop.tip_radius, kind='cubic')   
    prop.max_thickness_distribution = func_max_thickness_distribution(r_R) 
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution  
    
    ospath                          = os.path.abspath(__file__)
    separator                       = os.path.sep
    rel_path                        = os.path.dirname(ospath) + separator   
     
    airfoil_1                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil_1.coordinate_file       = rel_path +'..' + separator + 'Airfoils' + separator + 'E63.txt'
    airfoil_1.polar_files           = [rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_50000.txt'     ,rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_100000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_200000.txt'    ,rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_500000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_1000000.txt']
    airfoil_1.geometry              = import_airfoil_geometry(airfoil_1.coordinate_file,airfoil_1.number_of_points)
    airfoil_1.polars                = compute_airfoil_properties(airfoil_1.geometry,airfoil_1.polar_files)
    prop.append_airfoil(airfoil_1)  

    airfoil_2                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil_2.coordinate_file       = rel_path +'..' + separator + 'Airfoils' + separator + 'Clark_y.txt'
    airfoil_2.polar_files           = [ rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_50000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_100000.txt',rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_200000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_500000.txt',rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_1000000.txt']
    airfoil_2.geometry              = import_airfoil_geometry(airfoil_2.coordinate_file,airfoil_2.number_of_points)
    airfoil_2.polars                = compute_airfoil_properties(airfoil_2.geometry,airfoil_2.polar_files)
    prop.append_airfoil(airfoil_2)      
     
    prop.airfoil_polar_stations     = [0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1]   
    prop.mid_chord_alignment        = np.zeros_like(prop.chord_distribution) #  prop.chord_distribution/4. - prop.chord_distribution[0]/4.       
    return prop 