# Imports
import RCAIDE
from RCAIDE.Core import Units, Data  
from RCAIDE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_properties import compute_airfoil_properties
from RCAIDE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry    import import_airfoil_geometry    
from scipy.interpolate import interp1d
import os
import numpy as np  

# design propeller 
                                    
def design_Hubbard_prop(): 
    prop                            = RCAIDE.Library.Components.Powertrain.Converters.Propeller()
    prop.inputs                     = Data()
    prop.inputs.blade_pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'Hubbard_Propeller'  
    prop.tip_radius                 = 0.6096
    prop.hub_radius                 = prop.tip_radius*0.3 
    prop.number_of_blades           = 2  
    r_R                             = np.array([0.3,  0.35, 0.45,0.6 , 0.7  ,0.8 , 0.85 ,0.9  , 0.99 ])
    t_b                             = np.array([0.184931507,0.159452055,0.12109589,0.088493151,0.075068493,
                                                0.065205479,0.060821918,0.056712329,0.049589041 ])
    b_D                             = np.array([ 0.091506849,0.091232877,0.089863014,
                                                0.083835616,0.077260274,
                                                0.069315068,0.065205479,0.06109589,0.052876712])
    beta                            = np.array([ 0.089315068,0.07890411,0.064383562,
                                                 0.050958904,0.044931507,
                                                 0.040547945,0.038630137,0.036986301,0.034246575])*500 
    
    
    dim = 20                                
    new_radius_distribution         = np.linspace(0.3,0.98,dim )
    func_twist_distribution         = interp1d(r_R, (1.528 + beta )*Units.degrees , kind='cubic')
    func_chord_distribution         = interp1d(r_R,  b_D*2*prop.tip_radius       , kind='cubic')
    func_radius_distribution        = interp1d(r_R, r_R *prop.tip_radius    , kind='cubic')
    func_max_thickness_distribution = interp1d(r_R, t_b*b_D*2*prop.tip_radius  , kind='cubic')  
    
    prop.twist_distribution         = func_twist_distribution(new_radius_distribution)     
    prop.chord_distribution         = func_chord_distribution(new_radius_distribution)         
    prop.radius_distribution        = func_radius_distribution(new_radius_distribution)        
    prop.max_thickness_distribution = func_max_thickness_distribution(new_radius_distribution) 
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution    
    
    ospath                             = os.path.abspath(__file__)
    separator                          = os.path.sep
    rel_path                           = os.path.dirname(ospath) + separator  
     
    airfoil_1                          = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil_1.coordinate_file          =  rel_path +'..' + separator + 'Airfoils' + separator + 'Clark_y.txt'
    airfoil_1.polar_files              = [rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_50000.txt',
                                          rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_100000.txt',
                                          rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_200000.txt',
                                          rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_500000.txt',
                                          rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_1000000.txt']
    airfoil_1.geometry                 = import_airfoil_geometry(airfoil_1.coordinate_file,airfoil_1.number_of_points)
    airfoil_1.polars                   = compute_airfoil_properties(airfoil_1.geometry,airfoil_1.polar_files)
    prop.append_airfoil(airfoil_1)  
    
    airfoil_polar_stations             =  np.zeros(len(prop.thickness_to_chord)) 
    prop.mid_chord_alignment               = np.zeros_like(prop.chord_distribution) #  np.zeros_like(prop.chord_distribution) # prop.chord_distribution/4. - prop.chord_distribution[0]/4.
    prop.airfoil_polar_stations        = list(airfoil_polar_stations.astype(int) )  
    
    return prop