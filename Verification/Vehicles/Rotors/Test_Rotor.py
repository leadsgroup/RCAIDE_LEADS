# Test_Propeller.py
#
# Created:  Dec 2024 M. Clarke

# Imports
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor import design_lift_rotor
from RCAIDE.load    import load as load_rotor
from RCAIDE.save    import save as save_rotor  
 
import os
import numpy as np 
from copy import deepcopy    

# design rotor  
def Test_Rotor(new_regression=True):
    
    Hover_Load = 26487.0*1.1
    
    # Lift Rotor Design              
    lift_rotor                                             = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()   
    lift_rotor.tag                                         = 'lift_rotor_1'   
    lift_rotor.origin                                      = [[-0.073 ,  1.950 , 1.2]] 
    lift_rotor.active                                      = True          
    lift_rotor.tip_radius                                  = 2.8/2
    lift_rotor.hub_radius                                  = 0.1 
    lift_rotor.number_of_blades                            = 3     
    lift_rotor.hover.design_altitude                       = 40 * Units.feet  
    lift_rotor.hover.design_thrust                         = Hover_Load/8
    lift_rotor.hover.design_freestream_velocity            = np.sqrt(lift_rotor.hover.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))  
    lift_rotor.oei.design_altitude                         = 40 * Units.feet  
    lift_rotor.oei.design_thrust                           = Hover_Load/7  
    lift_rotor.oei.design_freestream_velocity              = np.sqrt(lift_rotor.oei.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))   

    ospath                                                 = os.path.abspath(__file__)
    separator                                              = os.path.sep
    rel_path                                               = os.path.dirname(ospath) + separator + '..'  + separator
    
    airfoil                                                = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                                = rel_path + 'Airfoils' + separator + 'NACA_4412.txt'
    airfoil.polar_files                                    = [rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt' ,
                                                             rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt' ,
                                                              rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt' ,
                                                              rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt' ,
                                                              rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt',
                                                              rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_3500000.txt',
                                                              rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_5000000.txt',
                                                              rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_7500000.txt' ]
    lift_rotor.append_airfoil(airfoil)                         
    lift_rotor.airfoil_polar_stations                      = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    
    
    test_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..' + separator + '..' + separator + 'Tests' + separator + 'propulsion'))
    
    if new_regression:
        design_lift_rotor(lift_rotor)
        save_rotor(lift_rotor, os.path.join(test_dir, 'test_rotor.res'))
    else:
        regression_lift_rotor = deepcopy(lift_rotor)
        design_lift_rotor(regression_lift_rotor, iterations=2)
        loaded_lift_rotor = load_rotor(os.path.join(test_dir, 'test_rotor.res'))
        
        for key,item in lift_rotor.items():
            lift_rotor[key] = loaded_lift_rotor[key] 
    
    return lift_rotor