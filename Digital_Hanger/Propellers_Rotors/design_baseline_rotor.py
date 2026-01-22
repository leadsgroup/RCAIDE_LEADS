# Imports
import RCAIDE
from RCAIDE.Core import Units, Data   
import numpy as np   

# design propeller 

def design_baseline_rotor():          
    prop                            = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()
    prop.inputs                     = Data() 
    prop.inputs.blade_pitch_command       = 5*Units.degrees 
    prop.inputs.y_axis_rotation     = 20*Units.degrees 
    prop.tag                        = 'baseline_rotor'
    prop.tip_radius                 = 1
    prop.hub_radius                 = 0.1 
    prop.number_of_blades           = 8   # distance between blades is 45 degrees  
    prop.airfoil_flag               = True
    num_sec                         = 9  
    prop.radius_distribution        = np.linspace(0.15,0.95,num_sec)
    prop.thickness_to_chord         = np.ones(num_sec)*0.1
    prop.chord_distribution         = np.ones(num_sec)*0.2
    prop.max_thickness_distribution = prop.thickness_to_chord*prop.chord_distribution
    prop.twist_distribution         = np.ones(num_sec)*10*Units.degrees    
    prop.mid_chord_alignment        = np.zeros_like(prop.chord_distribution) # blade is centered about y axis 
    prop.airfoil_polar_stations     = list(np.zeros(num_sec).astype(int)) 
    
    return prop