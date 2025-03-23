#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                              import Units
from RCAIDE.Library.Methods.Powertrain                  import setup_operating_conditions  

# ------------------------------------------------------------------------------
#   Rotor Analysis
# ------------------------------------------------------------------------------ 
def rotor_aerodynamic_analysis(rotor,
                           velocity_range,
                           angular_velocity = 2500*Units.rpm,
                           blade_pitch_command = 0, 
                           angle_of_attack = 0, 
                           altitude = 0,
                           design_flag=False):
       
    operating_state = setup_operating_conditions(rotor, altitude = altitude,velocity_range=velocity_range, angle_of_attack=angle_of_attack)
         
    rotor_conditions                             = operating_state.conditions.energy.converters[rotor.tag]
    rotor_conditions.design_flag                 = design_flag
    rotor_conditions.omega[:,0]                  = angular_velocity
    rotor_conditions.blade_pitch_command[:,0]    = blade_pitch_command  
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance(rotor,operating_state.conditions)
     
    results = operating_state.conditions.energy.converters[rotor.tag] 
    return  results