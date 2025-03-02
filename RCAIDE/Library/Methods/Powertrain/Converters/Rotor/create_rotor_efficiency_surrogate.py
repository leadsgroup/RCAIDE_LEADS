# RCAIDE/Methods/Energy/Propulsors/create_rotor_efficiency_surrogate.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
import RCAIDE
from RCAIDE.Framework.Core   import interp2d ,  Units,  Data  
from scipy.interpolate       import RegularGridInterpolator

# package imports 
import numpy as np
import scipy as sp  

# ----------------------------------------------------------------------------------------------------------------------  
#  Rotor Efficiency Surrogate
# ----------------------------------------------------------------------------------------------------------------------   
def create_rotor_efficiency_surrogate(rotor, design_flag=True): 
    D      = rotor.tip_radius*2
    N      = 5
    
    beta_range   = np.linspace(-20, 30,N) *Units.degrees
    
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor: 
        velocity_range = np.linspace(0.1, 30, N) *Units.kts   
        altitude_range = np.linspace(1, 500, N)  *Units.feet
    else: 
        velocity_range = np.linspace(50, 350, N) *Units.kts   
        altitude_range = np.linspace(1, 25000, N) *Units.feet     
     
    reference_omega = 1500*Units.rpm
    efficiency      = np.zeros((N,N,N)) 
    n               = reference_omega/(2.*np.pi)   # Rotations per second
    advance_ratio   = velocity_range/(n*D)
       
    for i in range(len(altitude_range)): 
        for j in range(len(beta_range)): 
            results    = RCAIDE.Library.Methods.Performance.rotor_aerodynamic_analysis(rotor, velocity_range ,  angular_velocity = reference_omega, pitch_command = beta_range[j], angle_of_attack = 0, altitude = altitude_range[i] , design_flag=design_flag) 
            efficiency[:,i,j]  = results.efficiency[:, 0]   
    surrogate   = RegularGridInterpolator((advance_ratio,altitude_range,beta_range),efficiency  ,method = 'linear',   bounds_error=False, fill_value=None)           
    return surrogate
 
