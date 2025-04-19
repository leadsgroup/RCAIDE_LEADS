# RCAIDE/Library/Attributes/Planets/Earth.py
# 
#
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------  
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  
 
from .Planet                import Planet 

# ----------------------------------------------------------------------------------------------------------------------  
#  Earth Class
# ----------------------------------------------------------------------------------------------------------------------  
class Earth(Planet):
    """Class for planet Earth with defauk properties  
    """
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """         
        self.tag = 'Earth'
        self.mass              = 5.98e24  # kg
        self.mean_radius       = 6.371e6  # m
        self.sea_level_gravity = 9.80665  # m/s^2    

    def compute_gravity(self, H=0.0):
        """Compute the gravitational acceleration at altitude
            
        Assumptions:       
            None 
            
        Source:
            None 

        Args:
            self                     : air           [unitless]
            H (float)                : altitude      [m]     

        Returns:
            g (float)                : acceleration due to gravity [m/s^2] 
            
        """          
        # Unpack
        g0  = self.sea_level_gravity
        Re  = self.mean_radius 
        
        # Calculate gravity
        gh = g0*(Re/(Re+H))**2.0
        
        return gh