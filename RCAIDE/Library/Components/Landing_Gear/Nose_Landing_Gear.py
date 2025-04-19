# RCAIDE/Compoments/Landing_Gear/Nose_Landing_Gear.py
# 
# Created:  Nov 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports     
from .Landing_Gear import Landing_Gear

# ---------------------------------------------------------------------------------------------------------------------- 
#  Nose_Landing_Gear
# ----------------------------------------------------------------------------------------------------------------------   
class Nose_Landing_Gear(Landing_Gear):
    """
    Nose landing gear component that provides directional control during ground operations 
    and supports a portion of the aircraft's forward weight.

    Attributes
    ----------
    tag : str
        Unique identifier for the nose landing gear component, defaults to 'nose_gear'
        
    tire_diameter : float
        Diameter of the nose gear tires, defaults to 0
        
    strut_length : float
        Length of the nose gear strut assembly, defaults to 0
        
    units : float
        Number of nose landing gear units, typically 1, defaults to 0
        
    wheels : float
        Number of wheels per nose gear unit, defaults to 0

    Notes
    -----
    The nose landing gear typically carries 10-20% of the aircraft weight and provides 
    steering capability during taxi, takeoff, and landing.
    
    **Major Assumptions**
    
    * Located on aircraft centerline
    * Provides steering capability
    * Designed for lower loads than main gear
    
    **Definitions**
        
    'Steering Angle'
        Maximum deflection angle available for directional control

    See Also
    --------
    RCAIDE.Library.Components.Landing_Gear.Landing_Gear
        Base landing gear class
    RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear
        Main gear component that works in conjunction with nose gear
    """

    def __defaults__(self): 
        """
        Sets default values for the nose landing gear attributes.
        """
        self.tag           = 'nose_gear'
        self.tire_diameter = 0.    
        self.strut_length  = 0.    
        self.units         = 0. # number of nose landing gear    
        self.wheels        = 0. # number of wheels on the nose landing gear 