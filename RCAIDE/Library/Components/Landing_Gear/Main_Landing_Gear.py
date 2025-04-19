# RCAIDE/Compoments/Landing_Gear/Main_Landing_Gear.py
# 
# Created:  Nov 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports     
from .Landing_Gear import Landing_Gear

# ---------------------------------------------------------------------------------------------------------------------- 
#  Main_Landing_Gear
# ---------------------------------------------------------------------------------------------------------------------- 
class Main_Landing_Gear(Landing_Gear):
    """
    Main landing gear component that handles the primary loads during landing, takeoff, 
    and ground operations.

    Attributes
    ----------
    tag : str
        Unique identifier for the main landing gear component, defaults to 'main_gear'
        
    units : float
        Number of main landing gear units in the assembly, defaults to 0
        
    strut_length : float
        Length of the main gear strut assembly, defaults to 0
        
    tire_diameter : float
        Diameter of the main gear tires, defaults to 0
        
    wheels : float
        Number of wheels per main landing gear unit, defaults to 0

    Notes
    -----
    The main landing gear typically carries 80-90% of the aircraft weight and is 
    designed to handle the primary impact loads during landing.
    
    **Major Assumptions**
    
    * Symmetrical arrangement about aircraft centerline
    * All units in an assembly have identical properties
    * Load distribution is uniform across all wheels in a unit
    
    **Definitions**

    'Multi-Bogey'
        Configuration with multiple wheels per strut to distribute loads
        
    'Track Width'
        Lateral distance between main gear units, critical for aircraft stability

    See Also
    --------
    RCAIDE.Library.Components.Landing_Gear.Landing_Gear
        Base landing gear class
    RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear
        Nose gear component that works in conjunction with main gear
    """

    def __defaults__(self):
        """
        Sets default values for the main landing gear attributes.
        """
        self.tag           = 'main_gear'
        self.units         = 0. # number of main landing gear units        
        self.strut_length  = 0.
        self.tire_diameter = 0. 
        self.wheels        = 0. # number of wheels on the main landing gear 