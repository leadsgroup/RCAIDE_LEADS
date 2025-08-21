# RCAIDE/Components/Landing_Gear/Landing_Gear.py
# 
# Created:  Nov 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports     
from RCAIDE.Library.Components import Component    

# ---------------------------------------------------------------------------------------------------------------------- 
#  Landing_Gear
# ----------------------------------------------------------------------------------------------------------------------  
class Landing_Gear(Component):
    """
    Base class for aircraft landing gear components providing core functionality for 
    both main and nose gear configurations.

    Attributes
    ----------
    tag : str
        Unique identifier for the landing gear component, defaults to 'landing_gear'
        
    tire_diameter : float
        Diameter of the landing gear tire, defaults to 0
        
    tire_width : float
        Width of the landing gear tire, defaults to 0 
        
    gear_extended : bool
        Flag indicating whether the landing gear is in extended position, 
        defaults to False
        
    wheels : int
        Number of wheels per landing gear unit, defaults to 0
        
    number_of_gear_types_in_tandem : int
        Number of wheels in tandem on landing gear, defaults to None
        
    number_of_wheels_in_gear_type : int
        Configuration of wheels in tandem, defaults to None
        
    fairing : bool
        Configuration of wheels in tandem, defaults to 0

    Notes
    -----
    This class serves as the foundation for specific landing gear types, providing:
    
    * Basic geometric properties
    * Configuration parameters
    * State tracking (extended/retracted)
    
    **Major Assumptions**
    
    * Landing gear components are rigid bodies
    * Tire and strut properties are uniform within each unit
    * Gear state is binary (either fully extended or fully retracted)
    
    **Definitions**

    'struct length'
        The main structural member of the landing gear that absorbs landing loads
        and supports the wheel assembly
        
    References
    ----------
    [1] https://www.faa.gov/documentLibrary/media/Order/Construction_5300_7.pdf

    See Also
    --------
    RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear
        Implementation for main landing gear
    RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear
        Implementation for nose landing gear
    """

    def __defaults__(self):
        """
        Sets default values for the landing gear attributes.
        """
        self.tag                             = 'landing_gear'   
        self.tire_diameter                   = 0  
        self.rim_diameter                    = 0  
        self.tire_width                      = 0 
        self.strut_length                    = 0  
        self.wheels                          = 0
        self.symmetric                       = False
        self.number_of_gear_types_in_tandem  = None
        self.number_of_wheels_in_gear_type   = None  
        self.gear_extended                   = False
        self.fairing                         = False