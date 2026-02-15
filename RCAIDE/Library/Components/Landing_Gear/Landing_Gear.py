# RCAIDE/Library/Components/Landing_Gear/Landing_Gear.py
# 
# Created:  Nov 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Library.Components import Component    
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_cuboid_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity import compute_cuboid_center_of_gravity


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
        self.length                          = 0 
        self.width                           = 0 
        self.height                          = 0
        self.wheel_clearance_factor          = 1.1
        self.number_of_gear_types_in_tandem  = 1
        self.number_of_wheels_in_gear_type   = 1  
        self.gear_extended                   = False
        self.fairing                         = False
         
    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for a landing gear.

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the center of gravity calculation
        """
        
        if type(self) == RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear():
            length = self.strut_length*self.wheel_clearance_factor 
        else:
            length = self.tire_diameter*self.wheel_clearance_factor 
        _  = compute_cuboid_center_of_gravity(self,length) 
        return
                 
    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the landing gear.

        Parameters
        ---------- 
        center_of_gravity : list, optional
            Reference point coordinates, defaults to [[0, 0, 0]] 
        """

        if type(self) == RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear():
            length = self.strut_length*self.wheel_clearance_factor  
            width  = self.tire_diameter*self.wheel_clearance_factor 
            height = self.tire_diameter*self.wheel_clearance_factor 
        else: 
            length = self.tire_diameter*self.wheel_clearance_factor 
            width  = self.strut_length*self.wheel_clearance_factor 
            height = self.tire_diameter*self.wheel_clearance_factor 
        
        _, _  = compute_cuboid_moment_of_inertia(self,length,width,height, inner_length = 0, inner_width = 0, inner_height = 0, center_of_gravity = center_of_gravity  )  
        return                   