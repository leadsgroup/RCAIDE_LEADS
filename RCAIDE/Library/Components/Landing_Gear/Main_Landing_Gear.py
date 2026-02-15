# RCAIDE/Components/Landing_Gear/Main_Landing_Gear.py
# 
# Created:  Nov 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports     
from .Landing_Gear import Landing_Gear
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_cuboid_moment_of_inertia

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
        self.tag                  = 'main_gear'  
        self.xz_plane_symmetric   = True 
        self.units                = 1
        
    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the landing gear.

        Parameters
        ---------- 
        center_of_gravity : list, optional
            Reference point coordinates, defaults to [[0, 0, 0]]
        
        Returns
        -------
        ndarray
            3x3 moment of inertia tensor
        """

        length = self.tire_diameter* 1.1
        width  = self.strut_length* 1.1
        height = self.tire_diameter* 1.1   
        
        _, _  = compute_cuboid_moment_of_inertia(self,length,width,height, inner_length = 0, inner_width = 0, inner_height = 0, center_of_gravity = [[length / 2,width / 2, height / 2]] )  
        return                           