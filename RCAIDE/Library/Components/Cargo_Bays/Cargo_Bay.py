# RCAIDE/Library/Components/Cargo_Bays/Cargo_Bay.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports   
from RCAIDE.Library.Components  import Component   
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_cuboid_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity import compute_cargo_bay_center_of_gravity


import  numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  Cargo_Bay
# ----------------------------------------------------------------------------------------------------------------------              
class Cargo_Bay(Component):
    """Base class for a cargo bay     
    """          
    def __defaults__(self):
        """This sets the default power draw.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        N/A
        """             
        self.tag           = 'cargo_bay'
        self.length        = 1.0
        self.width         = 1.0
        self.height        = 1.0 
        self.density       = 0.0 
        self.power_draw    = 0.0
        
    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for the  cargo bay.

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the center of gravity calculation
        """
        _ = compute_cargo_bay_center_of_gravity(self)
        return

    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the cargo bay.

        Parameters
        ---------- 
        center_of_gravity : list, optional
            Reference point coordinates, defaults to [[0, 0, 0]]
            
        """ 
        _,_ = compute_cuboid_moment_of_inertia(self,self.length,self.width,self.height, inner_length = 0, inner_width = 0, inner_height = 0, center_of_gravity = np.array([[0,0,0]]))  
        return       