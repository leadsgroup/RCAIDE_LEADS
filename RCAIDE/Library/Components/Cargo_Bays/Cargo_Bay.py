# RCAIDE/Library/Compoments/Cargo_Bays/Cargo_Bay.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from RCAIDE.Framework.Core      import Container
from RCAIDE.Library.Components  import Component   
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_cuboid_moment_of_inertia

import  numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  Cargo_Bay
# ----------------------------------------------------------------------------------------------------------------------              
class Cargo_Bay(Component):
    """
    A payload component model for representing mission cargo and equipment.

    Attributes
    ----------
    tag : str
        Identifier for the payload. Default is 'payload'.
        
    power_draw : float
        Power consumption of the payload. Default is 0.0.

    Notes
    -----
    The Payload class models mission-specific cargo and equipment, including:

    * Mass properties
    * Power requirements
    * Location in vehicle
    * Energy consumption
    * Operating conditions

    The model can represent:

    * Scientific instruments
    * Cargo containers
    * Passenger accommodations
    * Mission-specific equipment
    * Sensor packages
    * Communication systems

    **Major Assumptions**
    
    * Constant power draw during operation
    * Fixed mass and volume
    * Rigid mounting to vehicle
    * No thermal effects on vehicle
    * No aerodynamic effects
    * Steady-state operation

    See Also
    --------
    RCAIDE.Library.Components.Component 
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
        self.tag        = 'cargo_bay'
        self.length     = 1.0
        self.width      = 1.0
        self.height     = 1.0 
        self.density    = 0.0
        self.payload    = Component() 
        self.baggage    = Component() 
        self.power_draw = 0.0  

    def compute_moment_of_inertia(self, mass,length,width,height, center_of_gravity=[[0, 0, 0]], fuel_flag=False): 
        """
        Computes the moment of inertia tensor for the wing.

        Parameters
        ----------
        mass : float
            Wing mass
        center_of_gravity : list, optional
            Reference point coordinates, defaults to [[0, 0, 0]]
        fuel_flag : bool, optional
            Flag to include fuel mass, defaults to False

        Returns
        -------
        ndarray
            3x3 moment of inertia tensor
        """ 
        I = compute_cuboid_moment_of_inertia(self.origin, mass,length,width,height, length_inner = 0, width_inner = 0, height_inner = 0, center_of_gravity = np.array([[0,0,0]]))  
        return I     
    
    
# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------
Cargo_Bay.Container = Container    