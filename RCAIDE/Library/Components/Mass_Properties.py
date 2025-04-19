# RCAIDE/Library/Compoments/Mass_Properties.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE Imports 
from RCAIDE.Framework.Core import Data

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Mass_Properties
# ----------------------------------------------------------------------------------------------------------------------        
class Mass_Properties(Data):
    """
    A class containing mass and inertial properties for physical components.

    Attributes
    ----------
    mass : float
        Total mass of the component, defaults to 0.0
        
    volume : float
        Volume of the component, defaults to 0.0
        
    center_of_gravity : ndarray
        3D coordinates [x, y, z] of the component's center of gravity, 
        defaults to [0.0, 0.0, 0.0]
        
    moments_of_inertia : Data
        Collection of inertial properties
        
        - center : ndarray
            Reference point for inertia calculations [x, y, z],
            defaults to [0.0, 0.0, 0.0]
        - tensor : ndarray
            3x3 inertia tensor about the reference point,
            defaults to zero matrix

    Notes
    -----
    The Mass_Properties class provides a standardized structure for tracking mass-related 
    properties of components. It includes:
    
    * Basic mass and volume
    * Center of gravity location
    * Full inertia tensor definition
    * Reference point specification

    See Also
    --------
    RCAIDE.Library.Components.Component
        Parent class that uses Mass_Properties
    RCAIDE.Framework.Core.Data
        Base class providing data structure functionality
    """
    def __defaults__(self):
        """
        Sets default values for mass property attributes.
        """         
        self.mass   = 0.0
        self.volume = 0.0
        self.center_of_gravity = np.array([[0.0,0.0,0.0]])
        
        self.moments_of_inertia = Data()
        self.moments_of_inertia.center = np.array([0.0,0.0,0.0])
        self.moments_of_inertia.tensor = np.array([[0.0,0.0,0.0],
                                                  [0.0,0.0,0.0],
                                                  [0.0,0.0,0.0]])