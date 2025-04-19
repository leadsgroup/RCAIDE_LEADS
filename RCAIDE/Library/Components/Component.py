# RCAIDE/Library/Compoments/Component.py
#  
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

from RCAIDE.Framework.Core import Container as ContainerBase
from RCAIDE.Framework.Core import Data
from .Mass_Properties import Mass_Properties

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Component
# ----------------------------------------------------------------------------------------------------------------------         
class Component(Data):
    """
    Base class for all physical components in an aircraft configuration.

    Attributes
    ----------
    tag : str
        Unique identifier for the component, defaults to 'Component'
        
    mass_properties : Mass_Properties
        Mass and inertia properties, initialized empty
        
    origin : ndarray
        3D coordinates [x, y, z] defining component's reference point, 
        defaults to [0.0, 0.0, 0.0]

    Notes
    -----
    The Component class serves as the foundation for all physical parts in RCAIDE. 
    It provides:
    
    * Basic geometric positioning
    * Mass properties tracking
    * Container functionality for sub-components

    See Also
    --------
    RCAIDE.Library.Components.Mass_Properties
        Class containing mass and inertia data
    RCAIDE.Framework.Core.Data
        Parent class providing data structure functionality
    """
    def __defaults__(self):
        """
        Sets default values for the component attributes.
        """         
        self.tag             = 'Component' 
        self.mass_properties = Mass_Properties()
        self.origin          = np.array([[0.0,0.0,0.0]])
    
        
# ----------------------------------------------------------------------------------------------------------------------
#  Component Container
# ----------------------------------------------------------------------------------------------------------------------     
class Container(ContainerBase):
    """
    Container class for managing collections of components.

    Notes
    -----
    The Container class provides organization and mass calculation functionality 
    for groups of components. Key features include:
    
    * Recursive mass summation
    * Moment calculation about reference points
    * Component hierarchy management

    See Also
    --------
    RCAIDE.Framework.Core.Container
        Parent class providing base container functionality
    """
    pass 
    
# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------

Component.Container = Container
