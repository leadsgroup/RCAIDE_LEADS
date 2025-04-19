# RCAIDE/Library/Compoments/Network.py
# 
# Created:  Jul 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Container as ContainerBase
from RCAIDE.Framework.Core import Data
from .Mass_Properties import Mass_Properties

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------------------------------------------------------        
class Network(Data):
    """
    Base class for component networks that manage connections and interactions between 
    system components.

    Attributes
    ----------
    tag : str
        Unique identifier for the network, defaults to 'Network'
        
    mass_properties : Mass_Properties
        Mass and inertia properties, initialized empty
        
    origin : ndarray
        3D coordinates [x, y, z] defining network's reference point, 
        defaults to [0.0, 0.0, 0.0]
        
    inputs : Data
        Collection of network input parameters, initialized empty
        
    outputs : Data
        Collection of network output parameters, initialized empty

    Notes
    -----
    The Network class serves as a framework for managing interconnected components. 
    It provides:
    
    * Input/output parameter management
    * Mass tracking for network components
    * Spatial positioning capabilities
    * Container functionality for sub-networks

    See Also
    --------
    RCAIDE.Library.Components.Mass_Properties
        Class containing mass and inertia data
    RCAIDE.Framework.Core.Data
        Parent class providing data structure functionality
    """
    def __defaults__(self):
        """
        Sets default values for the network attributes.
        """         
        self.tag             = 'Network' 
        self.mass_properties = Mass_Properties()
        self.origin          = np.array([[0.0,0.0,0.0]]) 
        self.inputs          = Data()
        self.outputs         = Data()
        
# ----------------------------------------------------------------------------------------------------------------------
#  Network Container
# ----------------------------------------------------------------------------------------------------------------------     
class Container(ContainerBase):
    """
    Container class for managing collections of networks.

    Notes
    -----
    The Container class provides organization and mass calculation functionality 
    for groups of networks. Key features include:
    
    * Recursive mass summation
    * Moment calculation about reference points
    * Network hierarchy management

    See Also
    --------
    RCAIDE.Framework.Core.Container
        Parent class providing base container functionality
    """
    pass
    
# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------

Network.Container = Container
