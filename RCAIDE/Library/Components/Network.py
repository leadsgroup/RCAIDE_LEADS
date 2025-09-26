# RCAIDE/Library/Components/Network.py
# 
# Created:  Jul 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Container as ContainerBase
from RCAIDE.Framework.Core import Data 
from RCAIDE.Library.Components import Component
# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------------------------------------------------------        
class Network(Component):
    """
    Base class for component networks that manage connections and interactions between 
    system components.

    Attributes
    ----------
    tag : str
        Unique identifier for the network, defaults to 'Network' 
        
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
    RCAIDE.Framework.Core.Data
        Parent class providing data structure functionality
    """
    def __defaults__(self):
        """
        Sets default values for the network attributes.
        """         
        self.tag             = 'Network'
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
