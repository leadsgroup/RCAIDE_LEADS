# RCAIDE/Library/Compoments/Configs/Config.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Framework.Core    import Diffed_Data
from RCAIDE.Vehicle          import Vehicle  

# ----------------------------------------------------------------------------------------------------------------------
#  Config
# ----------------------------------------------------------------------------------------------------------------------   
class Config(Diffed_Data, Vehicle):
    """
    The top-level configuration management class for vehicle definition and analysis.

    Attributes
    ----------
    tag : str
        Identifier for the configuration. Default is 'config'.
        
    _base : Vehicle
        Base vehicle configuration containing default settings. Default is empty Vehicle().
        
    _diff : Vehicle
        Differential vehicle configuration containing changes from base. Default is empty Vehicle().

    Notes
    -----
    The Config class manages vehicle configurations through a differential approach,
    providing:

    * Base configuration management
    * Configuration differencing
    * Component relationship tracking
    * Mass properties aggregation
    * Multi-configuration support
    * Dynamic configuration switching

    The class uses a base-diff pattern where:
    
    * Base configuration (_base) contains default settings
    * Differential configuration (_diff) contains changes
    * Final configuration is computed by applying diff to base

    **Definitions**

    'Base Configuration'
        Complete reference configuration from which others are derived
    'Differential Configuration'
        Set of changes applied to base configuration
    'Configuration Switching'
        Process of changing between different configurations
    'Component Relationships'
        Connections and dependencies between components
    """ 
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """     
        self.tag    = 'config'
        self._base  = Vehicle()
        self._diff  = Vehicle()
