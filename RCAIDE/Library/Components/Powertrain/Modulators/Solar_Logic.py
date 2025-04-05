# RCAIDE/Library/Components/Powertrain/Modulators/Solar_Logic.py
#  
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Library.Components import Component
 
# ----------------------------------------------------------------------------------------------------------------------
#  Solar_Logic
# ----------------------------------------------------------------------------------------------------------------------  
class Solar_Logic(Component):
    """
    Class for managing solar power extraction using Maximum Power Point Tracking
    
    Attributes
    ----------
    MPPT_efficiency : float
        Efficiency of the Maximum Power Point Tracking system (default: 0.0)
        
    system_voltage : float
        Operating voltage of the electrical system (default: 0.0)

    Notes
    -----
    The Solar Logic component manages the complex power flow in solar aircraft,
    including:
    - Maximum power point tracking for solar panels
    - System voltage

    **Definitions**

    'Maximum Power Point Tracking (MPPT)'
        Control strategy that adjusts solar panel voltage to extract maximum
        available power under varying conditions

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Solar_Panels
        Solar panel components
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules
        Battery storage components
    """
    
    def __defaults__(self):
        """
        Sets default values for solar logic attributes
        
        Notes
        -----
        Initializes MPPT efficiency and system voltage to zero. These should be
        set to appropriate values based on the specific system configuration.
        """         
        
        self.MPPT_efficiency = 0.0
        self.system_voltage  = 0.0