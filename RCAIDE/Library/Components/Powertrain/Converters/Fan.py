# RCAIDE/Library/Components/Propulsors/Converters/Fan.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Fan.append_fan_conditions import append_fan_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Fan  
# ----------------------------------------------------------------------------------------------------------------------
class Fan(Converter):
    """
    A fan component model for turbofan and ducted fan propulsion systems.

    Attributes
    ----------
    tag : str
        Identifier for the fan component. Default is 'Fan'.
        
    polytropic_efficiency : float
        Efficiency of the compression process accounting for losses. Default is 1.0.
        
    pressure_ratio : float
        Ratio of outlet to inlet total pressure. Default is 1.0.
        
    angular_velocity : float
        Rotational speed of the fan [rad/s]. Default is 0.0.

    Notes
    -----
    The Fan class models the compression and energy addition process in a fan stage.
    The model:
    * Calculates work input required for given pressure ratio
    * Accounts for losses through polytropic efficiency
    * Handles variable speed operation
    * Assumes axial flow conditions
    * Models both subsonic and transonic fan operation

    **Major Assumptions**
    * Pressure ratio and efficiency do not change with varying conditions
    * Uniform flow at inlet and exit
    * No radial variations in flow properties
    * Adiabatic process (no heat transfer with surroundings)
    * Steady flow conditions

    **Definitions**

    'Polytropic Efficiency'
        Measure of compression efficiency accounting for real gas effects

    'Pressure Ratio'
        Ratio of exit to inlet total pressure
        
    'Angular Velocity'
        Rotational speed of the fan rotor

    References
    ----------
    [1] Mattingly, J. D., & Boyer, K. M. (2016). Elements of propulsion: Gas 
        turbines and rockets, second edition Jack D. Mattingly, Keith M. Boyer. 
        American Institute of Aeronautics and Astronautics.

    See Also
    --------
    RCAIDE.Library.Components.Component
    RCAIDE.Library.Methods.Powertrain.Converters.Fan.append_fan_conditions
    """
    
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            Pressure ratio and efficiency do not change with varying conditions.

        Source:
            https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/ 
        """         
        #set the default values
        self.tag                            = 'Fan'
        self.polytropic_efficiency          = 1.0
        self.mechanical_efficiency          = 1.0
        self.pressure_ratio                 = 1.0
        self.angular_velocity               = 0


    def append_operating_conditions(self,segment,propulsor): 
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_fan_conditions(self,segment,propulsor_conditions)
        return                                