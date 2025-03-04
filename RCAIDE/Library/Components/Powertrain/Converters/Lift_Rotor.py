# RCAIDE/Library/Components/Propulsors/Converters/Lift_Rotor.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports 
from RCAIDE.Framework.Core import Data
from .Rotor import Rotor

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  LIFT ROTOR CLASS
# ----------------------------------------------------------------------------------------------------------------------           
class Lift_Rotor(Rotor):
    """
    A lift rotor component model for vertical lift applications, inheriting from the base Rotor class.

    Attributes
    ----------
    tag : str
        Identifier for the lift rotor. Default is 'lift_rotor'.
        
    orientation_euler_angles : list
        Vector of angles [rad] defining rotor orientation [θ, φ, ψ]. 
        Default is [0., π/2, 0.] for Z-direction thrust up in vehicle frame.
        
    use_2d_analysis : bool
        Flag for using 2D aerodynamic analysis. Default is False.
        
    variable_pitch : bool
        Flag indicating if rotor has variable pitch capability. Default is False.
        
    hover : Data
        Hover performance parameters
        
        - design_thrust : float
            Design point thrust [N]. Default is None.
        
        - design_torque : float
            Design point torque [N·m]. Default is None.

        - design_power : float
            Design point power [W]. Default is None.
        
        - design_angular_velocity : float
            Design point rotational speed [rad/s]. Default is None.

        - design_tip_mach : float
            Design point blade tip Mach number. Default is None.
        
        - design_freestream_velocity : float
            Design point forward velocity [m/s]. Default is None.

        - design_acoustics : Data
            Acoustic characteristics at design point. Default is None.

        - design_performance : Data
            Performance metrics at design point. Default is None.

        - design_pitch_command : float
            Design point blade pitch command [rad]. Default is 0.0.

        - design_SPL_dBA : float
            Design point sound pressure level [dBA]. Default is None.

        - design_Cl : float
            Design point lift coefficient. Default is None.

        - design_thrust_coefficient : float
            Design point thrust coefficient. Default is None.

        - design_power_coefficient : float
            Design point power coefficient. Default is None.
            
    oei : Data
        One engine inoperative performance parameters
        
        - design_thrust : float
            OEI thrust requirement [N]. Default is None.

        - design_torque : float
            OEI torque requirement [N·m]. Default is None.

        - design_power : float
            OEI power requirement [W]. Default is None.

        - design_angular_velocity : float
            OEI rotational speed [rad/s]. Default is None.

        - design_freestream_velocity : float
            OEI forward velocity [m/s]. Default is None.

        - design_tip_mach : float
            OEI blade tip Mach number. Default is None.

        - design_altitude : float
            OEI design altitude [m]. Default is None.
        
        - design_acoustics : Data
            OEI acoustic characteristics. Default is None.

        - design_pitch_command : float
            OEI blade pitch command [rad]. Default is 0.0.

        - design_performance : Data
            OEI performance metrics. Default is None.

        - design_SPL_dBA : float
            OEI sound pressure level [dBA]. Default is None.

        - design_Cl : float
            OEI lift coefficient. Default is None.

        - design_thrust_coefficient : float
            OEI thrust coefficient. Default is None.

        - design_power_coefficient : float
            OEI power coefficient. Default is None.

    Notes
    -----
    The Lift_Rotor class models rotors specifically designed for vertical lift applications.
    It includes capabilities for:
    * Hover performance analysis
    * One engine inoperative (OEI) conditions
    * Variable pitch operation
    * Acoustic analysis
    * Performance coefficient calculations

    **Definitions**

    'OEI'
        One Engine Inoperative - emergency condition where one engine fails

    'SPL'
        Sound Pressure Level - measure of acoustic intensity
        
    'Tip Mach'
        Mach number at the blade tip, including rotational and forward flight effects

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Rotor
    """     
    def __defaults__(self):
        """This sets the default values for the component to function.
        
        Assumptions:
        None
        
        Source:
        N/A
        
        Inputs:
        None
        
        Outputs:
        None
        
        Properties Used:
        None
        """         


        self.tag                              = 'lift_rotor'
        self.orientation_euler_angles         = [0.,np.pi/2.,0.] # This is Z-direction thrust up in vehicle frame
        self.use_2d_analysis                  = False
        self.variable_pitch                   = False 

        self.hover                            = Data()    
        self.hover.design_thrust              = None
        self.hover.design_torque              = None
        self.hover.design_power               = None
        self.hover.design_angular_velocity    = None
        self.hover.design_tip_mach            = None
        self.hover.design_freestream_velocity = None
        self.hover.design_acoustics           = None
        self.hover.design_performance         = None
        self.hover.design_pitch_command       = 0.0
        self.hover.design_SPL_dBA             = None
        self.hover.design_Cl                  = None
        self.hover.design_thrust_coefficient  = None
        self.hover.design_power_coefficient   = None 
        
        self.oei                              = Data()
        self.oei.design_thrust                = None
        self.oei.design_torque                = None
        self.oei.design_power                 = None
        self.oei.design_angular_velocity      = None
        self.oei.design_freestream_velocity   = None
        self.oei.design_tip_mach              = None  
        self.oei.design_altitude              = None
        self.oei.design_acoustics             = None
        self.oei.design_pitch_command         = 0.0
        self.oei.design_performance           = None
        self.oei.design_SPL_dBA               = None
        self.oei.design_Cl                    = None
        self.oei.design_thrust_coefficient    = None
        self.oei.design_power_coefficient     = None         

        