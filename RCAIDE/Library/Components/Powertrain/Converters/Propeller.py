# RCAIDE/Library/Components/Propulsors/Converters/Propeller.py
# 
# 
# Created:  Mar 2024, M. Clarke 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
 # RCAIDE imports 
from .Rotor import Rotor

# ----------------------------------------------------------------------------------------------------------------------
#  Propeller
# ----------------------------------------------------------------------------------------------------------------------     
class Propeller(Rotor):
    """
    A propeller component model for aircraft propulsion, inheriting from the base Rotor class.

    Attributes
    ----------
    tag : str
        Identifier for the propeller. Default is 'propeller'.
        
    orientation_euler_angles : list
        Vector of angles [rad] defining propeller orientation [θ, φ, ψ]. 
        Default is [0., 0., 0.] for X-direction thrust in vehicle frame.
        
    use_2d_analysis : bool
        Flag for using 2D aerodynamic analysis. Default is False.
        
    variable_pitch : bool
        Flag indicating if propeller has variable pitch capability. Default is False.

    Notes
    -----
    The Propeller class models fixed or variable pitch propellers for aircraft
    propulsion. It includes capabilities for:

    * Forward flight performance analysis
    * Fixed and variable pitch operation
    * Blade element momentum theory calculations
    * Efficiency optimization
    * Acoustic analysis

    The model assumes:

    * Axial inflow (unless modified by orientation angles)
    * Rigid blades
    * Quasi-steady aerodynamics
    * No blade-to-blade aerodynamic interference
    * No compressibility effects (unless explicitly modeled)

    **Definitions**
    
    'Variable Pitch'
        Capability to change blade pitch angle during operation
    'Euler Angles'
        Set of three angles defining propeller orientation relative to vehicle frame
    '2D Analysis'
        Use of two-dimensional airfoil data for blade section analysis

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Rotor
    RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor
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

        self.tag                       = 'propeller'
        self.orientation_euler_angles  = [0.,0.,0.] # This is X-direction thrust in vehicle frame
        self.use_2d_analysis           = False       
        self.variable_pitch            = False
        
