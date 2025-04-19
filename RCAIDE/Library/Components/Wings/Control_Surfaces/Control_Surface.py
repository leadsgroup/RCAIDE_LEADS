# RCAIDE/Compoments/Wings/Control_Surfaces/Control_Surface.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports    
from RCAIDE.Library.Components import Component 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Control Surface
# ---------------------------------------------------------------------------------------------------------------------- 
class Control_Surface(Component):
    """
    Base class for aircraft control surfaces providing core functionality for geometric 
    definition and operation.

    Attributes
    ----------
    tag : str
        Unique identifier for the control surface, defaults to 'control_surface'
        
    span : float
        Spanwise length of the control surface in meters, defaults to 0.0
        
    span_fraction_start : float
        Spanwise location where control surface begins as fraction of wing span, 
        defaults to 0.0
        
    span_fraction_end : float
        Spanwise location where control surface ends as fraction of wing span, 
        defaults to 0.0
        
    hinge_fraction : float
        Chordwise location of hinge line as fraction of control surface chord, 
        defaults to 0.0
        
    chord_fraction : float
        Fraction of wing chord occupied by control surface, defaults to 0.0
        
    sign_duplicate : float
        Sign convention for duplicate control surface deflection (1.0 or -1.0), 
        defaults to 1.0
        
    deflection : float
        Control surface deflection angle, defaults to 0.0
        
    configuration_type : str
        Type of control surface construction (e.g., 'single_slotted'), 
        defaults to 'single_slotted'
        
    gain : float
        Deflection multiplier used for AVL analysis, defaults to 1.0

    Notes
    -----
    The control surface class provides the foundation for all movable aerodynamic 
    surfaces. Key features include:
    
    * Geometric definition relative to parent wing
    * Deflection and hinge line specification
    * Configuration type specification
    * Duplicate surface handling

    **Definitions**

    'Span Fraction'
        Location along wing span measured as fraction from root (0.0) to tip (1.0)
        
    'Chord Fraction'
        Portion of local wing chord occupied by control surface
        
    'Hinge Fraction'
        Location of hinge line measured as fraction of control surface chord
        
    'Sign Duplicate'
        Determines deflection relationship between paired surfaces:
        * 1.0 for synchronized deflection (e.g., flaps)
        * -1.0 for opposite deflection (e.g., ailerons)

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Flap
        High-lift device implementation
    RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron
        Roll control implementation
    RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator
        Pitch control implementation
    RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder
        Yaw control implementation
    """

    def __defaults__(self):
        """
        Sets default values for the control surface attributes.
        """
        self.tag                   = 'control_surface' 
        self.span                  = 0.0
        self.span_fraction_start   = 0.0
        self.span_fraction_end     = 0.0
        
        self.hinge_fraction        = 0.0
        self.chord_fraction        = 0.0
        
        self.sign_duplicate        = 1.0
        self.deflection            = 0.0  
        self.configuration_type    = 'single_slotted'
        
        self.gain                  = 1.0 #deflection multiplier used only for AVL
