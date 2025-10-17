# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/Non_Integral_Tank.py
# 
# 
# Created:  September 2024, A. Molloy and M. Clarke 
# Modified: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from .Fuel_Tank  import Fuel_Tank 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions import append_fuel_tank_conditions 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_non_integral_tank_volume import *

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Tank
# ---------------------------------------------------------------------------------------------------------------------    
class Non_Integral_Tank(Fuel_Tank):
    """
    Class for modeling non-integral fuel tank characteristics and behavior.
    
    Non-integral tanks are separate fuel storage containers that are not structurally
    integrated into the wing or fuselage. They can be attached to wings, fuselages,
    or configured as specialized tanks for blended wing body (BWB) aircraft.
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel tank (default: 'non_integral_tank')
        
    orientation_euler_angles : list
        Euler angles defining tank orientation [rad] (default: [0., 0., 0.])
        
    bwb_aft_tank : bool
        Flag indicating if tank is configured as BWB aft tank (default: False)
        
    aft_tank_start_root_chord : float, optional
        Starting position of aft tank along root chord [m] (default: None)
        
    aft_tank_end_rood_chord : float, optional
        Ending position of aft tank along root chord [m] (default: None)
        
    aft_tank_end_segment_tag : str, optional
        Tag of wing segment where aft tank ends (default: None)
        
    wing_root_tag : str, optional
        Tag of the root wing for BWB configurations (default: None)
        
    radial_offset : float
        Radial offset from attachment surface [m] (default: 0.0)
        
    wing_tag : str, optional
        Tag of the wing this tank is attached to (default: None)
        
    fuselage_tag : str, optional
        Tag of the fuselage this tank is attached to (default: None)
        
    length : float
        Tank length [m] (default: 0.0)
        
    width : float
        Tank width [m] (default: 0.0)
        
    height : float
        Tank height [m] (default: 0.0)
        
    fuel : Component, optional
        Fuel type stored in tank (default: None)

    radial_offset : float
        Reduction in radius for a tank (default: None)

    Notes
    -----
    Non-integral tanks provide flexibility in fuel storage placement and can be
    positioned to optimize aircraft balance and structural efficiency. They are
    commonly used in aircraft where integral wing tanks are not feasible or
    additional fuel capacity is required.



    **Definitions**

    'Non-integral Tank'
        A fuel storage container that is separate from the primary aircraft structure
        and attached externally to wings, fuselages, or other components.

    'BWB Aft Tank'
        A specialized non-integral tank configuration for blended wing body aircraft
        positioned in the aft section of the wing root.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank
        Base fuel tank class
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank
        Integral fuel tank class
    """
    
    def __defaults__(self):
        """
        Sets default values for central fuel tank attributes
        """          
        self.tag                         = 'non_integral_tank' 
        self.orientation_euler_angles    = [0.,0.,0.]
        self.geometry_type               = 'cylindrical'   # ['prismatic', 'cylindrical']
        self.bwb_aft_tank                = False
        self.aft_tank_start_root_chord   = None
        self.aft_tank_end_rood_chord     = None
        self.aft_tank_end_segment_tag    = None 
        self.wing_root_tag               = None 
        self.radial_offset               = None
        self.aspect_ratio                = None # Defined as the ratio of total length of the tank to the diameter of the tank.
        self.assigned_distributors       = []

    def __init__ (self, compoment=None):
        """
        Initialize  
        """ 
        if compoment is not None:
            if isinstance(compoment, RCAIDE.Library.Components.Wings.Wing):  
                self.wing_tag  =  compoment.tag  
            if isinstance(compoment, RCAIDE.Library.Components.Fuselages.Fuselage):  
                self.fuselage_tag = compoment.tag        
       
    def append_operating_conditions(self,segment):  
        """
        Append fuel tank operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        fuel_line : Component
            Connected fuel line component
        """
        append_fuel_tank_conditions(self,segment)  
        return                      
    
    def compute_volume(self, wings, fuselages):
        """
        Compute the volume of the non-integral fuel tank based on its attachment location.

        Parameters
        ----------
        wings : dict
            Dictionary containing wing components indexed by their tags
        fuselages : dict
            Dictionary containing fuselage components indexed by their tags

        Returns
        -------
        volume : float
            Computed volume of the fuel tank [m³]

        Notes
        -----
        The volume computation method depends on where the tank is attached:
            - If attached to a wing, uses wing geometry and tank dimensions
            - If attached to a fuselage, uses fuselage geometry and tank dimensions  
            - If configured as a BWB aft tank, uses special BWB-specific computation

        **Major Assumptions**
            * Tank dimensions (length, width, height) are properly defined
            * Wing or fuselage components exist in the provided dictionaries
            * For BWB aft tanks, the wing_root_tag is properly set

        See Also
        --------
        RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_non_integral_tank_volume
        """
        if self.geometry_type == 'prismatic':
            compute_prismatic_fuel_tank_volume(self)
        elif self.geometry_type == 'cylindrical':
            if self.wing_tag != None:
                wing = wings[self.wing_tag]  
                compute_wing_non_integral_tank_volume(self,wing)
            else:
                if self.bwb_aft_tank == True:
                    wing = wings[self.wing_root_tag]  
                    compute_bwb_aft_tank_volume(self,wing)
        return