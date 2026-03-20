# RCAIDE/Library/Components/Powertrain/Sources/Fuel_Tanks/Fuel_Tank.py
# 
# 
# Created:  Mar 2024, M. Clarke 
# Modified: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Components.Powertrain.Sources.Source    import Source   
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks   import * 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions import *
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_non_integral_tank_volume import *
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_cuboid_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_cuboid_center_of_gravity

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Tank
# ---------------------------------------------------------------------------------------------------------------------     
class Fuel_Tank(Source):
    """
    Base class for aircraft fuel tank implementations
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel tank (default: 'fuel_tank')
        
    fuel_selector_ratio : float
    fuel_flow_split_ratio : float
        Ratio of fuel flow allocation (default: 1.0)
        
    mass_properties.empty_mass : float
        Mass of empty tank structure [kg] (default: 0.0)
        
    secondary_fuel_flow : float
        Secondary fuel flow rate [kg/s] (default: 0.0)
        
    fuel : Component, optional
        Fuel type stored in tank (default: None)

    Notes
    -----
    The fuel tank base class provides common attributes and methods for
    different types of aircraft fuel tanks. It handles basic fuel storage
    and flow management functionality.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Central_Fuel_Tank
        Center section fuel tank
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank
        Wing-mounted fuel tank
    """
    
    def __defaults__(self):
        """
        Sets default values for fuel tank attributes
        """          
        self.tag                            = 'fuel_tank'  
        self.fuel                           = None
        self.secondary_mass_flow_rate       = 0.0   #kg/s
        self.wall_clearance                 = 0.0
        self.wall_thickness                 = 1E-3
        self.fuel_flow_split_ratio          = None
        self.xz_plane_symmetric             = True
        self.wing_tag                       = None
        self.fuselage_tag                   = None
        self.bwb_aft_tank                   = False
        self.lengths                        = Data()
        self.lengths.external               = 0.0
        self.lengths.interal                = 0.0  
        self.widths                         = Data()
        self.widths.external                = 0.0
        self.widths.interal                 = 0.0
        self.heights                        = Data()
        self.heights.external               = 0.0
        self.heights.internal               = 0.0 
        self.diameters                      = Data()
        self.diameters.external             = 0.0
        self.diameters.internal             = 0.0 
        self.segments_bounding_tank         = [None, None] 
        self.segments_percent_chord_start   = [0.1,0.1]
        self.segments_percent_chord_end     = [0.7,0.7]
        self.percent_span_location          = 0.0


    def unpack_unknowns(self,segment):
        return 

    def pack_residuals(self,segment): 
        return        

    def append_unknowns_and_residuals(self,segment):
        return         
 
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
    
    def append_segment_conditions(self, segment):
        append_fuel_tank_segment_conditions(self, segment)
        return
     
    def compute_volume(self, wings, fuselages,fuel_tanks):
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
        compute_prismatic_fuel_tank_volume(self)
        return
    
   
    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for a fuel tank.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]]

        Returns
        -------
        I : ndarray
            3x3 moment of inertia tensor in kg*m^2
 
        """ 
        _, _ = compute_cuboid_moment_of_inertia(self,
                                                outer_length=self.lengths.external,
                                                outer_width=self.widths.external,
                                                outer_height=self.heights.external,\
                                                inner_length=self.lengths.external- 2*self.wall_thickness,
                                                inner_width=self.widths.external- 2*self.wall_thickness,
                                                inner_height=self.heights.external- 2*self.wall_thickness,
                                                center_of_gravity=center_of_gravity,
                                                fuel_tank=True) 
                
        return
    
    def compute_performance(self,state,network):
        inputs, outputs, stored_results_flag, stored_source_tag = compute_fuel_tank_performance(self, state,network)
        return inputs, outputs, stored_results_flag, stored_source_tag 
    

    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for a fuel tank.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]]

        Returns
        -------
        I : ndarray
            3x3 moment of inertia tensor in kg*m^2 
        """
        
        _  = compute_cuboid_center_of_gravity(self, length=self.lengths.external)
            
        return