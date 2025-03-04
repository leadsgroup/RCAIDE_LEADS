# RCAIDE/Compoments/Fuselages/Fuselage.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                import Data 
from RCAIDE.Library.Components.Component  import Container
from RCAIDE.Library.Components            import Component
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_fuselage_moment_of_inertia import  compute_fuselage_moment_of_inertia

# ---------------------------------------------------------------------------------------------------------------------- 
#  Fuselage
# ---------------------------------------------------------------------------------------------------------------------- 
class Fuselage(Component):
    """
    Base class for aircraft fuselage components providing core functionality for 
    geometric definition and analysis.

    Attributes
    ----------
    tag : str
        Unique identifier for the fuselage component, defaults to 'fuselage'
        
    origin : list
        3D coordinates [x, y, z] defining the fuselage's reference point, 
        defaults to [[0.0, 0.0, 0.0]]
        
    aerodynamic_center : list
        3D coordinates [x, y, z] of the fuselage's aerodynamic center, 
        defaults to [0.0, 0.0, 0.0]
        
    differential_pressure : float
        Cabin pressurization differential, defaults to 0.0
        
    seats_abreast : float
        Number of passenger seats arranged side-by-side, defaults to 0.0
        
    seat_pitch : float
        Longitudinal distance between seat rows in meters, defaults to 0.0
        
    number_coach_seats : float
        Total number of economy class seats, defaults to 0.0
        
    areas : Data
        Collection of fuselage area measurements
        
        - front_projected : float
            Frontal area, defaults to 0.0
        - side_projected : float
            Side profile area, defaults to 0.0
        - wetted : float
            Total wetted surface area, defaults to 0.0
            
    effective_diameter : float
        Equivalent circular diameter, defaults to 0.0
        
    width : float
        Maximum fuselage width, defaults to 0.0
        
    heights : Data
        Collection of height measurements along the fuselage
        
        - maximum : float
            Maximum height in meters, defaults to 0.0
        - at_quarter_length : float
            Height at 25% of fuselage length, defaults to 0.0
        - at_three_quarters_length : float
            Height at 75% of fuselage length, defaults to 0.0
        - at_wing_root_quarter_chord : float
            Height at wing root quarter chord, defaults to 0.0
        - at_vertical_root_quarter_chord : float
            Height at vertical tail root quarter chord, defaults to 0.0
            
    lengths : Data
        Collection of length measurements
        
        - nose : float
            Length of nose section, defaults to 0.0
        - tail : float
            Length of tail section, defaults to 0.0
        - total : float
            Total fuselage length, defaults to 0.0
        - cabin : float
            Length of passenger cabin, defaults to 0.0
        - fore_space : float
            Length of forward equipment bay, defaults to 0.0
        - aft_space : float
            Length of aft equipment bay, defaults to 0.0
            
    x_rotation : float
        Rotation angle about x-axis, defaults to 0.0
        
    y_rotation : float
        Rotation angle about y-axis, defaults to 0.0
        
    z_rotation : float
        Rotation angle about z-axis, defaults to 0.0
        
    fineness : Data
        Fineness ratio parameters
        
        - nose : float
            Ratio of nose length to maximum diameter, defaults to 0.0
        - tail : float
            Ratio of tail length to maximum diameter, defaults to 0.0
            
    nose_curvature : float
        Shape parameter defining nose profile curvature, defaults to 1.5
        
    tail_curvature : float
        Shape parameter defining tail profile curvature, defaults to 1.5
        
    fuel_tanks : Container
        Collection of fuel tank components within the fuselage, initialized empty
        
    vsp_data : Data
        OpenVSP-specific geometry parameters
        
        - xsec_surf_id : str
            OpenVSP cross-section surface identifier
        - xsec_num : int
            Number of cross-sections in OpenVSP model, defaults to None
            
    segments : Container
        Collection of fuselage segment components, initialized empty

    Notes
    -----
    This class serves as the foundation for all fuselage types in RCAIDE, providing:
    
    * Geometric definition capabilities
    * Integration with OpenVSP for visualization and analysis
    * Support for fuel tank and segment management
    * Basic structural configuration parameters
    
    **Definitions**

    'Fineness Ratio'
        Ratio of length to maximum diameter, important for drag characteristics
        
    'Wetted Area'
        Total surface area exposed to airflow
        
    'Equipment Bay'
        Non-pressurized volume for systems and equipment storage

    See Also
    --------
    RCAIDE.Library.Components.Fuselages.Tube_Fuselage
        Implementation for conventional tube-and-wing aircraft
    RCAIDE.Library.Components.Fuselages.Blended_Wing_Body_Fuselage
        Implementation for blended wing body aircraft
    """
    
    def __defaults__(self):
        """
        Sets default values for all fuselage attributes.
        """      
        
        self.tag                                    = 'fuselage'
        self.origin                                 = [[0.0,0.0,0.0]]
        self.aerodynamic_center                     = [0.0,0.0,0.0] 
        self.differential_pressure                  = 0.0    
        self.seats_abreast                          = 0.0
        self.seat_pitch                             = 0.0
        self.number_coach_seats                     = 0.0

        self.areas                                  = Data()
        self.areas.front_projected                  = 0.0
        self.areas.side_projected                   = 0.0
        self.areas.wetted                           = 0.0
        
        self.effective_diameter                     = 0.0
        self.width                                  = 0.0  
        
        self.heights                                = Data() 
        self.heights.maximum                        = 0.0
        self.heights.at_quarter_length              = 0.0
        self.heights.at_three_quarters_length       = 0.0
        self.heights.at_wing_root_quarter_chord     = 0.0
        self.heights.at_vertical_root_quarter_chord = 0.0 
        
        self.lengths                                = Data()     
        self.lengths.nose                           = 0.0
        self.lengths.tail                           = 0.0
        self.lengths.total                          = 0.0 
        self.lengths.cabin                          = 0.0 
        self.lengths.fore_space                     = 0.0
        self.lengths.aft_space                      = 0.0 
        
        self.x_rotation                             = 0.0
        self.y_rotation                             = 0.0
        self.z_rotation                             = 0.0 

        self.fineness                               = Data() 
        self.fineness.nose                          = 0.0 
        self.fineness.tail                          = 0.0  
        self.nose_curvature                         = 1.5
        self.tail_curvature                         = 1.5   
    
        self.fuel_tanks                             = Container()
 
        self.vsp_data                               = Data()
        self.vsp_data.xsec_surf_id                  = ''    # There is only one XSecSurf in each VSP geom.
        self.vsp_data.xsec_num                      = None  # Number if XSecs in fuselage geom. 
        self.segments                               = Container()

        self.vsp_data                               = Data()
        self.vsp_data.xsec_id                       = ''       
        self.vsp_data.shape                         = ''                
        
    def append_segment(self,segment):
        """
        Adds a new segment to the fuselage's segment container.

        Parameters
        ----------
        segment : Data
            Fuselage segment to be added
        """

        # Assert database type
        if not isinstance(segment,RCAIDE.Library.Components.Fuselages.Segments.Segment):
            raise Exception('input component must be of type Segment')

        # Store data
        self.segments.append(segment)

        return
    
    def append_fuel_tank(self,fuel_tank):
        """
        Adds a new fuel tank to the fuselage's fuel tank container.

        Parameters
        ----------
        fuel_tank : Data
            Fuel tank component to be added
        """

        # Assert database type
        if not isinstance(fuel_tank,Data):
            raise Exception('input component must be of type Data()')
    
        # Store data
        self.Fuel_Tanks.append(fuel_tank)

        return 

    def compute_moment_of_inertia(self, center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the fuselage.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]]

        Returns
        -------
        I : ndarray
            3x3 moment of inertia tensor in kg*m^2

        See Also
        --------
        RCAIDE.Library.Methods.Weights.Moment_of_Inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """
        I = compute_fuselage_moment_of_inertia(self,center_of_gravity) 
        return I    