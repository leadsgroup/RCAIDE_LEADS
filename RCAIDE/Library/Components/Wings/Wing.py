# RCAIDE/Library/Components/Wings/Wing.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core      import Data,Container 
from RCAIDE.Library.Components  import Component   
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_wing_moment_of_inertia import  compute_wing_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_wing_center_of_gravity import  compute_wing_center_of_gravity

import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Wing
# ----------------------------------------------------------------------------------------------------------------------  
class Wing(Component):
    """
    Base class for aircraft lifting surfaces providing core functionality for geometric 
    definition and analysis.

    Attributes
    ----------
    tag : str
        Unique identifier for the wing, defaults to 'wing'
        
    mass_properties : Mass_Properties
        Mass and inertia properties, initialized empty
        
    origin : ndarray
        3D coordinates [x, y, z] defining wing's reference point, defaults to [0.0, 0.0, 0.0] 
        
    vertical : bool
        Flag indicating if wing is vertically oriented, defaults to False
        
    t_tail : bool
        Flag indicating if wing is mounted on vertical tail, defaults to False
        
    taper : float
        Wing taper ratio, defaults to 0.0
        
    dihedral : float
        Wing dihedral angle, defaults to 0.0
        
    aspect_ratio : float
        Wing aspect ratio, defaults to 0.0
        
    thickness_to_chord : float
        Average thickness-to-chord ratio, defaults to 0.0
        
    aerodynamic_center : list
        Location of aerodynamic center [x, y, z], defaults to [0.0, 0.0, 0.0]
        
    percent_span_unexposed : float
        Offset of exposed root from centerline, defaults to 0.0
        
    total_length : float
        Total length of wing, defaults to 0.0
        
    spans : Data
        Collection of span measurements
        
        - projected : float
            Projected span, defaults to 0.0
        - total : float
            Total span including dihedral effects, defaults to 0.0
            
    areas : Data
        Collection of area measurements
        
        - reference : float
            Reference area, defaults to 0.0
        - exposed : float
            Exposed area, defaults to 0.0
        - affected : float
            Area affected by high-lift devices, defaults to 0.0
        - wetted : float
            Wetted area, defaults to 0.0
            
    chords : Data
        Collection of chord measurements
        
        - mean_aerodynamic : float
            Mean aerodynamic chord, defaults to 0.0
        - mean_geometric : float
            Mean geometric chord, defaults to 0.0
        - root : float
            Root chord, defaults to 0.0
        - tip : float
            Tip chord, defaults to 0.0
            
    sweeps : Data
        Collection of sweep angles
        
        - quarter_chord : float
            Quarter-chord sweep angle, defaults to 0.0
        - leading_edge : float
            Leading edge sweep angle, defaults to None 
            
    twists : Data
        Collection of twist angles
        
        - root : float
            Root section twist angle, defaults to 0.0
        - tip : float
            Tip section twist angle, defaults to 0.0
        
    symbolic : bool
        Flag for symbolic computation mode, defaults to False 
        
    vortex_lift : bool
        Flag for vortex lift modeling, defaults to False
        
    transition_x_upper : float
        Upper surface transition location, defaults to 0.0
        
    transition_x_lower : float
        Lower surface transition location, defaults to 0.0
        
    Airfoil : Container
        Collection of airfoil definitions, initialized empty
        
    segments : Container
        Collection of wing segments, initialized empty
        
    control_surfaces : Container
        Collection of control surfaces, initialized empty
        
    fuel_tanks : Container
        Collection of fuel tanks, initialized empty

    Notes
    -----
    The Wing class serves as the foundation for all lifting surfaces in RCAIDE. 
    It provides:
    
    * Geometric definition capabilities
    * Segment management
    * Control surface integration
    * Fuel tank integration
    * Mass properties computation

    See Also
    --------
    RCAIDE.Library.Components.Wings.Main_Wing
        Primary lifting surface implementation
    RCAIDE.Library.Components.Wings.Horizontal_Tail
        Horizontal stabilizer implementation
    RCAIDE.Library.Components.Wings.Vertical_Tail
        Vertical stabilizer implementation
    """      

    def __defaults__(self):
        """
        Sets default values for the wing attributes.
        """         
        self.tag                                    = 'wing'
        self.origin                                 = np.array([[0.0,0.0,0.0]]) 
        self.xz_plane_symmetric                     = True 
        self.vertical                               = False
        self.t_tail                                 = False 
        self.taper                                  = 0.0
        self.dihedral                               = 0.0
        self.aspect_ratio                           = 0.0
        self.thickness_to_chord                     = 0.0
        self.aerodynamic_center                     = [0.0,0.0,0.0]
        self.percent_span_unexposed                 = 0.0
        self.total_length                           = 0.0 
             
        self.spans                                  = Data()
        self.spans.projected                        = 0.0
        self.spans.total                            = 0.0
                                                    
        self.areas                                  = Data()
        self.areas.reference                        = 0.0
        self.areas.exposed                          = 0.0
        self.areas.affected                         = 0.0
        self.areas.wetted                           = 0.0
        self.areas.planform                         = 0.0
                                                    
        self.chords                                 = Data()
        self.chords.mean_aerodynamic                = 0.0
        self.chords.mean_geometric                  = 0.0
        self.chords.root                            = 0.0
        self.chords.tip                             = 0.0
                                                    
        self.sweeps                                 = Data()
        self.sweeps.quarter_chord                   = None
        self.sweeps.leading_edge                    = None         
                                                    
        self.twists                                 = Data()
        self.twists.root                            = 0.0
        self.twists.tip                             = 0.0
                                                     
        self.symbolic                               = False  
        self.vortex_lift                            = False
                                                    
        self.transition_x_upper                     = 0.45
        self.transition_x_lower                     = 0.45
        self.airfoil                                = None 
        
        self.segments                               = Container()
        self.control_surfaces                       = Container()
   
        self.structural                             = Data()  
        self.structural.rib                         = False   
        self.structural.front_spar_percent_chords  = Data()
        self.structural.rear_spar_percent_chords   = Data()
        self.structural.front_spar_percent_chords.root    = 0.15  
        self.structural.rear_spar_percent_chord.root      = 0.65  
        self.structural.front_spar_percent_chords.tip    = 0.15  
        self.structural.rear_spar_percent_chord.tip      = 0.65  
        self.structural.stringer_percent_chords            = []         

    def append_segment(self, segment):
        """
        Adds a new segment to the wing's segment container.

        Parameters
        ----------
        segment : Data
            Wing segment to be added
        """
        # Assert database type
        if not isinstance(segment,RCAIDE.Library.Components.Wings.Segments.Segment):
            raise Exception('input component must be of type Segment')

        # Store data
        self.segments.append(segment)

        return
    
    def append_airfoil(self, airfoil):
        """
        Adds an airfoil definition to the wing.

        Parameters
        ----------
        airfoil : Data
            Airfoil data to be added
        """
        # Assert database type
        if not isinstance(airfoil,RCAIDE.Library.Components.Airfoils.Airfoil):
            raise Exception('input component must be of type Airfoil')

        # Store data
        self.airfoil = airfoil

        return        

    def append_control_surface(self, control_surface):
        """
        Adds a control surface to the wing.

        Parameters
        ----------
        control_surface : Data
            Control surface to be added
        """
        # Assert database type
        if not isinstance(control_surface,Data):
            raise Exception('input control surface must be of type Data()')

        # Store data
        self.control_surfaces.append(control_surface)

        return
    
    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the wing.

        Parameters
        ---------- 
        center_of_gravity : list, optional
            Reference point coordinates, defaults to [[0, 0, 0]]
        fuel_flag : bool, optional
            Flag to include fuel mass, defaults to False

        Returns
        -------
        ndarray
            3x3 moment of inertia tensor
        """ 
        _, _ = compute_wing_moment_of_inertia(self, center_of_gravity) 
        return
    

    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for the wing.

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the center of gravity calculation
        """
        compute_wing_center_of_gravity(self, vehicle)
        return
    
    
class Container(Component.Container):
    def get_children(self):
        """ Returns the components that can go inside
        
        Assumptions:
        None
    
        Source:
        N/A
    
        Inputs:
        None
    
        Outputs:
        None
    
        Properties Used:
        N/A
        """       
        from . import Main_Wing
        from . import Vertical_Tail
        from . import Horizontal_Tail
        
        return [Main_Wing,Vertical_Tail,Horizontal_Tail]


# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------

Wing.Container = Container
