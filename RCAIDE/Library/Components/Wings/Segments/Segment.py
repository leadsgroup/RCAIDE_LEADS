# RCAIDE/Library/Compoments/Wings/Segment.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core     import Data, Container
from RCAIDE.Library.Components import Component   

# ---------------------------------------------------------------------------------------------------------------------- 
#  Segment
# ----------------------------------------------------------------------------------------------------------------------   
class Segment(Component):
    """
    A class representing a wing segment that defines a portion of a wing's geometry.

    Attributes
    ----------
    tag : str
        Unique identifier for the segment, defaults to 'segment'
        
    prev : Component
        Link to previous segment in wing, defaults to None
        
    next : Component
        Link to next segment in wing, defaults to None
        
    percent_span_location : float
        Spanwise location as fraction of wing span, defaults to 0.0
        
    twist : float
        Local twist angle, defaults to 0.0
        
    taper : float
        Local taper ratio, defaults to 0.0
        
    root_chord_percent : float
        Root chord as fraction of wing root chord, defaults to 0.0
        
    dihedral_outboard : float
        Dihedral angle of outboard section, defaults to 0.0
        
    thickness_to_chord : float
        Local thickness-to-chord ratio, defaults to 0.0
        
    sweeps : Data
        Collection of sweep angles
        
        - quarter_chord : float
            Quarter-chord sweep angle, defaults to 0.0
        - leading_edge : float
            Leading edge sweep angle, defaults to None
            
    chords : Data
        Collection of chord lengths
        
        - mean_aerodynamic : float
            Mean aerodynamic chord, defaults to 0.0
            
    areas : Data
        Collection of area measurements
        
        - reference : float
            Reference area, defaults to 0.0
        - exposed : float
            Exposed area, defaults to 0.0
        - wetted : float
            Wetted area, defaults to 0.0
            
    Airfoil : Container
        Collection of airfoil definitions, initialized empty

    Notes
    -----
    Wing segments are used to build up complete wing geometries by defining 
    local properties at various spanwise locations.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Wing
        Parent wing class that uses segments
    RCAIDE.Library.Components.Wings.Main_Wing
        Main wing implementation using segments
    """ 

    def __defaults__(self):
        """
        Sets default values for the wing segment attributes.
        """         
        self.tag                     = 'segment'
        self.prev                    = None
        self.next                    = None  
        self.percent_span_location   = 0.0
        self.twist                   = 0.0
        self.taper                   = 0.0
        self.root_chord_percent      = 0.0
        self.dihedral_outboard       = 0.0
        self.thickness_to_chord      = 0.0
        
        self.sweeps                  = Data()
        self.sweeps.quarter_chord    = 0.0
        self.sweeps.leading_edge     = None
    
        self.chords                  = Data()
        self.chords.mean_aerodynamic = 0.0
        
        self.areas                   = Data()
        self.areas.reference         = 0.0
        self.areas.exposed           = 0.0
        self.areas.wetted            = 0.0

        self.airfoil                 = None
        
    def append_airfoil(self, airfoil):
        """
        Adds an airfoil definition to the segment.

        Parameters
        ----------
        airfoil : Data
            Airfoil data to be added to the segment
        """  
        # Assert database type
        if not isinstance(airfoil,RCAIDE.Library.Components.Airfoils.Airfoil):
            raise Exception('input component must be of type Airfoil')

        # store data
        self.airfoil = airfoil

         
class Segment_Container(Container):
    """
    Container class for managing wing segments. Provides organization and 
    access methods for segment components.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Segments.Segment
        The segment components stored in this container
    """     

    def get_children(self):
        """
        Returns a list of allowable child component types for the segment container.

        Returns
        -------
        list
            Empty list as segments do not contain child components
        """       
        return []
