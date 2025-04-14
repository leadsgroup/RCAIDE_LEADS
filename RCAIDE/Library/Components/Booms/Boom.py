# RCAIDE/Components/Booms/Boom.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
from RCAIDE.Library.Components import Component 
from RCAIDE.Framework.Core     import Data, Container 
 
# ----------------------------------------------------------------------------------------------------------------------
#  BOOM
# ----------------------------------------------------------------------------------------------------------------------  
class Boom(Component):
    """
    A structural boom component model for aircraft and rotorcraft applications.

    Attributes
    ----------
    tag : str
        Identifier for the boom component. Default is 'boom'.

    origin : list
        3D coordinates of the boom origin [m]. Default is [[0.0,0.0,0.0]].

    aerodynamic_center : list
        3D coordinates of the aerodynamic center [m]. Default is [0.0,0.0,0.0].

    areas : Data
        Collection of area measurements

        - front_projected : float
            Front projected area [m²]. Default is 0.0.
        - side_projected : float
            Side projected area [m²]. Default is 0.0.
        - wetted : float
            Wetted area of the boom [m²]. Default is 0.0.

    effective_diameter : float
        Effective diameter of the boom [m]. Default is 0.0.

    width : float
        Width of the boom [m]. Default is 0.0.

    heights : Data
        Collection of height measurements

        - maximum : float
            Maximum height of the boom [m]. Default is 0.0.
        - at_quarter_length : float
            Height at 25% of boom length [m]. Default is 0.0.
        - at_three_quarters_length : float
            Height at 75% of boom length [m]. Default is 0.0.
        - at_wing_root_quarter_chord : float
            Height at wing root quarter chord [m]. Default is 0.0.
        - at_vertical_root_quarter_chord : float
            Height at vertical root quarter chord [m]. Default is 0.0.

    x_rotation : float
        Rotation angle around x-axis [rad]. Default is 0.0.

    y_rotation : float
        Rotation angle around y-axis [rad]. Default is 0.0.

    z_rotation : float
        Rotation angle around z-axis [rad]. Default is 0.0.

    lengths : Data
        Collection of length measurements

        - nose : float
            Length of the nose section [m]. Default is 0.0.
        - total : float
            Total length of the boom [m]. Default is 0.0.
        - cabin : float
            Length of the cabin section [m]. Default is 0.0.
        - fore_space : float
            Length of space in front [m]. Default is 0.0.
        - aft_space : float
            Length of space in rear [m]. Default is 0.0.

    fineness : Data
        Fineness ratios

        - nose : float
            Fineness ratio of nose. Default is 0.0.
        - tail : float
            Fineness ratio of tail. Default is 0.0.

    differential_pressure : float
        Pressure differential across the boom [Pa]. Default is 0.0.

    vsp_data : Data
        Vehicle Sketch Pad related data

        - xsec_surf_id : str
            VSP cross-section surface identifier. Default is ''.
        - xsec_num : int
            Number of cross-sections in boom geometry. Default is None.

    segments : Container
        Container for boom segments. Default is empty container.

    Notes
    -----
    The Boom class provides a comprehensive framework for modeling structural
    booms in aircraft and rotorcraft, including:

    * Geometric definition
    * Cross-sectional properties
    * Aerodynamic characteristics
    * Structural interfaces
    * VSP integration
    * Segmentation capabilities

    **Definitions**

    'Fineness Ratio'
        Ratio of length to maximum diameter
    'Wetted Area'
        Total surface area exposed to airflow
    'Differential Pressure'
        Pressure difference between inside and outside of boom
    """
    
    def __defaults__(self):
        """ :meta private:"""
        #This sets the default values.
    
        #Assumptions:
        #    None
        
        #Source:
        #    None
              
        
        self.tag                                    = 'boom'
        self.origin                                 = [[0.0,0.0,0.0]]
        self.aerodynamic_center                     = [0.0,0.0,0.0]  
                 
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
        
        self.x_rotation                             = 0.0
        self.y_rotation                             = 0.0
        self.z_rotation                             = 0.0
             
        self.lengths                                = Data()
        self.lengths.nose                           = 0.0 
        self.lengths.total                          = 0.0
        self.lengths.cabin                          = 0.0 
                 
        self.fineness                               = Data()
        self.fineness.nose                          = 0.0
        self.fineness.tail                          = 0.0
             
        self.differential_pressure                  = 0.0 
              
        # For VSP     
        self.vsp_data                               = Data()
        self.vsp_data.xsec_surf_id                  = ''    # There is only one XSecSurf in each VSP geom.
        self.vsp_data.xsec_num                      = None  # Number if XSecs in rotor_boom geom.
                        
        self.segments                               = Container()
        
    def append_segment(self,segment):
        """
        Assumptions:
           None
            
        Source:
           None
        
        Args:
           self       : boom                  [unitless]
           segment    : cross-section of boom [unitless]   
            
        Outputs:
           None 
        """

        # Assert database type
        if not isinstance(segment,Data):
            raise Exception('input component must be of type Data()')

        # Store data
        self.segments.append(segment) 
        
        return 

class Container(Component.Container):
    def get_children(self):
        """ :meta private: """
        #Returns the components that can go inside
    
        #Assumptions:
        #    None
            
        #Source:
        #    None

        #Args:
        #    self       : container of booms [unitless]    
            
        #Outputs:
        #    Boom       : boom               [unitless] 
        
        return [Boom]

# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------ 
Boom.Container = Container
