# RCAIDE/Compoments/Nacelles/Nacelle.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports    
from RCAIDE.Framework.Core              import Data 
from RCAIDE.Library.Components          import Component   
import scipy as sp
import numpy as np
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Nacelle
# ----------------------------------------------------------------------------------------------------------------------  
class Nacelle(Component):
    """
    Base class for aircraft engine nacelles providing core functionality for geometric 
    definition and analysis.

    Attributes
    ----------
    tag : str
        Unique identifier for the nacelle component, defaults to 'nacelle'
        
    origin : list
        3D coordinates [x, y, z] defining the nacelle's reference point, 
        defaults to [[0.0, 0.0, 0.0]]
        
    aerodynamic_center : list
        3D coordinates [x, y, z] of the nacelle's aerodynamic center, 
        defaults to [0.0, 0.0, 0.0]
        
    areas : Data
        Collection of nacelle area measurements
        
        - front_projected : float
            Frontal area, defaults to 0.0
        - side_projected : float
            Side profile area, defaults to 0.0
        - wetted : float
            Total wetted surface area, defaults to 0.0
            
    diameter : float
        Maximum diameter of the nacelle, defaults to 0.0
        
    inlet_diameter : float
        Diameter of the engine inlet, defaults to 0.0
        
    length : float
        Total length of the nacelle, defaults to 0.0
        
    orientation_euler_angles : list
        Rotation angles [roll, pitch, yaw] in radians, defaults to [0.0, 0.0, 0.0]
        
    flow_through : bool
        Flag indicating if nacelle has flow passing through it, defaults to True
        
    has_pylon : bool
        Flag indicating if nacelle is mounted on a pylon, defaults to True
        
    differential_pressure : float
        Pressure differential between internal and external flow, defaults to 0.0
        
    cowling_airfoil_angle : float
        Angle of the cowling lip airfoil section, defaults to 0.0

    Notes
    -----
    The nacelle class provides the foundation for engine installation design, including:
    
    * Geometric definition capabilities
    * Coordinate transformation utilities
    * Integration with propulsion system analysis
    
    **Major Assumptions**
    
    * Rigid body for structural analysis
    * Quasi-steady aerodynamics
    
    **Definitions**

    'Nacelle Frame'
        Local coordinate system with X out the nose, Z towards the ground, 
        and Y out the right side
        
    'Vehicle Frame'
        Aircraft coordinate system with X towards the tail, Z towards the ceiling, 
        and Y out the right wing

    See Also
    --------
    RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle
        Implementation for axisymmetric nacelles
    RCAIDE.Library.Components.Nacelles.Stack_Nacelle
        Implementation for stacked segment nacelles
    """
    
    def __defaults__(self):
        """
        Sets default values for the nacelle attributes.
        """      
        self.tag                      = 'nacelle'
        self.origin                   = [[0.0,0.0,0.0]]
        self.aerodynamic_center       = [0.0,0.0,0.0]  
        self.areas                    = Data()
        self.areas.front_projected    = 0.0
        self.areas.side_projected     = 0.0
        self.areas.wetted             = 0.0 
        self.diameter                 = 0.0 
        self.inlet_diameter           = 0.0
        self.length                   = 0.0   
        self.orientation_euler_angles = [0.,0.,0.]    
        self.flow_through             = True
        self.has_pylon                = True
        self.differential_pressure    = 0.0    
        self.cowling_airfoil_angle    = 0.0

    def append_operating_conditions(self, segment, propulsor): 
        """
        Placeholder for adding operating conditions to the nacelle.

        Parameters
        ----------
        segment : Data
            Flight segment data
        propulsor : Data
            Propulsion system data
        """
        return

    def nac_vel_to_body(self):
        """
        Computes rotation matrix from nacelle velocity frame to body frame.

        Returns
        -------
        ndarray
            3x3 rotation matrix

        Notes
        -----
        Assumes two nacelle frames:
        * Vehicle frame describing location
        * Velocity frame for aerodynamic calculations
        """
        body2nacvel = self.body_to_nac_vel()
        r = sp.spatial.transform.Rotation.from_matrix(body2nacvel)
        r = r.inv()
        rot_mat = r.as_matrix()
        return rot_mat
    
    def body_to_nac_vel(self):
        """
        Computes rotation matrix from body frame to nacelle velocity frame.

        Returns
        -------
        ndarray
            3x3 rotation matrix

        Notes
        -----
        Transformation sequence:
        1. Body to vehicle frame
        2. Vehicle to nacelle vehicle frame
        3. Nacelle vehicle to nacelle velocity frame
        """
        # Go from body to vehicle frame
        body_2_vehicle = sp.spatial.transform.Rotation.from_rotvec([0,np.pi,0]).as_matrix()
        
        # Go from vehicle frame to nacelle vehicle frame
        rots = np.array(self.orientation_euler_angles) * 1. 
        vehicle_2_nac_vec = sp.spatial.transform.Rotation.from_rotvec(rots).as_matrix()        
        
        # Go from nacelle vehicle frame to nacelle velocity frame
        nac_vec_2_nac_vel = self.vec_to_vel()
        
        # Combine transformations
        rot1 = np.matmul(body_2_vehicle,vehicle_2_nac_vec)
        rot_mat = np.matmul(rot1,nac_vec_2_nac_vel) 
        return rot_mat    
    
    def vec_to_vel(self):
        """
        Computes rotation matrix from nacelle vehicle frame to nacelle velocity frame.

        Returns
        -------
        ndarray
            3x3 rotation matrix
        """
        rot_mat = sp.spatial.transform.Rotation.from_rotvec([0,np.pi,0]).as_matrix()
        return rot_mat
    
    
        