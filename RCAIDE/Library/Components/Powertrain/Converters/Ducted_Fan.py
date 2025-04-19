# RCAIDE/Compoments/Propulsors/Converters/Ducted_Fan.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core              import Data
from .Converter                         import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.append_ducted_fan_conditions import  append_ducted_fan_conditions
import numpy as np
import scipy as sp
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Nacalle
# ----------------------------------------------------------------------------------------------------------------------  
class Ducted_Fan(Converter):
    """
    Ducted Fan Component Class

    This class models a ducted fan propulsion system with both rotor and stator components. 
    It inherits from the base Converter class and implements ducted-fan specific attributes and methods.

    Attributes
    ----------
    tag : str
        Identifier for the ducted fan component, defaults to 'ducted_fan'
    number_of_radial_stations : int
        Number of radial stations for blade element analysis, defaults to 20
    number_of_rotor_blades : int
        Number of rotor blades, defaults to 12
    tip_radius : float
        Outer radius of the rotor [m], defaults to 1.0
    hub_radius : float
        Inner radius of the rotor at hub [m], defaults to 0.1
    exit_radius : float
        Radius at the duct exit [m], defaults to 1.1
    blade_clearance : float
        Clearance between blade tip and duct wall [m], defaults to 0.01
    length : float
        Axial length of the ducted fan [m], defaults to 1.0
    fan_effectiveness : float
        Fan effectiveness factor [-], defaults to 1.1
    Cp_polynomial_coefficients : list
        Coefficients for power coefficient polynomial [-, -, -]
    Ct_polynomial_coefficients : list
        Coefficients for thrust coefficient polynomial [-, -, -]
    etap_polynomial_coefficients : list
        Coefficients for propulsive efficiency polynomial [-, -, -]
    fidelity : str
        Analysis fidelity level, either 'Blade_Element_Momentum_Theory' or 'Rankine_Froude_Momentum_Theory'
    orientation_euler_angles : list
        Default orientation angles of rotor [rad, rad, rad]
    rotor : Data
        Container for rotor geometry and performance data
    stator : Data
        Container for stator geometry and performance data
    cruise : Data
        Container for cruise design conditions

    Notes
    -----
    The ducted fan model includes detailed geometric parameters and performance characteristics
    for both the rotor and stator components. The model supports multiple fidelity levels
    and includes coordinate transformation capabilities for thrust vectoring analysis.

    **Major Assumptions**
        * Axisymmetric flow
        * Steady state operation
        * Incompressible flow for low-fidelity analysis
        * No radial flow
        * Uniform inflow
        * No blade-to-blade interaction

    """ 
    
    def __defaults__(self):
        """ This sets the default values for the component to function.
        
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
        
        self.tag                                   = 'ducted_fan'  
        self.number_of_radial_stations             = 20
        self.number_of_rotor_blades                = 12  
        self.tip_radius                            = 1.0
        self.hub_radius                            = 0.1
        self.exit_radius                           = 1.1
        self.blade_clearance                       = 0.01
        self.length                                = 1 
        self.fan_effectiveness                     = 1.1 
        self.Cp_polynomial_coefficients            = [0.551,  0.0182, -0.0869]   
        self.Ct_polynomial_coefficients            = [0.4605,-0.0529, -0.1203]   
        self.etap_polynomial_coefficients          = [0.0653,4.1603 , -7.6128]  
        self.fidelity                              = 'Blade_Element_Momentum_Theory' # 'Rankine_Froude_Momentum_Theory'  
        self.orientation_euler_angles              = [0.,0.,0.]  # vector of angles defining default orientation of rotor
        self.rotor                                 = Data()
        self.stator                                = Data()
        self.rotor.percent_x_location              = 0.4
        self.stator.percent_x_location             = 0.7
        self.cruise                                = Data() 
        self.cruise.design_thrust                  = None
        self.cruise.design_power                   = None
        self.cruise.design_altitude                = None
        self.cruise.design_efficiency              = None  
        self.cruise.design_angular_velocity        = None 
        self.cruise.design_freestream_velocity     = None
        self.cruise.design_reference_velocity      = None 
        self.cruise.design_freestream_mach         = None  
        self.duct_airfoil                          = None
        self.hub_airfoil                           = None
      
    
    def append_duct_airfoil(self, airfoil):
        """
        Adds an airfoil to the ducted fan's duct section.

        Parameters
        ----------
        airfoil : Data
            Airfoil data container with aerodynamic properties for the duct section.
            Must be of type Data().

        Returns
        -------
        None

        Notes
        -----
        This method appends airfoil data to the duct_airfoil attribute of the ducted fan.
        The airfoil data is used to model the aerodynamic characteristics of the duct
        section, which influences the overall performance of the ducted fan system.

        Raises
        ------
        Exception
            If input airfoil is not of type Data()
        """

        # Assert database type
        if not isinstance(airfoil,RCAIDE.Library.Components.Airfoils.Airfoil):
            raise Exception('input component must be of type Airfoil') 

        # Store data
        self.duct_airfoil =  airfoil

        return
    

    def append_hub_airfoil(self, airfoil):
        """
        Adds an airfoil to the ducted fan's hub section.

        Parameters
        ----------
        airfoil : Data
            Airfoil data container with aerodynamic properties for the hub section.
            Must be of type Data().

        Returns
        -------
        None

        Notes
        -----
        This method appends airfoil data to the hub_airfoil attribute of the ducted fan.
        The airfoil data is used to model the aerodynamic characteristics of the hub
        section, which affects the flow field and performance of the ducted fan system.

        Raises
        ------
        Exception
            If input airfoil is not of type Data()
        """

        # Assert database type
        if not isinstance(airfoil,RCAIDE.Library.Components.Airfoils.Airfoil):
            raise Exception('input component must be of type Airfoil') 

        # Store data
        self.hub_airfoil =  airfoil

        return 

    def append_operating_conditions(ducted_fan,segment,propulsor): 
        energy_conditions       = segment.state.conditions.energy[propulsor.tag]
        append_ducted_fan_conditions(ducted_fan,segment,energy_conditions)
        return        
          
    def vec_to_vel(self):
        """
        Rotates from the ducted fan's vehicle frame to the ducted fan's velocity frame.

        Parameters
        ----------
        None

        Returns
        -------
        rot_mat : ndarray
            3x3 rotation matrix transforming from vehicle frame to velocity frame.

        Notes
        -----
        This method creates a rotation matrix for transforming coordinates between
        two reference frames of the ducted fan. When the ducted fan is axially
        aligned with the vehicle body:

        Velocity frame:
        * X-axis points out the nose
        * Z-axis points towards the ground
        * Y-axis points out the right wing

        Vehicle frame:
        * X-axis points towards the tail
        * Z-axis points towards the ceiling
        * Y-axis points out the right wing

        **Theory**
        The transformation is accomplished using a rotation of π radians about the Y-axis,
        represented as a rotation vector [0, π, 0].

        **Major Assumptions**
        * The ducted fan's default orientation is aligned with the vehicle body
        * Right-handed coordinate system is used
        """

        rot_mat = sp.spatial.transform.Rotation.from_rotvec([0,np.pi,0]).as_matrix()

        return rot_mat
    

    def body_to_prop_vel(self, commanded_thrust_vector):
        """
        Rotates from the system's body frame to the ducted fan's velocity frame.

        Parameters
        ----------
        commanded_thrust_vector : ndarray
            Vector of commanded thrust angles [rad] for each time step.

        Returns
        -------
        rot_mat : ndarray
            3x3 rotation matrix transforming from body frame to ducted fan velocity frame.
        rots : ndarray
            Array of rotation vectors including commanded thrust angles.

        Notes
        -----
        This method performs a sequence of rotations to transform coordinates from
        the vehicle body frame to the ducted fan's velocity frame. The transformation
        sequence is:
        1. Body to vehicle frame (π rotation about Y-axis)
        2. Vehicle to ducted fan vehicle frame (includes thrust vector rotation)
        3. Ducted fan vehicle to ducted fan velocity frame

        Reference frames:
        Velocity frame:
        * X-axis points out the nose
        * Z-axis points towards the ground
        * Y-axis points out the right wing

        Vehicle frame:
        * X-axis points towards the tail
        * Z-axis points towards the ceiling
        * Y-axis points out the right wing

        **Theory**
        The complete transformation is computed as:
        rot_mat = (body_2_vehicle @ vehicle_2_duct_vec) @ duct_vec_2_duct_vel

        **Major Assumptions**
        * The ducted fan's default orientation is defined by orientation_euler_angles
        * Right-handed coordinate system is used
        * Thrust vector rotation is applied about the Y-axis
        * Matrix multiplication order preserves proper transformation sequence
        """

        # Go from velocity to vehicle frame
        body_2_vehicle = sp.spatial.transform.Rotation.from_rotvec([0,np.pi,0]).as_matrix()

        # Go from vehicle frame to ducted fan vehicle frame: rot 1 including the extra body rotation
        cpts       = len(np.atleast_1d(commanded_thrust_vector))
        rots       = np.array(self.orientation_euler_angles) * 1.
        rots       = np.repeat(rots[None,:], cpts, axis=0) 
        rots[:,1] += commanded_thrust_vector[:,0] 
        
        vehicle_2_duct_vec = sp.spatial.transform.Rotation.from_rotvec(rots).as_matrix()

        # GO from the ducted fan vehicle frame to the ducted fan velocity frame: rot 2
        duct_vec_2_duct_vel = self.vec_to_vel()

        # Do all the matrix multiplies
        rot1    = np.matmul(body_2_vehicle,vehicle_2_duct_vec)
        rot_mat = np.matmul(rot1,duct_vec_2_duct_vel)
 
        return rot_mat , rots


    def duct_vel_to_body(self, commanded_thrust_vector):
        """
        Rotates from the ducted fan's velocity frame to the system's body frame.

        Parameters
        ----------
        commanded_thrust_vector : ndarray
            Vector of commanded thrust angles [rad] for each time step.

        Returns
        -------
        rot_mat : ndarray
            3x3 rotation matrix transforming from ducted fan velocity frame to body frame.
        rots : ndarray
            Array of rotation vectors including commanded thrust angles.

        Notes
        -----
        This method performs the inverse transformation sequence of body_to_prop_vel.
        The transformation sequence is:
        1. Ducted fan velocity to ducted fan vehicle frame
        2. Ducted fan vehicle to vehicle frame (includes thrust vector rotation)
        3. Vehicle to body frame (π rotation about Y-axis)

        Reference frames:
        Velocity frame:
        * X-axis points out the nose
        * Z-axis points towards the ground
        * Y-axis points out the right wing

        Vehicle frame:
        * X-axis points towards the tail
        * Z-axis points towards the ceiling
        * Y-axis points out the right wing

        **Theory**
        The transformation is computed by inverting the rotation matrix from 
        body_to_prop_vel using:
        rot_mat = (body_2_duct_vel)^(-1)

        **Major Assumptions**
        * The ducted fan's default orientation is defined by orientation_euler_angles
        * Right-handed coordinate system is used
        * Thrust vector rotation is applied about the Y-axis
        """

        body2ductvel,rots = self.body_to_duct_vel(commanded_thrust_vector)

        r = sp.spatial.transform.Rotation.from_matrix(body2ductvel)
        r = r.inv()
        rot_mat = r.as_matrix()

        return rot_mat, rots
    
    def vec_to_duct_body(self,commanded_thrust_vector):
        rot_mat, rots =  self.duct_vel_to_body(commanded_thrust_vector) 
        return rot_mat, rots 