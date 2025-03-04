# RCAIDE/Library/Compoments/Powertrain/Converters/Rotor.py
# 
# 
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

 # RCAIDE imports 
from RCAIDE.Framework.Core                              import Data , Units, Container
from RCAIDE.Library.Components                          import Component 
from RCAIDE.Framework.Analyses.Propulsion               import Momentum_Theory_Wake 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.append_rotor_conditions import  append_rotor_conditions

# package imports
import numpy as np
import scipy as sp

# ---------------------------------------------------------------------------------------------------------------------- 
#  Generalized Rotor Class
# ---------------------------------------------------------------------------------------------------------------------- 
class Rotor(Component):
    """
    A generalized rotor component model serving as the base class for various rotary propulsion devices.

    Attributes
    ----------
    tag : str
        Identifier for the rotor. Default is 'rotor'.
        
    number_of_blades : float
        Number of rotor blades. Default is 0.0.
        
    tip_radius : float
        Outer radius of the rotor [m]. Default is 0.0.
        
    hub_radius : float
        Inner radius of the rotor at hub [m]. Default is 0.0.
        
    twist_distribution : float
        Blade twist angle distribution [rad]. Default is 0.0.
        
    sweep_distribution : float
        Quarter chord sweep distribution [m]. Default is 0.0.
        
    chord_distribution : float
        Blade chord length distribution [m]. Default is 0.0.
        
    thickness_to_chord : float
        Ratio of blade thickness to chord. Default is 0.0.
        
    max_thickness_distribution : float
        Maximum thickness distribution along blade [m]. Default is 0.0.
        
    radius_distribution : ndarray
        Radial stations along blade [m]. Default is None.
        
    mid_chord_alignment : float
        Mid-chord line alignment [m]. Default is 0.0.
        
    blade_solidity : float
        Ratio of total blade area to rotor disk area. Default is 0.0.
        
    flap_angle : float
        Blade flapping angle [rad]. Default is 0.0.
        
    number_azimuthal_stations : int
        Number of azimuthal calculation points. Default is 16.
        
    vtk_airfoil_points : int
        Number of points for VTK airfoil visualization. Default is 40.
        
    Airfoils : Airfoil_Container
        Container for blade airfoil definitions. Default is empty container.
        
    airfoil_polar_stations : ndarray
        Radial stations for airfoil polars. Default is None.
        
    induced_power_factor : float
        Factor accounting for non-ideal induced power. Default is 1.48.
        
    profile_drag_coefficient : float
        Mean blade profile drag coefficient. Default is 0.03.
        
    clockwise_rotation : bool
        Direction of rotation. Default is True.
        
    phase_offset_angle : float
        Initial blade phase angle [rad]. Default is 0.0.
        
    orientation_euler_angles : list
        Angles defining rotor orientation [rad]. Default is [0.,0.,0.].
        
    pitch_command : float
        Commanded blade pitch angle [rad]. Default is 0.0.
        
    ducted : bool
        Flag for ducted rotor configuration. Default is False.
        
    sol_tolerance : float
        Solution convergence tolerance. Default is 1e-8.
        
    use_2d_analysis : bool
        Flag for 2D aerodynamic analysis. Default is False.
        
    nonuniform_freestream : bool
        Flag for nonuniform inflow conditions. Default is False.

    Notes
    -----
    The Rotor class provides a comprehensive framework for modeling rotary
    propulsion devices including:

    * Geometric definition of rotor and blades
    * Aerodynamic performance calculation
    * Wake modeling capabilities
    * Blade element analysis
    * Performance optimization
    * Acoustic analysis

    **Major Assumptions**

    * Rigid blade structure
    * Quasi-steady aerodynamics
    * Small angle approximations for flapping
    * Linear blade twist and taper
    * Uniform inflow (unless nonuniform_freestream is True)

    **Theory**

    The rotor model combines:
    * Blade Element Momentum Theory
    * Prescribed/Free Wake Analysis
    * Acoustic Propagation Models
    * Performance Optimization Methods

    **Definitions**

    'Blade Solidity'
        Ratio of total blade area to rotor disk area
    'Induced Power Factor'
        Correction for non-ideal induced power effects
    'Phase Offset'
        Initial azimuthal position of reference blade

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Propeller
    RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor
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

        self.tag                               = 'rotor' 
        
        # geometry properties 
        self.number_of_blades                  = 0.0
        self.tip_radius                        = 0.0
        self.hub_radius                        = 0.0
        self.twist_distribution                = 0.0
        self.sweep_distribution                = 0.0         # quarter chord offset from quarter chord of root airfoil
        self.chord_distribution                = 0.0
        self.thickness_to_chord                = 0.0
        self.max_thickness_distribution        = 0.0
        self.radius_distribution               = None
        self.mid_chord_alignment               = 0.0
        self.blade_solidity                    = 0.0 
        self.flap_angle                        = 0.0
        self.number_azimuthal_stations         = 16 
        self.vtk_airfoil_points                = 40        
        self.airfoils                          = Airfoil_Container() 
        self.airfoil_polar_stations            = []  
        self.etap_J_coefficients               = [0, 0.109, 0.314, 0.486, 0.756, 0.983, 1.259, 1.622, 1.969, 2.258, 2.380, 2.511, 2.578, 2.620]
        self.etap_eff_coefficients             = [0, 0.231, 0.553, 0.720, 0.826, 0.853, 0.863, 0.855, 0.836, 0.782, 0.719, 0.544, 0.323, 0]
        self.Cp_J_coefficients                 = [0, 0.772, 1.589, 2.266, 2.734]        
        self.Cp_power_coefficients             = [0.389, 0.369, 0.319, 0.178, 0]
        self.Ct_J_coefficients                 = [0, 0.846, 1.537, 1.678, 2.034, 2.658]        
        self.Ct_thrust_coefficients            = [0.180, 0.175, 0.156, 0.144, 0.096, 0]
        self.fidelity                          = 'Blade_Element_Momentum_Theory_Helmholtz'  
        
        # design flight conditions 
        self.cruise                            = Data() 
        self.cruise.design_power               = None
        self.cruise.design_thrust              = None
        self.cruise.design_power_coefficient   = 0.01 
        self.cruise.design_thrust_coefficient  = 0.01

        # operating conditions 
        self.induced_power_factor              = 1.48        # accounts for interference effects
        self.profile_drag_coefficient          = .03
        self.clockwise_rotation                = True
        self.phase_offset_angle                = 0.0
        self.orientation_euler_angles          = [0.,0.,0.]  # vector of angles defining default orientation of rotor
        self.pitch_command                     = 0.0
        self.ducted                            = False
        self.sol_tolerance                     = 1e-8 
        self.use_2d_analysis                   = False       # True if rotor is at an angle relative to freestream or nonuniform freestream
        self.nonuniform_freestream             = False   
        self.axial_velocities_2d               = None        # user input for additional velocity influences at the rotor
        self.tangential_velocities_2d          = None        # user input for additional velocity influences at the rotor
        self.radial_velocities_2d              = None        # user input for additional velocity influences at the rotor 
        self.start_angle                       = 0.0         # angle of first blade from vertical
        self.start_angle_idx                   = 0           # azimuthal index at which the blade is started 
        self.variable_pitch                    = False
        self.electric_propulsion_fraction      = 1.0

        # Initialize the default wake set to Fidelity Zero 
        self.Wake                      = Momentum_Theory_Wake() 
        
        # blade optimization parameters     
        self.optimization_parameters                                    = Data() 
        self.optimization_parameters.tip_mach_range                     = [0.3,0.7] 
        self.optimization_parameters.multiobjective_aeroacoustic_weight = 1.0
        self.optimization_parameters.multiobjective_performance_weight  = 1.0
        self.optimization_parameters.multiobjective_acoustic_weight     = 1.0
        self.optimization_parameters.noise_evaluation_angle             = 135 * Units.degrees 
        self.optimization_parameters.tolerance                          = 1E-4
        self.optimization_parameters.ideal_SPL_dBA                      = 30
        self.optimization_parameters.ideal_efficiency                   = 1.0     
        self.optimization_parameters.ideal_figure_of_merit              = 1.0

    def append_operating_conditions(rotor,segment,propulsor): 
        energy_conditions       = segment.state.conditions.energy[propulsor.tag]
        noise_conditions        = segment.state.conditions.noise[propulsor.tag]
        append_rotor_conditions(rotor,segment,energy_conditions,noise_conditions)
        return        
         
    def append_airfoil(self,airfoil):
        """ Adds an airfoil to the segment

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
        # assert database type
        if not isinstance(airfoil,Data):
            raise Exception('input component must be of type Data()')


        # See if the component exists, if it does modify the name
        keys = self.keys()
        if airfoil.tag in keys:
            string_of_keys = "".join(self.keys())
            n_comps = string_of_keys.count(airfoil.tag)
            airfoil.tag = airfoil.tag + str(n_comps+1)    
            
        # store data
        self.airfoils.append(airfoil)
    
    def vec_to_vel(self):
        """
        Rotates from the rotor's vehicle frame to the rotor's velocity frame.

        Returns
        -------
        rot_mat : ndarray
            3x3 rotation matrix transforming from vehicle frame to velocity frame.

        Notes
        -----
        This method creates a rotation matrix for transforming coordinates between
        two reference frames of the rotor. When rotor is axially aligned with the 
        vehicle body.

        Velocity frame:

        * X-axis points out the nose
        * Z-axis points towards the ground
        * Y-axis points out the right wing

        Vehicle frame:

        * X-axis points towards the tail
        * Z-axis points towards the ceiling
        * Y-axis points out the right wing

        **Major Assumptions**

        * The rotor's default orientation is aligned with the vehicle body
        * Right-handed coordinate system is used
        * Small angle approximations are not used
        * Rotation sequence is fixed
        """

        rot_mat = sp.spatial.transform.Rotation.from_rotvec([0,np.pi,0]).as_matrix()

        return rot_mat
    

    def body_to_prop_vel(self, commanded_thrust_vector):
        """
        Rotates from the system's body frame to the rotor's velocity frame.

        Parameters
        ----------
        commanded_thrust_vector : ndarray
            Vector of commanded thrust angles [rad] for each time step.

        Returns
        -------
        rot_mat : ndarray
            3x3 rotation matrix transforming from body frame to rotor velocity frame.
        rots : ndarray
            Array of rotation vectors including commanded thrust angles and orientation.

        Notes
        -----
        This method performs a sequence of rotations to transform coordinates from
        the vehicle body frame to the rotor's velocity frame. The transformation
        sequence is:

        1. Body to vehicle frame (π rotation about Y-axis)
        2. Vehicle to rotor vehicle frame (includes thrust vector and orientation)
        3. Rotor vehicle to rotor velocity frame

        **Theory**
        The complete transformation is computed as:
        .. math::
            R_{total} = (R_{body2vehicle} R_{vehicle2rotor}) R_{rotor2vel}

        Where:

        * R_{body2vehicle} is a π rotation about Y-axis
        * R_{vehicle2rotor} includes orientation_euler_angles and thrust command
        * R_{rotor2vel} is from vec_to_vel()

        Velocity frame:

        * X-axis points out the nose
        * Z-axis points towards the ground
        * Y-axis points out the right wing

        Vehicle frame:

        * X-axis points towards the tail
        * Z-axis points towards the ceiling
        * Y-axis points out the right wing

        **Major Assumptions**

        * The rotor's default orientation is defined by orientation_euler_angles
        * Right-handed coordinate system is used
        * Thrust vector rotation is applied about the Y-axis
        * Matrix multiplication order preserves proper transformation sequence
        * Euler angle sequence is fixed
        """

        # Go from velocity to vehicle frame
        body_2_vehicle = sp.spatial.transform.Rotation.from_rotvec([0,np.pi,0]).as_matrix()

        # Go from vehicle frame to propeller vehicle frame: rot 1 including the extra body rotation
        cpts       = len(np.atleast_1d(commanded_thrust_vector))
        rots       = np.array(self.orientation_euler_angles) * 1.
        rots       = np.repeat(rots[None,:], cpts, axis=0) 
        rots[:,1] += commanded_thrust_vector[:,0] 
        
        vehicle_2_prop_vec = sp.spatial.transform.Rotation.from_rotvec(rots).as_matrix()

        # GO from the propeller vehicle frame to the propeller velocity frame: rot 2
        prop_vec_2_prop_vel = self.vec_to_vel()

        # Do all the matrix multiplies
        rot1    = np.matmul(body_2_vehicle,vehicle_2_prop_vec)
        rot_mat = np.matmul(rot1,prop_vec_2_prop_vel)
 
        return rot_mat , rots


    def prop_vel_to_body(self, commanded_thrust_vector):
        """
        Rotates from the rotor's velocity frame to the system's body frame.

        Parameters
        ----------
        commanded_thrust_vector : ndarray
            Vector of commanded thrust angles [rad] for each time step.

        Returns
        -------
        rot_mat : ndarray
            3x3 rotation matrix transforming from rotor velocity frame to body frame.
        rots : ndarray
            Array of rotation vectors including commanded thrust angles and orientation.

        Notes
        -----
        This method performs the inverse transformation sequence of body_to_prop_vel.
        The transformation sequence is:

        1. Rotor velocity to rotor vehicle frame
        2. Rotor vehicle to vehicle frame (includes thrust vector and orientation)
        3. Vehicle to body frame (π rotation about Y-axis)

        **Theory**
        The complete transformation is computed as:
        .. math::
            R_{total} = (R_{body2propvel})^{-1}

        Velocity frame:

        * X-axis points out the nose
        * Z-axis points towards the ground
        * Y-axis points out the right wing

        Vehicle frame:

        * X-axis points towards the tail
        * Z-axis points towards the ceiling
        * Y-axis points out the right wing

        **Major Assumptions**

        * The rotor's default orientation is defined by orientation_euler_angles
        * Right-handed coordinate system is used
        * Thrust vector rotation is applied about the Y-axis
        * Rotation matrices are orthogonal (inverse = transpose)
        * Euler angle sequence is fixed
        """

        body2propvel,rots = self.body_to_prop_vel(commanded_thrust_vector)

        r = sp.spatial.transform.Rotation.from_matrix(body2propvel)
        r = r.inv()
        rot_mat = r.as_matrix()

        return rot_mat, rots
    
    def vec_to_prop_body(self,commanded_thrust_vector):
        rot_mat, rots =  self.prop_vel_to_body(commanded_thrust_vector) 
        return rot_mat, rots

 
class Airfoil_Container(Container):
    """ Container for rotor airfoil  
    
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
        
        return []
