# RCAIDE/Library/Components/Propulsors/Converters/DC_Motor.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Converter  import Converter
from RCAIDE.Framework.Core                  import Data 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.append_motor_conditions   import  append_motor_conditions 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.compute_motor_performance import compute_motor_performance
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_cylinder_center_of_gravity  import compute_cylinder_center_of_gravity
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_cylinder_moment_of_inertia  import compute_cylinder_moment_of_inertia

# ----------------------------------------------------------------------------------------------------------------------
#  DC_Motor  
# ----------------------------------------------------------------------------------------------------------------------           
class DC_Motor(Converter):
    """
    A direct current (DC) electric motor component model for electric propulsion systems.

    Attributes
    ----------
    tag : str
        Identifier for the motor. Default is 'motor'.
        
    resistance : float
        Internal electrical resistance of the motor [Ω]. Default is 0.0.
        
    no_load_current : float
        Current drawn by the motor with no mechanical load [A]. Default is 0.0.
        
    speed_constant : float
        Motor speed constant (Kv). Default is 0.0.
        
    efficiency : float
        Overall motor efficiency. Default is 1.0.
        
    gearbox.gear_ratio : float
        Ratio of output shaft speed to motor speed. Default is 1.0.  
        
    design_torque : float
        Design point torque output [N·m]. Default is 0.0.
        
    interpolated_func : callable
        Function for interpolating motor performance. Default is None.

    Notes
    -----
    The DC_Motor class models a direct current electric motor's performance
    characteristics. It accounts for electrical, mechanical, and thermal effects
    including:
        * Internal resistance losses
        * No-load current losses
        * Gearbox losses
        * Speed-torque relationships
        * Power distribution for multiple loads

    **Definitions**

    'Kv'
        Motor velocity constant, relating voltage to unloaded motor speed

    'No-load Current'
        Current drawn by motor to overcome internal friction when unloaded
        
    'Power Split Ratio'
        Fraction of total power delivered to primary load in multi-load applications

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Motor
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
        self.tag                           = 'motor' 
        self.diameter                      = 0.0
        self.length                        = 0.0  
        self.gearbox                       = Data()
        self.gearbox.gear_ratio            = 1.0  # default unity ratio 
        self.interpolated_func             = None
        self.type                          = "DC"
        self.interpolated_func             = None
        self.reverse_mode_computation      = False
        self.design_angular_velocity       = 0.0  # [rad/s]
        self.design_torque                 = 0.0  # [N·m]
        self.design_current                = 0.0  # [A]
        self.resistance                    = 0.0  # [Ω]
        self.no_load_current               = 0.0  # [A]
        self.speed_constant                = 0.0  # [rpm/V]
        self.efficiency                    = 1.0  # [-] 
        self.speed_constant                = 6.56  # [rpm/V]
        self.stator_inner_diameter         = 0.16  # [m]
        self.stator_outer_diameter         = 0.348  # [m]
        self.winding_factor                = 0.95  # [-]
        self.resistance                    = 0.002  # [Ω]
        self.motor_stack_length            = 0.1140  # [m]
        self.number_of_turns               = 80  # [-]
        self.length_of_path                = 0.4  # [m]
        self.mu_0                          = 1.256637061e-6  # [N/A^2]
        self.mu_r                          = 1005  # [-]
        self.thermal_conductivity          = 200  # [W/m·K]
        self.Delta_T                       = 10  # [K]
        self.characteristic_length_of_flow = 0.01  # [m]
        self.thermal_conductivity_fluid    = 0.026  # [W/m·K]
        self.length_of_conductive_path     = 0.4  # [m]
        self.Re_cooling_flow               = 100000  # [-]
        self.Re_airgap                     = 100000  # [-]
        self.Prandtl_number                = 0.708  # [-]
        self.height_of_duct                = 0.005  # [m]
        self.width_of_duct                 = 0.005  # [m]
        self.hydraulic_diameter_of_duct    = 0.005  # [m]
        self.length_of_channel             = 0.005  # [m]
        self.volume_flow_rate_of_fluid     = 0.005  # [m^3/s]
        self.density_of_fluid              = 1000  # [kg/m^3]
        self.velocity_of_fluid             = 0.005  # [m/s]
        self.Taylor_number                 = 20  # [-]
        self.axial_gap_to_radius_of_rotor  = 0.01  # [-]
        self.Conduction_laminar_flow       = True  # [-]
        self.Convection_laminar_flow       = True  # [-]        
        
    def append_operating_conditions(self,segment): 
        append_motor_conditions(self,segment)
        return 

    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the motor.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]] 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.moments_of_inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """
        _ , _ = compute_cylinder_moment_of_inertia(self,outer_length=self.length,outer_radius=self.diameter/2,center_of_gravity= center_of_gravity) 
        return
    

    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for the motor.

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the center of gravity calculation
        """
        _  = compute_cylinder_center_of_gravity(self, length=self.length) 
        return

    def compute_performance(self,state):

        inputs, outputs, stored_results_flag,stored_converter_tag = compute_motor_performance(self,state)
        return inputs, outputs, stored_results_flag, stored_converter_tag
    
        