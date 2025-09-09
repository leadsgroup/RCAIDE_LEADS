# RCAIDE/Library/Components/Propulsors/Converters/Motor.py
# 
# 
# Created:  Mar 2024, M. Clarke 
# Modified: May 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Converter  import Converter
from RCAIDE.Framework.Core                  import Data 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.append_motor_conditions import  append_motor_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  DC_Motor  
# ----------------------------------------------------------------------------------------------------------------------           
class DC_Motor(Converter):
    """
    A direct current electric motor component model for electric propulsion systems.

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
          
    power_split_ratio : float
        Ratio of power distribution when motor drives multiple loads. Default is 0.0.
        
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
        self.tag                     = 'motor' 
        self.motor_type              = 'DC'

        if self.motor_type == 'DC':
            self.resistance              = 0.0
            self.no_load_current         = 0.0
            self.speed_constant          = 0.0
            self.efficiency              = 1.0
            self.gearbox                 = Data()
            self.gearbox.gear_ratio      = 1.0 
            self.design_angular_velocity = 0.0 
            self.design_torque           = 0.0 
            self.design_current          = 0.0
            self.inverse_calculation     = False
            self.interpolated_func       = None

        elif self.motor_type == 'PMSM':
            # Input data from Datasheet      
            self.speed_constant                = 6.56                        # [rpm/V]        speed constant
            self.stator_inner_diameter         = 0.16                        # [m]            stator inner diameter
            self.stator_outer_diameter         = 0.348                       # [m]            stator outer diameter
            self.gearbox                       = Data()
            self.gearbox.gear_ratio            = 1.0 

            # Input data from Literature      
            self.winding_factor                = 0.95                        # [-]            winding factor

            # Input data from Assumptions
            self.resistance                    = 0.002                       # [Ω]            resistance
            self.motor_stack_length            = 0.1140                      # [m]            (It should be around 0.14 m) motor stack length 
            self.number_of_turns               = 80                          # [-]            number of turns  
            self.length_of_path                = 0.4                         # [m]            length of the path  
            self.mu_0                          = 1.256637061e-6              # [N/A**2]       permeability of free space
            self.mu_r                          = 1005                        # [N/A**2]       relative permeability of the magnetic material 
            self.thermal_conductivity          = 200                         # [W/m*K]        thermal conductivity of the magnetic material
            self.Delta_T                       = 10                          # [K]            temperature difference between the inner and outer surfaces of the stator
            self.characteristic_length_of_flow = 0.01                    # [m]            characteristic length of the flow
            self.thermal_conductivity_fluid    = 0.026                      # [W/m*K]        thermal conductivity of the fluid
            self.length_of_conductive_path     = 0.4                         # [m]            length of the conductive path  
            self.Re_cooling_flow               = 100000                      # [-]            Reynolds number of the coolingflow
            self.Re_airgap                     = 100000                      # [-]            Reynolds number of the flow in the airgap
            self.Prandtl_number                = 0.708                       # [-]            Prandtl number of the flow
            self.height_of_duct                = 0.005                       # [m]            height of the duct
            self.width_of_duct                 = 0.005                       # [m]            width of the duct
            self.hydraulic_diameter_of_duct    = 0.005                      # [m]            hydraulic diameter of the duct
            self.length_of_channel             = 0.005                       # [m]            length of the channel
            self.volume_flow_rate_of_fluid     = 0.005                       # [m**3/s]       volume flow rate of the fluid
            self.density_of_fluid              = 1000                        # [kg/m**3]      density of the fluid
            self.velocity_of_fluid             = 0.005                       # [m/s]          velocity of the fluid
            self.Taylor_number                 = 20                          # [-]            Taylor number 
            self.axial_gap_to_radius_of_rotor  = 0.01                     # [-]            ratio of the axial gap to the radius of the rotor 
            self.inverse_calculation           = False
            self.Conduction_laminar_flow       = True                        # [-]            True if the flow is laminar, False if the flow is turbulent
            self.Convection_laminar_flow       = True                        # [-]            True if the flow is laminar, False if the flow is turbulent
        
    def append_operating_conditions(self,segment,energy_conditions,noise_conditions=None): 
        append_motor_conditions(self,segment,energy_conditions)
        return
    