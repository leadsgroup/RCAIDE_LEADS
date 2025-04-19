# RCAIDE/Library/Components/Propulsors/Converters/PMSM_Motor.py
# 
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.append_motor_conditions import  append_motor_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  PMSM_Motor  
# ----------------------------------------------------------------------------------------------------------------------           
class PMSM_Motor(Converter):
    """
    Permanent Magnet Synchronous Motor (PMSM) Component Class

    This class models a PMSM motor for electric propulsion systems. It inherits from the base Converter class
    and implements motor-specific attributes and methods.

    Attributes
    ----------
    tag : str
        Identifier for the motor component, defaults to 'PMSM_motor'

    speed_constant : float
        Motor speed constant [rpm/V]
    stator_inner_diameter : float
        Inner diameter of the stator [m]
    stator_outer_diameter : float
        Outer diameter of the stator [m]
    winding_factor : float
        Winding factor of the motor [-]
    resistance : float
        Motor winding resistance [Ω]
    motor_stack_length : float
        Length of the motor stack [m]
    number_of_turns : int
        Number of winding turns [-]
    length_of_path : float
        Length of the magnetic path [m]
    mu_0 : float
        Permeability of free space [N/A^2]
    mu_r : float
        Relative permeability of magnetic material [N/A^2]
    thermal_conductivity : float
        Thermal conductivity of magnetic material [W/m*K]
    Delta_T : float
        Temperature difference between inner and outer stator surfaces [K]
    characteristic_length_of_flow : float
        Characteristic length for thermal calculations [m]
    thermal_conductivity_fluid : float
        Thermal conductivity of cooling fluid [W/m*K]
    length_of_conductive_path : float
        Length of thermal conductive path [m]
    Re_cooling_flow : float
        Reynolds number of cooling flow [-]
    Re_airgap : float
        Reynolds number of airgap flow [-]
    Prandtl_number : float
        Prandtl number of fluid [-]
    height_of_duct : float
        Height of cooling duct [m]
    width_of_duct : float
        Width of cooling duct [m]
    hydraulic_diameter_of_duct : float
        Hydraulic diameter of cooling duct [m]
    length_of_channel : float
        Length of cooling channel [m]
    volume_flow_rate_of_fluid : float
        Volume flow rate of cooling fluid [m^3/s]
    density_of_fluid : float
        Density of cooling fluid [kg/m^3]
    velocity_of_fluid : float
        Velocity of cooling fluid [m/s]
    Taylor_number : float
        Taylor number for flow calculations [-]
    axial_gap_to_radius_of_rotor : float
        Ratio of axial gap to rotor radius [-]
    Conduction_laminar_flow : bool
        Flag for laminar conduction flow
    Convection_laminar_flow : bool
        Flag for laminar convection flow

    Notes
    -----
    The PMSM motor model includes detailed thermal and electromagnetic parameters for accurate 
    performance modeling. Default values are provided for all parameters but should be updated
    based on specific motor designs.

    **Major Assumptions**
        * Linear magnetic properties
        * Uniform temperature distribution in components
        * Simplified thermal model
        * No saturation effects considered

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters
    """      
    def __defaults__(self):
        """
        """           
        self.tag                       = 'PMSM_motor' 
        # Input data from Datasheet
        self.speed_constant            = 6.56                        # [rpm/V]        speed constant
        self.stator_inner_diameter     = 0.16                        # [m]            stator inner diameter
        self.stator_outer_diameter     = 0.348                       # [m]            stator outer diameter

        # Input data from Literature
        self.winding_factor            = 0.95                        # [-]            winding factor

        # Input data from Assumptions
        self.resistance                = 0.002                       # [Ω]            resistance
        self.motor_stack_length        = 11.40                       # [m]            (It should be around 0.14 m) motor stack length 
        self.number_of_turns           = 80                          # [-]            number of turns  
        self.length_of_path            = 0.4                         # [m]            length of the path  
        self.mu_0                      = 1.256637061e-6              # [N/A**2]       permeability of free space
        self.mu_r                      = 1005                        # [N/A**2]       relative permeability of the magnetic material 
        self.thermal_conductivity      = 200                         # [W/m*K]        thermal conductivity of the magnetic material
        self.Delta_T                   = 10                          # [K]            temperature difference between the inner and outer surfaces of the stator
        self.characteristic_length_of_flow = 0.01                    # [m]            characteristic length of the flow
        self.thermal_conductivity_fluid = 0.026                      # [W/m*K]        thermal conductivity of the fluid
        self.length_of_conductive_path = 0.4                         # [m]            length of the conductive path  
        self.Re_cooling_flow           = 100000                      # [-]            Reynolds number of the coolingflow
        self.Re_airgap                 = 100000                      # [-]            Reynolds number of the flow in the airgap
        self.Prandtl_number            = 0.708                       # [-]            Prandtl number of the flow
        self.height_of_duct            = 0.005                       # [m]            height of the duct
        self.width_of_duct             = 0.005                       # [m]            width of the duct
        self.hydraulic_diameter_of_duct = 0.005                      # [m]            hydraulic diameter of the duct
        self.length_of_channel         = 0.005                       # [m]            length of the channel
        self.volume_flow_rate_of_fluid = 0.005                       # [m**3/s]       volume flow rate of the fluid
        self.density_of_fluid          = 1000                        # [kg/m**3]      density of the fluid
        self.velocity_of_fluid         = 0.005                       # [m/s]          velocity of the fluid
        self.Taylor_number             = 20                          # [-]            Taylor number 
        self.axial_gap_to_radius_of_rotor = 0.01                     # [-]            ratio of the axial gap to the radius of the rotor
        self.Conduction_laminar_flow   = True                        # [-]            True if the flow is laminar, False if the flow is turbulent
        self.Convection_laminar_flow   = True                        # [-]            True if the flow is laminar, False if the flow is turbulent
        
    def append_operating_conditions(self,segment,propulsor):
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_motor_conditions(self,segment,propulsor_conditions)
        return
    