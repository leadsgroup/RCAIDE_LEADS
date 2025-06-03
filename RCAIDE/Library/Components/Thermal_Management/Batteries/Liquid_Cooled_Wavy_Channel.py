# RCAIDE/Library/Components/Thermal_Management/Batteries/Liquid_Cooled_Wavy_Channel.py
# 
# 
# Created:  Apr 2024 S. Shekar 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core                                                          import Units
from RCAIDE.Library.Attributes.Coolants.Glycol_Water                                import Glycol_Water
from RCAIDE.Library.Components.Component                                            import Component  
from RCAIDE.Library.Attributes.Materials.Aluminum                                   import Aluminum
from RCAIDE.Library.Components                                                      import Component
from RCAIDE.Library.Methods.Thermal_Management.Batteries.Liquid_Cooled_Wavy_Channel import wavy_channel_rating_model,append_wavy_channel_conditions,append_wavy_channel_segment_conditions 
from RCAIDE.Library.Plots.Thermal_Management                                        import plot_wavy_channel_conditions
# ----------------------------------------------------------------------------------------------------------------------
# Liquid_Cooled_Wavy_Channel_Heat_Acquisition_System
# ----------------------------------------------------------------------------------------------------------------------
class Liquid_Cooled_Wavy_Channel(Component):
    """
    Liquid-cooled wavy channel heat acquisition system for battery thermal management.

    Attributes
    ----------
    tag : str
        Identifier for the heat acquisition system. default is 'wavy_channel_heat_acquisition'
    heat_transfer_efficiency : float
        Efficiency factor for heat transfer from battery to coolant. default is 1
    coolant : Coolant
        Coolant fluid properties and characteristics. Default is Glycol Water
    coolant_Reynolds_number : float
        Reynolds number for coolant flow characterization. Default is 1
    coolant_velocity : float
        Velocity of coolant flow through channels. Default is 1 m/s
    coolant_flow_rate : float
        Volumetric flow rate of coolant. Default is 1 m³/s
    coolant_inlet_temperature : float
        Temperature of coolant entering the system. Default is 293.15 K
    coolant_hydraulic_diameter : float
        Hydraulic diameter of coolant channels. Default is 1 m
    channel_side_thickness : float
        Thickness of channel walls through which conduction occurs. Default is 0.001 m
    channel_top_thickness : float
        Thickness of channel top surface where no conduction occurs. Default is 0.001 m
    channel_width : float
        Width of individual cooling channels. Default is 0.005 m
    channel_height : float
        Height of individual cooling channels. Default is 0.003 m
    channel_contact_angle : float
        Contact arc angle between channel and battery surface. Default is 47.5 degrees
    channel : Material
        Material properties of the channel structure. Default is Aluminum
    channel_aspect_ratio : float
        Ratio of channel width to height. Default is 1
    channels_per_module : int
        Number of cooling channels per battery module. Default is 1
    battery_contact_area : float
        Total contact area between channels and battery surface in m². Default is 1
    contact_area_per_module : float
        Contact area per individual battery module in m². Default is 1
    power_draw : float
        Electrical power consumption of the cooling system in Watts. Default is 1
    single_side_contact : bool
        Flag indicating if cooling occurs on one side of battery only. Default is True
    design_heat_removed : float
        Design heat removal capacity in Watts. Default is None
    percent_operation : float
        Operating percentage of full cooling capacity (0.0 to 1.0). Default is 1
    type : str
        Type identifier for the cooling system. Default is 'Liquid'

    Notes
    -----
    The Liquid_Cooled_Wavy_Channel system provides active thermal management
    for battery packs through forced convection cooling. The wavy channel
    geometry enhances heat transfer through increased surface area and
    improved mixing of the coolant flow.

    Default coolant is glycol-water mixture unless specified otherwise.
    Channel geometry parameters are initialized with nominal values that
    should be optimized for specific battery configurations.

    The system operates by circulating coolant through channels in thermal
    contact with battery modules, removing heat generated during charge/
    discharge cycles to maintain optimal operating temperatures.

    **Major Assumptions**
        * Wavy channel heat acquisition loops through entire battery pack
        * Uniform heat generation across battery modules
        * Steady-state thermal analysis for each time step
        * Single-phase coolant flow (no boiling)

    **Definitions**

    'Hydraulic Diameter'
        Characteristic length scale for non-circular ducts, equal to 4 times
        the cross-sectional area divided by the wetted perimeter

    'Reynolds Number'
        Dimensionless parameter characterizing flow regime (laminar vs turbulent)

    See Also
    --------
    RCAIDE.Library.Components.Thermal_Management.Batteries.Air_Cooled
        Air-cooled heat acquisition system
    """
    
    def __defaults__(self):  
        """This sets the default values.
        
        Assumptions:
           The wavy channel heat Acquisition loops through the battery pack.
           The coolant is assumed to be Glycol Water unless specified otherwise. 
           The geometry parameters are set based on nominal values to be further optmized.
           
        Source:
           None
        
        """            
         
        self.tag                           = 'wavy_channel_heat_acquisition' 
        self.heat_transfer_efficiency      = 1
        self.coolant                       = Glycol_Water()
        self.coolant_Reynolds_number       = 1.
        self.coolant_velocity              = 1.
        self.coolant_flow_rate             = 1
        self.coolant_inlet_temperature     = None
        self.coolant_hydraulic_diameter    = 1.
        self.channel_side_thickness        = 0.001                  # Thickness of the Chanel through which condcution occurs 
        self.channel_top_thickness         = 0.001                  # Thickness of Channel on the top where no conduction occurs
        self.channel_width                 = 0.005                  # width of the channel 
        self.channel_height                = 0.003                  # height of the channel 
        self.channel_contact_angle         = 47.5 * Units.degrees   # Contact Arc angle in degrees    
        self.channel                       = Aluminum()
        self.channel_aspect_ratio          = 1. 
        self.channels_per_module           = 1
        self.battery_contact_area          = 1.
        self.contact_area_per_module       = 1.  
        self.power_draw                    = 1. 
        self.single_side_contact           = True 
        self.design_heat_removed           = None   
        self.percent_operation             = 1.0
        self.type                          = 'Liquid'
        
        return
    
    def __init__ (self, distributor=None):
        
        """This creates Reservoir and Heat Exchanger containers if it does not exist on
            the coolant line as a liquid cooled system requires these and cannot operate without.
    
        Assumptions:
            None
        
        Source:
            None
        """                
        
    def append_operating_conditions(self,segment,coolant_line):
        append_wavy_channel_conditions(self,segment,coolant_line)
        return
    
    def append_segment_conditions(self, segment,coolant_line):
        append_wavy_channel_segment_conditions(self, segment,coolant_line)
        return
    
    def compute_thermal_performance(self,battery,bus,coolant_line,Q_heat_gen,T_cell,state,delta_t,t_idx):
        T_battery_current =  wavy_channel_rating_model(self, battery,bus,coolant_line, Q_heat_gen, T_cell, state, delta_t, t_idx)
        return  T_battery_current
    
    def plot_operating_conditions(self, results, coolant_line,save_filename, save_figure,show_legend,file_type , width, height):
        plot_wavy_channel_conditions(self, results, coolant_line,save_filename,save_figure,show_legend,file_type , width, height)
        return