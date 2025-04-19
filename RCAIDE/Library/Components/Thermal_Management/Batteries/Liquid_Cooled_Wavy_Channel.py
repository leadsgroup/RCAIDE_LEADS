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
    '''
       Wavy Channel Heat Acqusition System
    '''
    
    def __defaults__(self):  
        """This sets the default values.
        
        Assumptions:
           The wavy channel heat Acquisition loops through the battery pack.
           The coolant is assumed to be Glycol Water unless specified otherwise. 
           The geometry parameters are set based on nomrinal values to be further optmized.
           
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
    
    def append_segment_conditions(self, segment,coolant_line, conditions):
        append_wavy_channel_segment_conditions(self, segment,coolant_line, conditions)
        return
    
    def compute_thermal_performance(self,battery,bus,coolant_line,Q_heat_gen,T_cell,state,delta_t,t_idx):
        T_battery_current =  wavy_channel_rating_model(self, battery,bus,coolant_line, Q_heat_gen, T_cell, state, delta_t, t_idx)
        return  T_battery_current
    
    def plot_operating_conditions(self, results, coolant_line,save_filename, save_figure,show_legend,file_type , width, height):
        plot_wavy_channel_conditions(self, results, coolant_line,save_filename,save_figure,show_legend,file_type , width, height)
        return