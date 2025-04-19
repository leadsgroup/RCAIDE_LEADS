# RCAIDE/Library/Methods/Thermal_Management/Batteries/Air_Cooled/append_air_cooling_conditions.py
# 
# Created:  Aug 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_wavy_channel_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_wavy_channel_conditions(wavy_channel,segment,coolant_line):
     """ Packs the initial wavy channel heat acquisition conditions

         Assumptions:
         None

         Source:
         N/A

         Inputs:  
              wavy_channel          (data structure)              [None]

         Outputs:
             segment
               heat_removed                                       [watts]           
               outlet_coolant_temperature                         [Kelvin]
               coolant_mass_flow_rate                             [kg/s]
               effectiveness                                      [None]
               power                                              [watts]    
               
         Properties Used:
         None
     """             
     
     atmosphere    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
     alt           = -segment.conditions.frames.inertial.position_vector[:,2] 
     if segment.temperature_deviation != None:
          temp_dev = segment.temperature_deviation    
     atmo_data    = atmosphere.compute_values(altitude = alt,temperature_deviation=temp_dev)
     
     
     ones_row                                                                                        = segment.state.ones_row
     segment.state.conditions.energy[coolant_line.tag][wavy_channel.tag]                            = Conditions()
     segment.state.conditions.energy[coolant_line.tag][wavy_channel.tag].heat_removed               = 0 * ones_row(1) 
     segment.state.conditions.energy[coolant_line.tag][wavy_channel.tag].outlet_coolant_temperature = atmo_data.temperature[0,0]  * ones_row(1)     
     segment.state.conditions.energy[coolant_line.tag][wavy_channel.tag].coolant_mass_flow_rate     = 0 * ones_row(1)  
     segment.state.conditions.energy[coolant_line.tag][wavy_channel.tag].effectiveness              = 0 * ones_row(1)
     segment.state.conditions.energy[coolant_line.tag][wavy_channel.tag].power                      = 0 * ones_row(1)
     
     return

def append_wavy_channel_segment_conditions(wavy_channel,segment,coolant_line,conditions):
     """Sets the initial wavy channel heat acquisition properties at the
        start of each segment as the last point from the previous segment 

         Assumptions:
         None

         Source:
         N/A

         Inputs:  
          wavy_channel          (data structure)              [None]

         Outputs:
         None

         Properties Used:
         None
     """     

     wavy_channel_conditions = conditions[coolant_line.tag][wavy_channel.tag]
     if segment.state.initials:  
          wavy_channel_initials                                    = segment.state.initials.conditions.energy[coolant_line.tag][wavy_channel.tag]
          wavy_channel_conditions.outlet_coolant_temperature[:,0] = wavy_channel_initials.outlet_coolant_temperature[-1,0]
          wavy_channel_conditions.power[:,0]                      = wavy_channel_initials.power[-1,0] 
          wavy_channel_conditions.coolant_mass_flow_rate[:,0]     = wavy_channel_initials.coolant_mass_flow_rate[-1,0]     
     
     return