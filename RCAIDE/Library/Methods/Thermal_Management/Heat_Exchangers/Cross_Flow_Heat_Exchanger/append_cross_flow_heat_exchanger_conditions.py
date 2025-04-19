# RCAIDE/Library/Methods/Thermal_Management/Heat_Exchangers/Cross_Flow_Heat_Exchanger/append_cross_flow_heat_exchanger_conditions.py
# 
# Created:  Aug 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions


# ---------------------------------------------------------------------------------------------------------------------- 
#  append_cross_flow_heat_exchanger_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_cross_flow_heat_exchanger_conditions(cross_flow_hex,segment,coolant_line):
    """ Packs the initial cross flow heat exchanger conditions
    
        Assumptions:
        None
    
        Source:
        N/A
    
        Inputs:  
             cross_flow_hex          (data structure)              [None]
           
        Outputs:
            segment
               coolant_mass_flow_rate                              [kg/s]
               power                                               [watts]
               inlet_air_temperature                               [Kelvin]
               outlet_coolant_temperature                          [Kelvin]
               air_mass_flow_rate                                  [kg/s]
               air_inlet_pressure                                  [Pascal]
               coolant_inlet_pressure                              [Pascal]
               pressure_diff_air                                   [Pascal]
               effectiveness_HEX                                   [unitless]
               
        Properties Used:
        None
    """        
    
    atmosphere    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    alt           = -segment.conditions.frames.inertial.position_vector[:,2] 
    if segment.temperature_deviation != None:
        temp_dev = segment.temperature_deviation    
    atmo_data    = atmosphere.compute_values(altitude = alt,temperature_deviation=temp_dev)
    
    ones_row                                                                                         = segment.state.ones_row
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag]                            = Conditions()
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].coolant_mass_flow_rate     = 0 * ones_row(1)  
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].power                      = 0 * ones_row(1)  
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].inlet_air_temperature      = atmo_data.temperature[0,0]* ones_row(1) 
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].outlet_coolant_temperature = atmo_data.temperature[0,0]* ones_row(1) 
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].air_mass_flow_rate         = 0 * ones_row(1) 
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].air_inlet_pressure         = 0 * ones_row(1) 
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].coolant_inlet_pressure     = 0 * ones_row(1) 
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].pressure_diff_air          = 0 * ones_row(1)
    segment.state.conditions.energy[coolant_line.tag][cross_flow_hex.tag].effectiveness_HEX          = 0 * ones_row(1)
    
    return

def append_cross_flow_hex_segment_conditions(cross_flow_hex,segment,bus,coolant_line,conditions):
    """Sets the initial cross flow heat exchanger properties at the start of each segment as the last point from the previous segment 
    
        Assumptions:
        None
    
        Source:
        N/A
    
        Inputs:  
         cross_flow_hex            (data structure)              [None]
               
        Outputs:
        None
    
        Properties Used:
        None
    """    

    cross_flow_hex_conditions = conditions[coolant_line.tag][cross_flow_hex.tag]
    if segment.state.initials:  
        cross_flow_hex_initials                                   = segment.state.initials.conditions.energy[coolant_line.tag][cross_flow_hex.tag]
        cross_flow_hex_conditions.coolant_mass_flow_rate[:,0]     = cross_flow_hex_initials.coolant_mass_flow_rate[-1,0]     
        cross_flow_hex_conditions.power[:,0]                      = cross_flow_hex_initials.power[-1,0]     
        cross_flow_hex_conditions.air_inlet_pressure[:,0]         = cross_flow_hex_initials.air_inlet_pressure[-1,0]       
        cross_flow_hex_conditions.coolant_inlet_pressure[:,0]     = cross_flow_hex_initials.coolant_inlet_pressure[-1,0] 
        cross_flow_hex_conditions.inlet_air_temperature[:,0]      = cross_flow_hex_initials.inlet_air_temperature[-1,0]   
        cross_flow_hex_conditions.pressure_diff_air[:,0]          = cross_flow_hex_initials.pressure_diff_air[-1,0]                     
        cross_flow_hex_conditions.air_mass_flow_rate[:,0]         = cross_flow_hex_initials.air_mass_flow_rate[-1,0]     
        cross_flow_hex_conditions.outlet_coolant_temperature[:,0] = cross_flow_hex_initials.outlet_coolant_temperature[-1,0]  
        cross_flow_hex_conditions.effectiveness_HEX[:,0]          = cross_flow_hex_initials.effectiveness_HEX[-1,0]
        
        return
    