# RCAIDE/Library/Missions/Common/Update/atmosphere.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Update Atmosphere
# ----------------------------------------------------------------------------------------------------------------------
def atmosphere(segment):
    """ Computes conditions of the atmosphere at given altitudes
    
        Assumptions:
        N/A
        
        Inputs:
            state.conditions:
                freestream.altitude             [meters]
            segment.analyses.atmoshere          [Function]
            
        Outputs:
            state.conditions:
                freestream.pressure             [pascals]
                freestream.temperature          [kelvin]
                freestream.density              [kilogram/meter^3]
                freestream.speed_of_sound       [meter/second]
                freestream.dynamic_viscosity    [pascals-seconds]
                freestream.kinematic_viscosity  [meters^2/second]
                freestream.thermal_conductivity [Watt/meter-Kelvin]
                freestream.prandtl_number       [unitless]
                
        Properties Used:
        N/A
                                
    """
    
    # unpack
    conditions            = segment.state.conditions
    h                     = conditions.freestream.altitude
    temperature_deviation = segment.temperature_deviation
    atmosphere            = segment.analyses.atmosphere
    
    # compute
    atmosphere_data = atmosphere.compute_values(h,temperature_deviation)
    
    # pack
    conditions.freestream.pressure                        = atmosphere_data.pressure
    conditions.freestream.temperature                     = atmosphere_data.temperature
    conditions.freestream.thermal_conductivity            = atmosphere_data.thermal_conductivity
    conditions.freestream.density                         = atmosphere_data.density
    conditions.freestream.speed_of_sound                  = atmosphere_data.speed_of_sound
    conditions.freestream.dynamic_viscosity               = atmosphere_data.dynamic_viscosity
    conditions.freestream.kinematic_viscosity             = atmosphere_data.kinematic_viscosity
    conditions.freestream.prandtl_number                  = atmosphere_data.prandtl_number
    conditions.freestream.constant_pressure_specific_heat = atmosphere_data.constant_pressure_specific_heat
    conditions.freestream.specific_heat                   = atmosphere_data.specific_heat
        
    return
     