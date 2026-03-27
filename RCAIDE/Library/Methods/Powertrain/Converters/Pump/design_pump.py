# RCAIDE/Library/Methods/Powertrain/Converters/Pump/design_pump.py
# 
# Created:  Jan 2026, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  RCAIDE imports 
# ----------------------------------------------------------------------------------------------------------------------  
import RCAIDE

# ---------------------------------------------------------------------------------------------------------------------- 
#  design_pump
# ----------------------------------------------------------------------------------------------------------------------    
def design_pump(pump):
    """size pump from power density
    """
    
    if isinstance(pump.working_fluid,RCAIDE.Library.Attributes.Coolants.Coolant) == False and\
       isinstance(pump.working_fluid,RCAIDE.Library.Attributes.Propellants.Propellant) == False:
        raise AssertionError('Working fluid must be either a coolant or propellant!')  
    
    # compute delta P
    pressure_rise = pump.design_outlet_pressure - pump.design_inlet_pressure

    # mass flow 
    m_dot  = pump.design_mass_flow_rate 

    # volumetric flow rate
    Q  = m_dot /pump.working_fluid.density

    # hydraulic power
    Hydraulic_Power  =  Q * pressure_rise

    # mass of pump 
    total_efficiency =  pump.pump_efficiency * pump.turbine_efficiency

    # shaft power 
    Shaft_Power = Hydraulic_Power / total_efficiency

    # mass 
    mass = (Shaft_Power / pump.power_density ) * pump.casting_and_mount_factor
    pump.mass_properties.mass = mass   

    return  