# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/append_fuel_tank_conditions.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def append_fuel_tank_conditions(tank, segment):
    """
    Appends initial conditions for fuel tank component during later mission analysis.
    
    Parameters
    ----------
    tank : FuelTank
        The fuel tank component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the fuel tank is operating.
    distributor : Fuel Line of Bus
        The fuel line or bus connected to the fuel tank.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function initializes the conditions for a fuel tank component at the start
    of a mission segment. It creates a Conditions object for the fuel tank within
    the segment's energy conditions dictionary, indexed by the fuel line tag and tank tag.
    
    The function initializes arrays for:
        - Mass flow rate [kg/s]
        - Mass [kg]
    
    These arrays are initialized with ones of the same length as the segment's state vector,
    which will be updated during mission analysis based on the fuel tank's performance
    and fuel consumption.
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks 
    """ 
    ones_row    = segment.state.ones_row
        
    segment.state.conditions.energy.sources[tank.tag]                            = Conditions()  
    segment.state.conditions.energy.sources[tank.tag].fuel_mass                  = tank.fuel.mass_properties.mass * ones_row(1)  
    segment.state.conditions.energy.sources[tank.tag].mass_flow_rate             = 0 * ones_row(1)  
    segment.state.conditions.energy.sources[tank.tag].surface_temperature        = 0 * ones_row(1)  
    segment.state.conditions.energy.sources[tank.tag].boil_off_flow_rate         = 0 * ones_row(1)  
    segment.state.conditions.energy.sources[tank.tag].ullage                     = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].secondary_mass_flow_rate   = tank.secondary_mass_flow_rate * ones_row(1) 
    segment.state.conditions.energy.sources[tank.tag].power_split_ratio          = tank.power_split_ratio * ones_row(1) 
    segment.state.conditions.energy.sources[tank.tag].inputs                     = Conditions()         
    segment.state.conditions.energy.sources[tank.tag].inputs.power               = Conditions() 
    segment.state.conditions.energy.sources[tank.tag].inputs.power.propulsive    = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].inputs.power.mechanical    = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].inputs.power.electrical    = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].inputs.power.chemical      = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].inputs.power.pneumatic     = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].inputs.power.hydraulic     = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].inputs.power.thermal       = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[tank.tag].outputs                    = Conditions()  
    segment.state.conditions.energy.sources[tank.tag].outputs.power              = Conditions() 
    segment.state.conditions.energy.sources[tank.tag].outputs.power.propulsive   = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].outputs.power.mechanical   = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].outputs.power.electrical   = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].outputs.power.chemical     = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].outputs.power.pneumatic    = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].outputs.power.hydraulic    = 0 * ones_row(1)
    segment.state.conditions.energy.sources[tank.tag].outputs.power.thermal      = 0 * ones_row(1)
    segment.state.conditions.weights.components.mass[tank.fuel.tag]               = tank.fuel.mass_properties.mass * ones_row(1) 
         
    return

def append_fuel_tank_segment_conditions(tank, segment):  
    energy_conditions  = segment.state.conditions.energy     
    energy_conditions.sources[tank.tag].inputs.power.propulsive[:,0]    = 0.0
    energy_conditions.sources[tank.tag].inputs.power.mechanical[:,0]    = 0.0 
    energy_conditions.sources[tank.tag].inputs.power.electrical[:,0]    = 0.0 
    energy_conditions.sources[tank.tag].inputs.power.chemical[:,0]      = 0.0
    energy_conditions.sources[tank.tag].inputs.power.pneumatic[:,0]     = 0.0 
    energy_conditions.sources[tank.tag].inputs.power.hydraulic[:,0]     = 0.0 
    energy_conditions.sources[tank.tag].inputs.power.thermal[:,0]       = 0.0 
    energy_conditions.sources[tank.tag].outputs.power.propulsive[:,0]   = 0.0
    energy_conditions.sources[tank.tag].outputs.power.mechanical[:,0]   = 0.0   
    energy_conditions.sources[tank.tag].outputs.power.electrical[:,0]   = 0.0 
    energy_conditions.sources[tank.tag].outputs.power.chemical[:,0]     = 0.0
    energy_conditions.sources[tank.tag].outputs.power.pneumatic[:,0]    = 0.0   
    energy_conditions.sources[tank.tag].outputs.power.hydraulic[:,0]    = 0.0   
    energy_conditions.sources[tank.tag].outputs.power.thermal[:,0]      = 0.0  
    
    return 
    
    
