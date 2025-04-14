# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/size_module_from_mass.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ---------------------------------------------------------------------------------------------------------------------- -Common
def size_module_from_mass(battery_module):
    """
    Calculates the maximum energy and power of a battery module based on its mass.
    
    Parameters
    ----------
    battery_module : BatteryModule
        The battery module with the following attributes:
            - mass_properties.mass : float
                Total mass of the battery module [kg]
            - BMS_additional_weight_factor : float
                Factor accounting for battery management system weight
            - cell.mass : float or None
                Mass of a single cell [kg]
            - cell.specific_energy : float
                Specific energy of the cell [J/kg]
            - cell.specific_power : float
                Specific power of the cell [W/kg]
            - maximum_voltage : float
                Maximum voltage of the battery module [V]
            - cell.maximum_voltage : float
                Maximum voltage of a single cell [V]
    
    Returns
    -------
    None
        This function modifies the battery_module object in-place, setting the following attributes:
            - maximum_energy : float
                Maximum energy capacity of the battery module [J]
            - maximum_power : float
                Maximum power output of the battery module [W]
            - initial_maximum_energy : float
                Initial maximum energy capacity (same as maximum_energy) [J]
            - electrical_configuration.series : int
                Number of cells in series
            - electrical_configuration.parallel : int
                Number of cells in parallel
    
    Notes
    -----
    This function calculates the energy and power capabilities of a battery module based on
    its mass and the specific energy/power characteristics of its cells. It also determines
    the electrical configuration (series/parallel arrangement) of the cells.
    
    The function first calculates the effective mass of the cells by removing the mass
    contribution of the battery management system (BMS). It then uses this mass along with
    the specific energy and power values to determine the module's capabilities.
    
    If the cell mass is provided, the function calculates the number of cells that can fit
    within the module mass and arranges them in a series-parallel configuration based on
    the voltage requirements. If cell mass is not provided, a default configuration of
    1 series × 1 parallel is used.
    
    **Major Assumptions**
        * Constant specific energy and power values
        * BMS weight is a fixed fraction of the total module weight
        * All cells have identical characteristics
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.compute_module_properties
    """
    mass = battery_module.mass_properties.mass/battery_module.BMS_additional_weight_factor
    
    if battery_module.cell.mass == None: 
        n_series   = 1
        n_parallel = 1 
    else:
        n_cells    = int(mass/battery_module.cell.mass)
        n_series   = int(battery_module.maximum_voltage/battery_module.cell.maximum_voltage)
        n_parallel = int(n_cells/n_series)
        
    battery_module.maximum_energy                    = mass*battery_module.cell.specific_energy  
    battery_module.maximum_power                     = mass*battery_module.cell.specific_power
    battery_module.initial_maximum_energy            = battery_module.maximum_energy    
    battery_module.electrical_configuration.series   = n_series
    battery_module.electrical_configuration.parallel = n_parallel     
