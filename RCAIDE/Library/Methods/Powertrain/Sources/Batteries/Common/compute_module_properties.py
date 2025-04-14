# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/compute_module_properties.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from RCAIDE.Framework.Core import Units 
import  numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def compute_module_properties(battery_module):  
    """
    Calculates module level properties of battery module using cell properties and module configuration.
    
    Parameters
    ----------
    battery_module : BatteryModule
        The battery module with the following attributes:
            - cell.nominal_capacity : float
                Nominal capacity of a single cell [Ah]
            - cell.maximum_voltage : float
                Maximum voltage of a single cell [V]
            - cell.mass : float
                Mass of a single cell [kg]
            - electrical_configuration.series : int
                Number of cells in series
            - electrical_configuration.parallel : int
                Number of cells in parallel
            - geometrtic_configuration : GeometricConfiguration
                Object containing geometric layout information
            - volume_packaging_factor : float
                Factor accounting for additional volume due to packaging
            - orientation_euler_angles : numpy.ndarray
                Euler angles for module orientation [rad]
            - BMS_additional_weight_factor : float
                Factor accounting for battery management system weight
    
    Returns
    -------
    None
        This function modifies the battery_module object in-place, setting the following attributes:
            - length : float
                Length of the battery module [m]
            - width : float
                Width of the battery module [m]
            - height : float
                Height of the battery module [m]
            - mass_properties.mass : float
                Total mass of the battery module [kg]
            - specific_energy : float
                Specific energy of the battery module [Wh/kg]
            - maximum_energy : float
                Maximum energy capacity of the battery module [J]
            - specific_power : float
                Specific power of the battery module [W/kg]
            - maximum_power : float
                Maximum power output of the battery module [W]
            - maximum_voltage : float
                Maximum voltage of the battery module [V]
            - initial_maximum_energy : float
                Initial maximum energy capacity [J]
            - nominal_capacity : float
                Nominal capacity of the battery module [Ah]
            - voltage : float
                Voltage of the battery module [V]
    
    Notes
    -----
    This function calculates the physical and electrical properties of a battery module
    based on the properties of its constituent cells and the module configuration.
    
    The physical dimensions are calculated based on the geometric layout of cells,
    including spacing between cells and a volume packaging factor. The module orientation
    can be adjusted using Euler angles.
    
    The electrical properties are calculated based on the series-parallel configuration
    of cells, with series connections increasing voltage and parallel connections
    increasing capacity.
    
    **Major Assumptions**
        * Total battery module pack mass can be modeled with a build-up factor for battery module casing,
          internal wires, thermal management system and battery module management system.
    
    References
    ----------
    [1] Chin, J. C., Schnulo, S. L., Miller, T. B., Prokopius, K., and Gray, J., "Battery Performance Modeling on Maxwell X-57", AIAA Scitech, San Diego, CA, 2019. URL http://openmdao.org/pubs/chin_battery_performance_x57_2019.pdf.
    """
    

    series_e           = battery_module.electrical_configuration.series
    parallel_e         = battery_module.electrical_configuration.parallel 
    normal_count       = battery_module.geometrtic_configuration.normal_count  
    parallel_count     = battery_module.geometrtic_configuration.parallel_count
    stacking_rows      = battery_module.geometrtic_configuration.stacking_rows

    if int(parallel_e*series_e) != int(normal_count*parallel_count):
        pass #raise Exception('Number of cells in gemetric layout not equal to number of cells in electric circuit configuration ')
        
        
    normal_spacing     = battery_module.geometrtic_configuration.normal_spacing   
    parallel_spacing   = battery_module.geometrtic_configuration.parallel_spacing
    volume_factor      = battery_module.volume_packaging_factor
    cell_diameter      = battery_module.cell.diameter
    cell_height        = battery_module.cell.height  
    euler_angles       = battery_module.orientation_euler_angles
    weight_factor      = battery_module.BMS_additional_weight_factor
    
    x1 =  normal_count * (cell_diameter + normal_spacing) * volume_factor # distance in the module-level normal direction
    x2 =  parallel_count * (cell_diameter + parallel_spacing) * volume_factor # distance in the module-level parallel direction
    x3 =  cell_height * volume_factor # distance in the module-level height direction 

    length = x1 / stacking_rows
    width  = x2
    height = x3 *stacking_rows     
    
    if  euler_angles[0] == (np.pi / 2):
        x1prime      = x2
        x2prime      = -x1
        x3prime      = x3 
    if euler_angles[1] == (np.pi / 2):
        x1primeprime = -x3prime
        x2primeprime = x2prime
        x3primeprime = x1prime
    if euler_angles[2] == (np.pi / 2):
        length       = x1primeprime
        width        = x3primeprime
        height       = -x2primeprime

    # store length, width and height
    battery_module.length = length
    battery_module.width  = width
    battery_module.height = height 
    
    amp_hour_rating                               = battery_module.cell.nominal_capacity   
    total_battery_assemply_mass                   = battery_module.cell.mass * series_e * parallel_e  
    battery_module.mass_properties.mass           = total_battery_assemply_mass*weight_factor  
    battery_module.specific_energy                = (amp_hour_rating*battery_module.cell.maximum_voltage)/battery_module.cell.mass  * Units.Wh/Units.kg
    battery_module.maximum_energy                 = total_battery_assemply_mass*battery_module.specific_energy    
    battery_module.specific_power                 = battery_module.specific_energy/battery_module.cell.nominal_capacity 
    battery_module.maximum_power                  = battery_module.specific_power*battery_module.mass_properties.mass  
    battery_module.maximum_voltage                = battery_module.cell.maximum_voltage  * series_e   
    battery_module.initial_maximum_energy         = battery_module.maximum_energy      
    battery_module.nominal_capacity               = battery_module.cell.nominal_capacity* parallel_e 
    battery_module.voltage                        = battery_module.maximum_voltage 
