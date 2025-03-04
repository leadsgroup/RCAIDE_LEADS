# RCAIDE/Library/Compoments/Powertrain/Sources/Batteries/Lithium_Ion_LiNiMnCoO2_18650.py
# 
# 
# Created:  Mar 2024, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                            import Units , Data
from .Generic_Battery_Module                                          import Generic_Battery_Module   
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Lithium_Ion_NMC  import *
# package imports 
import numpy as np
import os 
from scipy.interpolate  import RegularGridInterpolator 

# ----------------------------------------------------------------------------------------------------------------------
#  Lithium_Ion_NMC
# ----------------------------------------------------------------------------------------------------------------------  
class Lithium_Ion_NMC(Generic_Battery_Module):
    """
    Class for modeling 18650-format lithium nickel manganese cobalt oxide batteries
    
    Attributes
    ----------
    tag : str
        Identifier for the battery module (default: 'lithium_ion_nmc')
        
    maximum_energy : float
        Maximum energy storage capacity [J] (default: 0.0)
        
    maximum_power : float
        Maximum power output [W] (default: 0.0)
        
    maximum_voltage : float
        Maximum voltage output [V] (default: 0.0)
        
    cell : Data
        Cell-specific properties
        
        - chemistry : str
            Battery chemistry type (default: 'LiNiMnCoO2')
        - diameter : float
            Cell diameter [m] (default: 0.0185)
        - height : float
            Cell height [m] (default: 0.0653)
        - mass : float
            Cell mass [kg] (default: 0.048)
        - surface_area : float
            Total cell surface area [m^2]
        - volume : float
            Cell volume [m^3]
        - density : float
            Cell density [kg/m^3]
        - electrode_area : float
            Active electrode area [m^2] (default: 0.0342)
        - maximum_voltage : float
            Maximum cell voltage [V] (default: 4.2)
        - nominal_capacity : float
            Rated capacity [Ah] (default: 3.8)
        - nominal_voltage : float
            Nominal operating voltage [V] (default: 3.6)
        - charging_voltage : float
            Charging voltage [V] (default: nominal_voltage)
        - resistance : float
            Internal resistance [Ohms] (default: 0.025)
        - specific_heat_capacity : float
            Cell specific heat [J/kgK] (default: 1108)
        - radial_thermal_conductivity : float
            Radial thermal conductivity [W/mK] (default: 0.4)
        - axial_thermal_conductivity : float
            Axial thermal conductivity [W/mK] (default: 32.2)
        - discharge_performance_map : RegularGridInterpolator
            Interpolator for voltage vs discharge characteristics

    Notes
    -----
    The NMC cell model includes detailed thermal and electrical characteristics
    based on 18650-format cells. The model includes forced air cooling assumptions
    with a convective heat transfer coefficient for 35 m/s airflow.

    References
    ----------
    [1] Jeon, D. H., & Baek, S. M. (2011). Thermal modeling of cylindrical 
        lithium ion battery during discharge cycle. Energy Conversion and Management,
        52(8-9), 2973-2981.
        
    [2] Yang, S., et al. (2019). A Review of Lithium-Ion Battery Thermal Management 
        System Strategies and the Evaluate Criteria. Int. J. Electrochem. Sci, 14,
        6077-6107.
        
    [3] Muenzel, V., et al. (2015). A comparative testing study of commercial
        18650-format lithium-ion battery cells. Journal of The Electrochemical
        Society, 162(8), A1592.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module
        Base battery module class
    """       
    
    def __defaults__(self):   
        """This sets the default values.
    
        Assumptions:
            Convective Thermal Conductivity Coefficient corresponds to forced
            air cooling in 35 m/s air 
        
        Source: 
            convective  heat transfer coefficient, h 
            Jeon, Dong Hyup, and Seung Man Baek. "Thermal modeling of cylindrical 
            lithium ion battery during discharge cycle." Energy Conversion and Management
            52.8-9 (2011): 2973-2981.
            
            thermal conductivity, k 
            Yang, Shuting, et al. "A Review of Lithium-Ion Battery Thermal Management 
            System Strategies and the Evaluate Criteria." Int. J. Electrochem. Sci 14
            (2019): 6077-6107.
            
            specific heat capacity, Cp
            (axial and radial)
            Yang, Shuting, et al. "A Review of Lithium-Ion Battery Thermal Management 
            System Strategies and the Evaluate Criteria." Int. J. Electrochem. Sci 14
            (2019): 6077-6107.
            
            # Electrode Area
            Muenzel, Valentin, et al. "A comparative testing study of commercial
            18650-format lithium-ion battery cells." Journal of The Electrochemical
            Society 162.8 (2015): A1592.
        
        """
        # ----------------------------------------------------------------------------------------------------------------------
        #  Module Level Properties
        # ----------------------------------------------------------------------------------------------------------------------
        
        self.tag                                         = 'lithium_ion_nmc'
        self.maximum_energy                              = 0.0
        self.maximum_power                               = 0.0
        self.maximum_voltage                             = 0.0  
         
        # ----------------------------------------------------------------------------------------------------------------------
        #  Cell Level Properties
        # ----------------------------------------------------------------------------------------------------------------------        
        self.cell.chemistry                   = 'LiNiMnCoO2'
        self.cell.diameter                    = 0.0185                                                                            # [m]
        self.cell.height                      = 0.0653                                                                            # [m]
        self.cell.mass                        = 0.048 * Units.kg                                                                  # [kg]
        self.cell.surface_area                = (np.pi*self.cell.height*self.cell.diameter) + (0.5*np.pi*self.cell.diameter**2)  # [m^2]
        self.cell.volume                      = np.pi*(0.5*self.cell.diameter)**2*self.cell.height 
        self.cell.density                     = self.cell.mass/self.cell.volume                                                  # [kg/m^3]  
        self.cell.electrode_area              = 0.0342                                                                           # [m^2] 
                                                                                                                           
        self.cell.maximum_voltage             = 4.2                                                                              # [V]
        self.cell.nominal_capacity            = 3.8                                                                             # [Amp-Hrs]
        self.cell.nominal_voltage             = 3.6                                                                              # [V] 
        self.cell.charging_voltage            = self.cell.nominal_voltage                                                        # [V] 
        
        self.cell.watt_hour_rating            = self.cell.nominal_capacity  * self.cell.nominal_voltage                          # [Watt-hours]      
        self.cell.specific_energy             = self.cell.watt_hour_rating*Units.Wh/self.cell.mass                               # [J/kg]
        self.cell.specific_power              = self.cell.specific_energy/self.cell.nominal_capacity                             # [W/kg]   
        self.cell.resistance                  = 0.025                                                                            # [Ohms] 
                                                            
        self.cell.specific_heat_capacity      = 1108                                                                             # [J/kgK]    
        self.cell.radial_thermal_conductivity = 0.4                                                                              # [J/kgK]  
        self.cell.axial_thermal_conductivity  = 32.2                                                                             # [J/kgK] # estimated
    
                                              
        battery_raw_data                      = load_battery_results()                                                   
        self.cell.discharge_performance_map   = create_discharge_performance_map(battery_raw_data)  

        return  
    
    def energy_calc(self,state,bus,coolant_lines, t_idx, delta_t): 
        """
        Computes the state of the NMC battery cell
        
        This method calculates the battery's electrical performance and thermal
        behavior during operation, including voltage, current, power, and 
        temperature distributions.

        Parameters
        ----------
        state : Data
            Current system state containing:
            - Temperature distributions
            - Power demands
            - Operating conditions
            
        bus : Component
            Connected electrical bus containing:
            - Voltage requirements
            - Power requirements
            - Load characteristics
            
        coolant_lines : Component
            Thermal management system containing:
            - Coolant properties
            - Flow conditions
            - Heat exchanger parameters
            
        t_idx : int
            Current time index in the simulation
            
        delta_t : float
            Time step size [s]

        Returns
        -------
        stored_results_flag : bool
            Flag indicating if results were stored for future reuse
            
        stored_battery_tag : str
            Identifier for stored battery state data

        Notes
        -----
        The calculation includes:
        - Voltage and current based on load demand
        - Heat generation from internal resistance
        - Thermal distribution with cooling effects
        - State of charge tracking
        """        
        stored_results_flag, stored_battery_tag =  compute_nmc_cell_performance(self,state,bus,coolant_lines, t_idx,delta_t) 
        
        return stored_results_flag, stored_battery_tag
    
    def reuse_stored_data(self,state,bus,stored_results_flag, stored_battery_tag):
        reuse_stored_nmc_cell_data(self,state,bus,stored_results_flag, stored_battery_tag)
        return 
    
    def update_battery_age(self,segment,battery_conditions,increment_battery_age_by_one_day = False):  
        """
        Updates battery aging parameters based on usage and environmental conditions
        
        This method tracks battery degradation by considering factors such as:
        cycle count, depth of discharge, temperature exposure, and calendar aging.

        Parameters
        ----------
        segment : Segment
            Flight segment containing:
            - Duration
            - Operating conditions
            - Power profile
            
        battery_conditions : Data
            Battery state data including:
            - Temperature history
            - Current rates
            - State of charge history
            
        increment_battery_age_by_one_day : bool, optional
            Flag to increment calendar age (default: False)

        Notes
        -----
        The aging model accounts for:
        - Capacity fade from cycling
        - Calendar aging effects
        - Temperature-dependent degradation
        - Current rate impacts
        """        
        update_nmc_cell_age(self,segment,battery_conditions,increment_battery_age_by_one_day) 
        
        return  

def create_discharge_performance_map(raw_data):
    """
    Creates discharge and charge response surface for a LiNiMnCoO2 battery cell   

    Parameters
    ----------
    raw_data : Data
        Container with experimental battery data including:
        - Voltage : array
            Discharge voltage curves at different currents and temperatures
        - Temperature : array
            Cell temperature profiles during discharge

    Returns
    -------
    battery_data : Data
        Container with interpolation functions:
        - Voltage : RegularGridInterpolator
            Predicts voltage based on [current, temperature, SOC]
        - Temperature : RegularGridInterpolator
            Predicts cell temperature based on [current, temperature, SOC]

    Notes
    -----
    The function creates 3D interpolations for:
    - Voltage as function of current (0-8A), temperature (0-50°C), and SOC (0-1)
    - Temperature rise as function of same parameters
    
    Uses regular grid interpolation for smooth predictions across the operating space.
    """   
    # Process raw data   
    processed_data = Data() 
    processed_data.Voltage        = np.zeros((5,6,15,2)) # current , operating temperature , state_of_charge vs voltage      
    processed_data.Temperature    = np.zeros((5,6,15,2)) # current , operating temperature , state_of_charge vs temperature 

    # Reshape  Data          
    raw_data.Voltage 
    for i, Amps in enumerate(raw_data.Voltage):
        for j , Deg in enumerate(Amps):
            min_x    = 0 
            max_x    = max(Deg[:,0])
            x        = np.linspace(min_x,max_x,15)
            y        = np.interp(x,Deg[:,0],Deg[:,1])
            vec      = np.zeros((15,2))
            vec[:,0] = x/max_x
            vec[:,1] = y
            processed_data.Voltage[i,j,:,:]= vec   

    for i, Amps in enumerate(raw_data.Temperature):
        for j , Deg in enumerate(Amps):
            min_x    = 0   
            max_x    = max(Deg[:,0])
            x        = np.linspace(min_x,max_x,15)
            y        = np.interp(x,Deg[:,0],Deg[:,1])
            vec      = np.zeros((15,2))
            vec[:,0] = x/max_x
            vec[:,1] = y
            processed_data.Temperature[i,j,:,:]= vec  
    
    # Create performance maps  
    battery_data             = Data() 
    amps                    = np.linspace(0, 8, 5)
    temp                    = np.linspace(0, 50, 6) +  272.65  # Convert to Kelvin
    SOC                     = np.linspace(0, 1, 15)
    battery_data.Voltage     = RegularGridInterpolator((amps, temp, SOC), processed_data.Voltage,bounds_error=False,fill_value=None)
    battery_data.Temperature = RegularGridInterpolator((amps, temp, SOC), processed_data.Temperature,bounds_error=False,fill_value=None) 
     
    return battery_data  

def load_battery_results(): 
    '''Load experimental raw data of NMC cells 
        
       Assumptions:
           Ideal gas
           
       Source:
           Automotive Industrial Systems Company of Panasonic Group, Technical Information of 
           NCR18650G, URL https://www.imrbatteries.com/content/panasonic_ncr18650g.pdf
    
       Args: 
           None
           
       Returns:
           battery_data: raw data from battery   [unitless]
    '''    
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator     
    return RCAIDE.load(rel_path+ 'NMC_Raw_Data.res')