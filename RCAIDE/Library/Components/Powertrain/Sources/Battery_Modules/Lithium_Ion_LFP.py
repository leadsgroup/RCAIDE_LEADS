# RCAIDE/Library/Components/Powertrain/Energy/Sources/Battery_Modules/Lithium_Ion_LFP.py
# 
# 
# Created: Nov 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core          import Units,Data
from .Generic_Battery_Module import  Generic_Battery_Module
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Lithium_Ion_LFP  import * 

# package imports 
import numpy as np  
import os
from scipy.interpolate  import NearestNDInterpolator

# ----------------------------------------------------------------------------------------------------------------------
#  Lithium_Ion_LFP
# ----------------------------------------------------------------------------------------------------------------------  
class Lithium_Ion_LFP(Generic_Battery_Module):
    """
    Class for modeling A123 26650 lithium iron phosphate battery characteristics
    
    Attributes
    ----------
    tag : str
        Identifier for the battery module (default: 'lithium_ion_lfp')
        
    power_split_ratio : float, optional
        Power distribution ratio for multiple battery systems
        
    number_of_cells : int
        Number of cells in the module (default: 1)
        
    maximum_energy : float
        Maximum energy storage capacity [J] (default: 0.0)
        
    maximum_power : float
        Maximum power output [W] (default: 0.0)
        
    maximum_voltage : float
        Maximum voltage output [V] (default: 0.0)
        
    cell : Data
        Cell-specific properties
        
        - chemistry : str
            Battery chemistry type (default: 'LiFePO4')
        - diameter : float
            Cell diameter [m] (default: 0.0185)
        - height : float
            Cell height [m] (default: 0.0653)
        - mass : float
            Cell mass [kg] (default: 0.03)
        - surface_area : float
            Total cell surface area [m^2]
        - volume : float
            Cell volume [m^3]
        - density : float
            Cell density [kg/m^3]
        - electrode_area : float
            Active electrode area [m^2] (default: 0.0342)
        - maximum_voltage : float
            Maximum cell voltage [V] (default: 3.6)
        - nominal_capacity : float
            Rated capacity [Ah] (default: 2.6)
        - nominal_voltage : float
            Nominal operating voltage [V] (default: 3.6)
        - resistance : float
            Internal resistance [Ohms] (default: 0.022)
        - specific_heat_capacity : float
            Cell specific heat [J/kgK] (default: 1115)
        - radial_thermal_conductivity : float
            Radial thermal conductivity [W/mK] (default: 0.475)
        - axial_thermal_conductivity : float
            Axial thermal conductivity [W/mK] (default: 37.6)
        - discharge_performance_map : NearestNDInterpolator
            Interpolator for voltage vs discharge characteristics

    Notes
    -----
    The LFP cell model includes detailed thermal and electrical characteristics
    based on the A123 26650 cell. Performance data is interpolated from
    experimental measurements.

    References
    ----------
    [1] LithiumWerks (2019).A123 26650 Datasheet
        https://a123batteries.com/product_images/uploaded_images/26650.pdf
        
    [2] Arora, S., & Kapoor, A. (2019). Experimental Study of Heat Generation 
        Rate during Discharge of LiFePO4 Pouch Cells. Batteries, 5(4), 70.
        https://doi.org/10.3390/batteries5040070

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module
        Base battery module class
    """
    
    def __defaults__(self):
    
        # ----------------------------------------------------------------------------------------------------------------------
        #  Module Level Properties
        # ----------------------------------------------------------------------------------------------------------------------        
        self.tag                                               = 'lithium_ion_lfp' 
        self.power_split_ratio                                 = None
        self.number_of_cells                                   = 1
        self.maximum_energy                                    = 0.0
        self.maximum_power                                     = 0.0
        self.maximum_voltage                                   = 0.0       
        
        # ----------------------------------------------------------------------------------------------------------------------
        #  Cell Level Properties
        # ----------------------------------------------------------------------------------------------------------------------        
        self.cell.chemistry                   = 'LiFePO4'
        self.cell.diameter                    = 0.0185                                                    # [m]
        self.cell.height                      = 0.0653                                                    # [m]
        self.cell.mass                        = 0.03  * Units.kg                                          # [kg]
        self.cell.surface_area                = (np.pi*self.cell.height*self.cell.diameter) \
                                                + (0.5*np.pi*self.cell.diameter**2)                       # [m^2]

        self.cell.volume                      = np.pi*(0.5*self.cell.diameter)**2*self.cell.height        # [m^3] 
        self.cell.density                     = self.cell.mass/self.cell.volume                           # [kg/m^3]
        self.cell.electrode_area              = 0.0342                                                    # [m^2]  # estimated 
                                                        
        self.cell.maximum_voltage             = 3.6                                                       # [V]
        self.cell.nominal_capacity            = 2.6                                                       # [Amp-Hrs]
        self.cell.nominal_voltage             = 3.6                                                       # [V]
         
        self.cell.watt_hour_rating            = self.cell.nominal_capacity  * self.cell.nominal_voltage   # [Watt-hours]      
        self.cell.specific_energy             = self.cell.watt_hour_rating*Units.Wh/self.cell.mass        # [J/kg]
        self.cell.specific_power              = self.cell.specific_energy/self.cell.nominal_capacity      # [W/kg]   
        self.cell.resistance                  = 0.022                                                     # [Ohms]
                                                                                                            
        self.cell.specific_heat_capacity      = 1115                                                      # [J/kgK]                                                     
        self.cell.radial_thermal_conductivity = 0.475                                                     # [J/kgK]  
        self.cell.axial_thermal_conductivity  = 37.6                                                      # [J/kgK]  

        battery_raw_data                      = load_battery_results()                                                   
        self.cell.discharge_performance_map   = create_discharge_performance_map(battery_raw_data)

        return                                     

    def energy_calc(self,state,bus,coolant_lines, t_idx, delta_t): 
        """
        Computes the state of the LFP battery cell
        
        Parameters
        ----------
        state : Data
            Current system state
        bus : Component
            Connected electrical bus
        coolant_lines : Component
            Connected cooling system
        t_idx : int
            Time index
        delta_t : float
            Time step [s]
            
        Returns
        -------
        stored_results_flag : bool
            Flag indicating if results were stored
        stored_battery_tag : str
            Identifier for stored results
        """      
        stored_results_flag, stored_battery_tag =  compute_lfp_cell_performance(self,state,bus,coolant_lines, t_idx,delta_t) 
                        
        return stored_results_flag, stored_battery_tag
    
    def reuse_stored_data(self,state,bus,stored_results_flag, stored_battery_tag):
        """
        Reuses previously stored battery performance data
        
        Parameters
        ----------
        state : Data
            Current system state
        bus : Component
            Connected electrical bus
        coolant_lines : Component
            Connected cooling system
        t_idx : int
            Time index
        delta_t : float
            Time step [s]
        stored_results_flag : bool
            Flag indicating stored results exist
        stored_battery_tag : str
            Identifier for stored results
        """
        reuse_stored_lfp_cell_data(self,state,bus,stored_results_flag, stored_battery_tag)
        return    
      
    def update_battery_age(self,segment, battery_conditions,increment_battery_age_by_one_day): 
        """
        Updates battery age and degradation parameters
        
        Parameters
        ----------
        segment : Segment
            Flight segment data
        battery_conditions : Data
            Battery operating conditions
        increment_battery_age_by_one_day : bool
            Flag to increment battery age
        """
        update_lfp_cell_age(self,segment, battery_conditions,increment_battery_age_by_one_day)
        return 
    
def create_discharge_performance_map(raw_data):
    """
    Creates an interpolator for battery discharge voltage characteristics
    
    This function processes raw battery test data to create an interpolation
    function that predicts battery voltage based on C-rate, temperature,
    and discharge capacity.

    Parameters
    ----------
    raw_data : dict
        Dictionary containing battery test data with structure:
        {c_rate: {temperature: {'discharge': [...], 'voltage': [...]}}}

    Returns
    -------
    battery_data : NearestNDInterpolator
        Interpolator function that takes [C-rate, temperature, discharge_capacity]
        and returns voltage

    Notes
    -----
    The function creates a 3D interpolation of voltage as a function of:
    - C-rate (discharge current relative to capacity)
    - Temperature
    - Discharge capacity (state of charge)
    
    Uses nearest neighbor interpolation for computational efficiency. Linear
    interpolation is possible but increases computation time by ~30x.

    References
    ----------
    [1] Lin, X., Perez, H., Siegel, J. B., & Stefanopoulou, A. G. (2024).
        "An Electro-Thermal Model for the A123 26650 LiFePO4 Battery."
        University of Michigan.
        https://hdl.handle.net/2027.42/97341
    """    
    # Prepare lists for the data needed for interpolation
    c_rates = []
    temperatures = []
    discharge_capacities = []
    voltages = []

    # Iterate through the data structure to populate the lists
    for c_rate_key, temp_data in raw_data.items():
        c_rate = float(c_rate_key)  # Convert C-rate to float

        for temp_key, data in temp_data.items():
            initial_temp = int(temp_key.split()[-1])  # Extract temperature as an integer

            # Extend lists with the discharge, voltage, and other data
            discharge_capacities.extend(data['discharge'])
            voltages.extend(data['voltage'])
            c_rates.extend([c_rate] * len(data['discharge']))
            temperatures.extend([initial_temp] * len(data['discharge']))

    # Convert lists to numpy arrays
    points = np.array([c_rates, temperatures, discharge_capacities]).T
    values = np.array(voltages)

    # Create the interpolant
    battery_data = NearestNDInterpolator(points, values) # Can be replaced by a Linear Interpolator for a better fit but computation time increases by 30 times. 

    return battery_data

def load_battery_results(): 
    '''Load experimental raw data of LFP cells 
        
    Assumptions:
        
        
    Source:
        
    
    Args: 
        None
        
    Returns:
        battery_data: raw data from battery   [unitless]
    '''    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(current_dir, 'lfp_raw_data.res')

    # Load the raw_data using RCAIDE.load()
    raw_data = RCAIDE.load(full_path)

    return raw_data