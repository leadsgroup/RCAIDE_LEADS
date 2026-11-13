# RCAIDE/Library/Components/Powertrain/Powertrain/Sources/Batteries/Modules/Lithium_Ion_LFP.py
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
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.append_battery_unknown_and_residual      import append_battery_unknown_and_residual

# package imports 
import numpy as np  
import os

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
            - discharge_performance_map : Data
                Regular-grid table of voltage over C-rate, temperature and discharge capacity

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
    RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Generic_Battery_Module
        Base battery module class
    """
    
    def __defaults__(self):
    
        # ----------------------------------------------------------------------------------------------------------------------
        #  Module Level Properties
        # ----------------------------------------------------------------------------------------------------------------------        
        self.tag                              = 'lithium_ion_lfp'  
        self.number_of_cells                  = 1
        self.maximum_energy                   = 0.0
        self.maximum_power                    = 0.0
        self.maximum_voltage                  = 0.0       
        
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
        self.cell.specific_power              = 1500.                                                      # [W/kg] LFP typically supports higher continuous discharge power than NMC at similar mass
        self.cell.resistance                  = 0.022                                                     # [Ohms]
                                                                                                            
        self.cell.specific_heat_capacity      = 1115                                                      # [J/kgK]                                                     
        self.cell.radial_thermal_conductivity = 0.475                                                     # [J/kgK]  
        self.cell.axial_thermal_conductivity  = 37.6                                                      # [J/kgK]  

        battery_raw_data                      = load_battery_results()                                                   
        self.cell.discharge_performance_map   = create_discharge_performance_map(battery_raw_data)

        return                       

    def append_unknowns_and_residuals(self,battery,segment):
        """
        Registers the cell temperature and state-of-charge unknowns/residuals,
        solved implicitly alongside every other network unknown.
        """
        append_battery_unknown_and_residual(self,battery,segment)
        return

    def unpack_unknowns(self,battery,segment):
        return

    def pack_residuals(self,battery,segment):
        return

    def compute_performance(self,battery,state,network):
        """
        Computes the state of the LFP battery cell

        Parameters
        ----------
        battery : Battery_Pack
            The battery pack containing this module
        state : Data
            Current system state
        network : Network
            The powertrain network

        Returns
        -------
        stored_results_flag : bool
            Flag indicating if results were stored
        stored_battery_tag : str
            Identifier for stored results
        """
        inputs, outputs, stored_results_flag, stored_source_tag =  compute_lfp_cell_performance(self,battery,state,network)
                        
        return inputs, outputs, stored_results_flag, stored_source_tag
    
    def reuse_stored_data(self,state,stored_battery_tag,stored_battery_module_tag):
        """
        Reuses previously stored battery performance data

        Parameters
        ----------
        state : Data
            Current system state
        stored_battery_tag : str
            Tag of the battery pack whose module results are being reused
        stored_battery_module_tag : str
            Tag of the module whose results are being reused
        """
        inputs, outputs = reuse_stored_lfp_cell_data(self,state,stored_battery_tag,stored_battery_module_tag)
        return inputs, outputs
      
    def update_battery_age(self,battery,segment,increment_battery_age_by_one_day): 
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
        update_lfp_cell_age(self,battery,segment,increment_battery_age_by_one_day)
        return 
    
def create_discharge_performance_map(raw_data, cutoff_voltage = 2.0, charge_voltage = 3.6, capacity_points = 361):
    """
    Creates the regular-grid table of battery discharge voltage over C-rate, temperature and discharge capacity.

    Parameters
    ----------
    raw_data : dict
        Dictionary containing battery test data with structure:
        {c_rate: {temperature: {'discharge': [...], 'voltage': [...]}}}
    cutoff_voltage : float
        Discharge cut-off voltage of the cell [V]; 2.0 V, the minimum voltage of the ANR26650M1B [2].
    charge_voltage : float
        Highest voltage of the cell [V]; 3.6 V, the recommended (fast) charge voltage of the ANR26650M1B [2], which a
        discharging cell does not exceed.
    capacity_points : int
        Number of discharge capacity points of the common capacity grid.

    Returns
    -------
    battery_data : Data
        grid   : (C-rates, temperatures [deg C], discharge capacities [Ah]), increasing
        values : voltage on that grid [V], shape (C-rates, temperatures, capacities)

    Notes
    -----
    Every discharge curve is resampled onto one capacity grid from zero to the largest capacity in the data, so the
    table is regular and is interpolated multilinearly (regular_grid_interpolation) on NumPy and JAX. Each curve
    ends where the cell would stop discharging: from its first point below the cut-off voltage, or above the charge
    voltage, it holds the cut-off voltage. Beyond that point the data at high C-rates and deep discharge is not
    physical (voltages from -600 V to +36 V). The small voltage recovery of cold cells at high rates, as they heat
    up, stays within these limits and is kept.

    References
    ----------
    [1] Lin, X., Perez, H., Siegel, J. B., & Stefanopoulou, A. G. (2024).
        "An Electro-Thermal Model for the A123 26650 LiFePO4 Battery."
        University of Michigan.
        https://hdl.handle.net/2027.42/97341
    [2] Lithium Werks, "26650 Power Cell, ANR26650M1B" datasheet (2022): Minimum Voltage / HPPC Pulse 2 V / 1.6 V;
        Recommended Fast Charge Voltage 3.6 V.
        https://lithiumwerks.com/wp-content/uploads/2022/08/26650-Power-Cell-080322.pdf
    """
    c_rates      = np.array(sorted(float(c_rate_key) for c_rate_key in raw_data.keys()))
    first_c_rate = next(iter(raw_data.values()))
    temperatures = np.array(sorted(int(temp_key.split()[-1]) for temp_key in first_c_rate.keys()))
    max_capacity = max(np.max(data['discharge']) for temp_data in raw_data.values() for data in temp_data.values())
    capacities   = np.linspace(0., max_capacity, capacity_points)

    voltages = np.full((len(c_rates), len(temperatures), capacity_points), np.nan)
    for c_rate_key, temp_data in raw_data.items():
        i = int(np.flatnonzero(c_rates == float(c_rate_key))[0])
        for temp_key, data in temp_data.items():
            j         = int(np.flatnonzero(temperatures == int(temp_key.split()[-1]))[0])
            discharge = np.asarray(data['discharge'], dtype=float)
            voltage   = np.asarray(data['voltage'], dtype=float)
            # the curve ends at its first point outside [cut-off, charge voltage]; the cell holds the cut-off after
            outside = (voltage < cutoff_voltage) | (voltage > charge_voltage)
            if np.any(outside):
                voltage = np.where(np.arange(len(voltage)) >= np.argmax(outside), cutoff_voltage, voltage)
            voltages[i, j] = np.interp(capacities, discharge, voltage)

    return Data(grid = (c_rates, temperatures.astype(float), capacities), values = voltages)

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