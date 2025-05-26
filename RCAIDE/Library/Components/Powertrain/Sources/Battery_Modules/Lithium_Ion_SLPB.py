# RCAIDE/Library/Compoments/Powertrain/Sources/Battery_Modules/Lithium_Ion_SLPB.py
# 
# 
# Created:  May 2025, C. Boggan
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                            import Units , Data
from RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module  import Generic_Battery_Module   
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Lithium_Ion_SLPB import *
# package imports 
import numpy as np
import os 
from scipy.interpolate  import RegularGridInterpolator 
import pybamm

# ----------------------------------------------------------------------------------------------------------------------
#  Lithium_Ion_SLPB
# ----------------------------------------------------------------------------------------------------------------------  
class Lithium_Ion_SLPB(Generic_Battery_Module):
    
    def __defaults__(self):   
        
        # ----------------------------------------------------------------------------------------------------------------------
        #  Module Level Properties
        # ----------------------------------------------------------------------------------------------------------------------
        
        self.tag                                         = 'lithium_ion_slpb'
        self.maximum_energy                              = 0.0
        self.maximum_power                               = 0.0
        self.maximum_voltage                             = 0.0  
         
        # ----------------------------------------------------------------------------------------------------------------------
        #  Cell Level Properties
        # ----------------------------------------------------------------------------------------------------------------------        
        self.cell.thickness                   = 0.0075                                                                            # [m]
        self.cell.width                       = 0.106
        self.cell.height                      = 0.1                                                                           # [m]
        self.cell.mass                        = 0.155 * Units.kg                                                                  # [kg]
        self.cell.surface_area                = 2 * (self.cell.thickness * self.cell.width) + 2 * (self.cell.thickness * self.cell.height) + 2 * (self.cell.width * self.cell.height)  # [m^2]
        self.cell.volume                      = self.cell.thickness * self.cell.width * self.cell.height                        # [m^3]
        self.cell.density                     = self.cell.mass/self.cell.volume                                                  # [kg/m^3]  
                                                                                                                           
        self.cell.maximum_voltage             = 4.2                                                                              # [V]
        self.cell.nominal_capacity            = 7.5                                                                                 # [Amp-Hrs]
        self.cell.nominal_voltage             = 3.7                                                                              # [V] 
        self.cell.charging_voltage            = self.cell.nominal_voltage                                                       # [V] 
        self.cell.maximum_discharge_current   = 40  #8 for continuous                                                                               # [Amps] 
        self.cell.nominal_charging_current    = 3   #unknown for now                                                                             # [Amps]  
        self.cell.maximum_charging_current    = 9   #unknown for now                                                              # [Amps]    


        
        self.cell.watt_hour_rating            = self.cell.nominal_capacity  * self.cell.nominal_voltage                          # [Watt-hours]      
        self.cell.specific_energy             = self.cell.watt_hour_rating*Units.Wh/self.cell.mass                               # [J/kg]
        
        # ----------------------------------------------------------------------------------------------------------------------
        #  PyBaMM Model
        # ----------------------------------------------------------------------------------------------------------------------   
        self.cell.model = pybamm.lithium_ion.SPM( 
            {
                "SEI": "solvent-diffusion limited",
                "SEI porosity change": "true",
                "lithium plating": "partially reversible",
                "lithium plating porosity change": "true",
                #"particle mechanics": ("swelling and cracking", "swelling only"),
                "loss of active material": "reaction-driven",
                "calculate discharge energy": "true",  
                "thermal": "lumped"
            })
    
        self.cell.battery_parameters =  pybamm.ParameterValues("Ecker2015")
        self.cell.battery_parameters.update({ "Negative electrode thickness [m]": 1.62e-04*20,
                             "Positive electrode thickness [m]": 1.24e-04*20})
                                  
        return  
    
    def energy_calc(self,state,bus,coolant_lines, t_idx, delta_t): 
             
        stored_results_flag, stored_battery_tag =  compute_SLPB_cell_performance(self,state,bus,coolant_lines, t_idx,delta_t) 
        
        return stored_results_flag, stored_battery_tag
    
    def reuse_stored_data(self,state,bus,stored_results_flag, stored_battery_tag):
        
        reuse_stored_SLPB_cell_data(self,state,bus,stored_results_flag, stored_battery_tag)
        
        return 
    
    def update_battery_age(self,segment,battery_conditions, increment_battery_age_by_one_day = False):  
              
        update_SLPB_cell_age(battery_conditions, increment_battery_age_by_one_day) 
        
        return  
