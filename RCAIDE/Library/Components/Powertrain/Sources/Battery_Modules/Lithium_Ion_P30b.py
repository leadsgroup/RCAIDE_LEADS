# RCAIDE/Library/Compoments/Powertrain/Sources/Battery_Modules/Lithium_Ion_P30b.py
# 
# 
# Created:  Mar 2024, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                            import Units , Data
from RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module  import Generic_Battery_Module   
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Lithium_Ion_P30b import *
# package imports 
import numpy as np
import os 
from scipy.interpolate  import RegularGridInterpolator 
import pybamm

# ----------------------------------------------------------------------------------------------------------------------
#  Lithium_Ion_NMC
# ----------------------------------------------------------------------------------------------------------------------  
class Lithium_Ion_P30b(Generic_Battery_Module):
    """
    """       
    
    def __defaults__(self):   
        """
        """
        # ----------------------------------------------------------------------------------------------------------------------
        #  Module Level Properties
        # ----------------------------------------------------------------------------------------------------------------------
        
        self.tag                                         = 'lithium_ion_p30b'
        self.maximum_energy                              = 0.0
        self.maximum_power                               = 0.0
        self.maximum_voltage                             = 0.0  
         
        # ----------------------------------------------------------------------------------------------------------------------
        #  Cell Level Properties
        # ----------------------------------------------------------------------------------------------------------------------        
        self.cell.diameter                    = 0.0186                                                                            # [m]
        self.cell.height                      = 0.0652                                                                            # [m]
        self.cell.mass                        = 0.047 * Units.kg                                                                  # [kg]
        self.cell.surface_area                = (np.pi*self.cell.height*self.cell.diameter) + (0.5*np.pi*self.cell.diameter**2)  # [m^2]
        self.cell.volume                      = np.pi*(0.5*self.cell.diameter)**2*self.cell.height 
        self.cell.density                     = self.cell.mass/self.cell.volume                                                  # [kg/m^3]  
                                                                                                                           
        self.cell.maximum_voltage             = 4.2                                                                              # [V]
        self.cell.nominal_capacity            = 3                                                                                 # [Amp-Hrs]
        self.cell.nominal_voltage             = 3.6                                                                              # [V] 
        self.cell.charging_voltage            = self.cell.nominal_voltage                                                    # [V] 
        self.cell.maximum_discharge_current   = 30                                                                               # [Amps] 
        self.cell.nominal_charging_current    = 3                                                                                # [Amps]  
        self.cell.maximum_charging_current    = 9                                                                                 # [Amps]    


        
        self.cell.watt_hour_rating            = self.cell.nominal_capacity  * self.cell.nominal_voltage                          # [Watt-hours]      
        self.cell.specific_energy             = self.cell.watt_hour_rating*Units.Wh/self.cell.mass                               # [J/kg]
        #self.cell.specific_power              = self.cell.specific_energy/self.cell.nominal_capacity
        #self.cell.specific_heat_capacity      = 1108                  # [W/kg], taken from NMC not accurate but not sure if needed   
        
        #self.cell.specific_power              = self.cell.maximum_discharge_current * self.cell.maximum_voltage / self.cell.mass # [W/kg] 
        #modelo = 
        self.cell.model = pybamm.lithium_ion.SPM( #initializing first model to have degradation
            {
                "SEI": "solvent-diffusion limited",
                "SEI porosity change": "true",
                "lithium plating": "partially reversible",
                "lithium plating porosity change": "true",  # alias for "SEI porosity change"
                "particle mechanics": ("swelling and cracking", "swelling only"),
                "SEI on cracks": "true",
                "loss of active material": "stress-driven",
                "calculate discharge energy": "true",  # for compatibility with older PyBaMM versions  
                #"thermal": "lumped"
            })
        self.cell.battery_parameters =  pybamm.ParameterValues("OKane2022")
        # self.cell.battery_parameters.update({"Electrode width [m]": 9.06108669e-01,
        #             "Negative electrode density [kg.m-3]": 2.23367642e+03,
        #             "Positive electrode density [kg.m-3]": 3.35604961e+03,
        #             'Bulk solvent concentration [mol.m-3]': 9.04026059e+02,
        #             'Cation transference number': 1.00108959e+00,
        #             'Nominal cell capacity [A.h]': 3.43328039e+00,
        #             'Negative electrode porosity': 9.85780579e-01,
        #             'Separator porosity':1.01419255e+00, 
        #             "Ambient temperature [K]": 296.15,
        #             'Reference temperature [K]': 296.15
        #             })                                            
        return  
    
    def energy_calc(self,state,bus,coolant_lines, t_idx, delta_t): 
        """
        """        
        stored_results_flag, stored_battery_tag =  compute_p30b_cell_performance(self,state,bus,coolant_lines, t_idx,delta_t) 
        
        return stored_results_flag, stored_battery_tag
    
    def reuse_stored_data(self,state,bus,stored_results_flag, stored_battery_tag):
        reuse_stored_p30b_cell_data(self,state,bus,stored_results_flag, stored_battery_tag)
        return 
    
    def update_battery_age(self,segment,battery_conditions, increment_battery_age_by_one_day = False):  
        """
        """        
        update_p30b_cell_age(self,segment,battery_conditions, increment_battery_age_by_one_day) 
        
        return  
