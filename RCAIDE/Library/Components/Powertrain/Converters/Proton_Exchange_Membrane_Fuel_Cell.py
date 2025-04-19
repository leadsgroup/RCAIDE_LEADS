# RCAIDE/Library/Compoments/Powertrain/Sources/Fuel_Cell/Proton_Exchange_Membrane_Fuel_Cell.py
# 
# 
# Created:  Dec 2024, M. Guidotti and M. Clarke 
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core       import Data
from .Generic_Fuel_Cell_Stack    import  Generic_Fuel_Cell_Stack
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Proton_Exchange_Membrane.compute_fuel_cell_performance import *
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Proton_Exchange_Membrane.append_fuel_cell_conditions   import *

# ----------------------------------------------------------------------------------------------------------------------
#  Proton_Exchange_Membrane_Fuel_Cell
# ---------------------------------------------------------------------------------------------------------------------- 
class Proton_Exchange_Membrane_Fuel_Cell(Generic_Fuel_Cell_Stack):
    """
    Proton Exchange Membrane Fuel Cell class 
    """ 
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None 

        Parameters:
        ----------
        type: string
            The type of PEM model to use, "LT" for low temperature, or "HT" for high temperature
        R: float 
            Universal gas constant (J / (mol*K))
        F: float 
            Faraday constant (C / mol)
        E_C: float 
            Activation energy of ORR (J)
        H2_molar_mass: float 
            Molar mass of H2 (kg/mol)
        O2_molar_mass: float 
            Molar mass of O2 (kg/mol)
        t_m: Length 
            The membrane thickness of the PEM cell 
        a_c: float 
            The catalyst specific area in cm^2 / mg Pt
        L_c: float 
            The catalyst platinum loading in mg Pt / cm^2
        A: Area 
            The membrane area 
        compressor_expander_module: compressor_expander_module 
            The compressor expander module of the air supply system 
        maximum_deg: float 
            The maximum voltage drop due to degradation of the fuel cell at EOL
        rated_current_density: float 
            The rated current density for pressure drop calculations (typically the current density at max power)
            Can be calculated using the evaluate_max_PD method
            Defaults to None 
        rated_power_density: float 
            The rated power density of the fuel cell 
            Defaults to None 
        rated_p_drop_fc: Pressure 
            The pressure drop at rated current density of the fuel cell cathode 
            Defaults to 0.240 bar 
        rated_p_drop_hum: Pressure 
            The pressure drop at rated current density of one side of the membrane humidifier
            Defaults to 0.025 bar
        gamma_para: float 
            The parasitic power draw divided by the gross cell power 
            Does not include compressor/ram-air heat exchanger coolant power
            Defaults to 0.03
        alpha: float 
            The charge transfer coefficient
            Used for both models, is between 0-1, but usually around 0.5 
            Defaults to 0.375 (matches reports and experimental data)
        gamma: float 
            The activation loss pressure dependency coefficient
            Generally 1 for HT-PEM 
            Defaults to 0.45 for LT-PEM
        lambda_eff: float 
            Fitting parameter used to calculate the conductivity of Nafion membranes 
            Only used if type is "LT"
            Values from 9-24 are reasonable 
            Defaults to 9.15 for fully humidifed air 
        c1: float 
            Membrane conductivity of HT PBI membrane at 100 C (ms/cm)
            Only used if type is "HT"
            defaults to 0.0435 for standard PBI membrane 
        c2: float 
            Membrane conductivity of HT PBI membrane at 200 C (ms/cm)
            Only used if type is "HT"
            defaults to 0.0636 for standard PBI membrane 
        i0ref: float 
            Reference current density for activation losses for HT PEM model (A / cm^2 Pt)
            Note that it is given per unit catalyst surface area, not per membrane area 
            defaults to 4e-8
        i0ref_P_ref: float 
            Pressure at which i0ref was determined  
            defaults to 1 bar 
        ioref_T_ref: float 
            Temperature at which i0ref was determined 
            defaults to 369.05 K
        i_lim_multiplier: float 
            The value which the limiting current is multiplied by for advanced PEM systems
        area_specific_mass: float 
            The mass per active membrane area of the fuel cell (kg/m2)
            
             
        References
        [1] Chilver-Stainer, James, et al. "Power output optimisation via arranging gas flow channels
            for low-temperature polymer electrolyte membrane fuel cell (PEMFC) for
            hydrogen-powered vehicles." Energies 16.9 (2023): 3722.
        
        """ 

        self.tag                                                               = 'pem_fuel_cell'        
        self.fuel_cell.Universal_gas_constant                                  = 8.31   # Universal gas constant (J / (mol*K))
        self.fuel_cell.Faraday_constant                                        = 96485  # Faraday constant (C / mol)
        self.fuel_cell.E_C                                                     = 66000  # Activation energy of ORR (J)
        self.fuel_cell.H2_molar_mass                                           = 2.0E-3 # Molar mass of H2 (kg/mol) 
        self.fuel_cell.O2_molar_mass                                           = 32E-3  # Molar mass of O2 (kg/mol) 
        self.fuel_cell.O2_mass_frac                                            = 0.233          
        self.fuel_cell.type                                                    = "LT"
        self.fuel_cell.specific_heat_capacity                                  = 903    # J/kg·K  # Chilver-Stainer, James, et al
        self.fuel_cell.t_m                                                     = 0.0024 #* Units.cm 
        self.fuel_cell.interface_area                                          = 50 #* Units.cm^2  # area of the fuel cell interface        
        self.fuel_cell.a_c                                                     = 98
        self.fuel_cell.L_c                                                     = 0.1 
        self.fuel_cell.compressor_expander_module                              = Data() 
        self.fuel_cell.compressor_expander_module.compressor_efficiency        = 0.71
        self.fuel_cell.compressor_expander_module.expander_efficiency          = 0.73 
        self.fuel_cell.compressor_expander_module.motor_efficiency             = 0.801025 
        self.fuel_cell.compressor_expander_module.generator_efficiency         = 1
        self.fuel_cell.compressor_expander_module.specific_weight              = None
        self.fuel_cell.compressor_expander_module.weight                       = 0 
        self.fuel_cell.maximum_deg                                             = 0
        self.fuel_cell.rated_current_density                                   = 0.1 # A/cm^2
        self.fuel_cell.rated_p_drop_fc                                         = 0.240 
        self.fuel_cell.rated_p_drop_hum                                        = 0.025
        self.fuel_cell.rated_H2_pressure                                       = 1   
        self.fuel_cell.rated_air_pressure                                      = 1  
        self.fuel_cell.gamma_para                                              = 0.03 
        self.fuel_cell.alpha                                                   = 0.375
        self.fuel_cell.gamma                                                   = 0.45
        self.fuel_cell.lambda_eff                                              = 24 
        self.fuel_cell.c1                                                      = 0.0435 
        self.fuel_cell.c2                                                      = 0.0636
        self.fuel_cell.stack_temperature                                       = 353.15 # Kelvin
        self.fuel_cell.air_excess_ratio                                        = 2
        self.fuel_cell.oxygen_relative_humidity                                = 1
        self.fuel_cell.i0ref                                                   = 9E-6
        self.fuel_cell.i0ref_P_ref                                             = 1 
        self.fuel_cell.i0ref_T_ref                                             = 353
        self.fuel_cell.current_density_limit_multiplier                        = 1
        self.fuel_cell.area_specific_mass                                      = 2.5 
        return 
        
    def energy_calc(self,state,bus,coolant_lines, t_idx, delta_t): 
        """Computes the state of the fuel cell battery cell. 
        """        
        if not (self.fuel_cell.type == "LT") or  (self.fuel_cell.type == "HT"): 
            raise ValueError('PEM type not supported, currently supported types are "LT" and "HT"')         
        
        stored_results_flag, stored_battery_tag = compute_fuel_cell_performance(self,state,bus,coolant_lines, t_idx,delta_t) 
        
        return stored_results_flag, stored_battery_tag
    
    def append_operating_conditions(self,segment,bus):  
        append_fuel_cell_conditions(self,segment,bus)  
        return
    
    def append_fuel_cell_segment_conditions(self,bus, conditions, segment):
        append_fuel_cell_segment_conditions(self,bus, conditions, segment)
        return 

    def reuse_stored_data(self,state,bus,stored_results_flag, stored_fuel_cell_tag):
        reuse_stored_fuel_cell_data(self,state,bus,stored_results_flag, stored_fuel_cell_tag)
        return       