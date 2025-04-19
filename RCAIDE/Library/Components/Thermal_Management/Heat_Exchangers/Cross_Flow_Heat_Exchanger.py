# RCAIDE/Library/Components/Thermal_Management/Heat_Exchangers/Cross_flow_Heat_Exchanger.py
# 
# Created:  Apr 2024, S. Shekar 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports
from RCAIDE.Framework.Core                                                                import Data, Units 
from RCAIDE.Library.Components                                                            import Component  
from RCAIDE.Library.Attributes.Coolants.Glycol_Water                                      import Glycol_Water  
from RCAIDE.Library.Attributes.Gases                                                      import Air
from RCAIDE.Library.Methods.Thermal_Management.Heat_Exchangers.Cross_Flow_Heat_Exchanger  import  cross_flow_hex_rating_model, append_cross_flow_heat_exchanger_conditions, append_cross_flow_hex_segment_conditions
from RCAIDE.Library.Plots.Thermal_Management.plot_cross_flow_heat_exchanger_conditions    import plot_cross_flow_heat_exchanger_conditions 

import os
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Cross Flow Heat Exchanger 
# ----------------------------------------------------------------------------------------------------------------------  
class Cross_Flow_Heat_Exchanger(Component):
    """
    A class representing a cross-flow heat exchanger with strip fins for thermal 
    management systems.

    Attributes
    ----------
    tag : str
        Unique identifier for the heat exchanger, defaults to 'cross_flow_heat_exchanger'
        
    coolant : Glycol_Water
        Coolant properties for calculations, defaults to Glycol_Water()
        
    air : Air
        Air properties for calculations, defaults to Air()
        
    design_heat_removed : float
        Design heat removal capacity, defaults to 0.0
        
    minimum_air_speed : float
        Minimum air velocity through exchanger, defaults to 105 knots
        
    heat_exchanger_efficiency : float
        Overall heat transfer effectiveness, defaults to 0.8381
        
    density : float
        Material density in kg/m^3, defaults to 8440
        
    thermal_conductivity : float
        Material thermal conductivity in W/m.K, defaults to 121
        
    specific_heat_capacity : float
        Material specific heat in J/kg.K, defaults to 871
        
    stack_length : float
        Length of heat exchanger core, defaults to 1
        
    stack_width : float
        Width of heat exchanger core, defaults to 1
        
    stack_height : float
        Height of heat exchanger core, defaults to 1
        
    t_w : float
        Plate thickness in meters, defaults to 5e-4
        
    t_f : float
        Fin thickness in meters, defaults to 1e-4
        
    fin_spacing_cold : float
        Cold side fin spacing in meters, defaults to 2.54e-3
        
    fin_spacing_hot : float
        Hot side fin spacing in meters, defaults to 2.54e-3
        
    fin_metal_thickness_hot : float
        Hot side fin metal thickness in meters, defaults to 0.102e-3
        
    fin_metal_thickness_cold : float
        Cold side fin metal thickness in meters, defaults to 0.102e-3
        
    fin_exposed_strip_edge_hot : float
        Hot side exposed strip edge in meters, defaults to 3.175e-3
        
    fin_exposed_strip_edge_cold : float
        Cold side exposed strip edge in meters, defaults to 3.175e-3
        
    fin_area_density_hot : float
        Hot side fin area density in m^2/m^3, defaults to 2254
        
    fin_area_density_cold : float
        Cold side fin area density in m^2/m^3, defaults to 2254
        
    finned_area_to_total_area_hot : float
        Hot side fin area ratio, defaults to 0.785
        
    finned_area_to_total_area_cold : float
        Cold side fin area ratio, defaults to 0.785
        
    coolant_hydraulic_diameter : float
        Coolant passage hydraulic diameter in meters, defaults to 1.54e-3
        
    air_hydraulic_diameter : float
        Air passage hydraulic diameter in meters, defaults to 1.54e-3
        
    fin_conductivity : float
        Fin thermal conductivity in W/m.K, defaults to 121
        
    wall_conductivity : float
        Wall thermal conductivity in W/m.K, defaults to 121

    Notes
    -----
    The cross-flow heat exchanger uses strip fins and is based on the 1/8-19.86 
    surface designation. The design includes:
    
    * Liquid coolant passages with strip fins
    * Air passages with strip fins
    * Counter-flow arrangement for maximum effectiveness

    **Assumptions**
    
    * Coolant is 50-50 Glycol Water mixture
    * Constant entrance and exit loss coefficients
    * Surface designation of 1/8-19.86 with strip fins

    **Definitions**

    'Strip Fin'
        Interrupted fin design that enhances heat transfer while reducing pressure drop
        
    'Hydraulic Diameter'
        Characteristic dimension for internal flow calculations

    References
    ----------
    [1] Kays, W.M. and London, A.L. (1998) Compact Heat Exchangers. 3rd Edition, 
        McGraw-Hill, New York.

    See Also
    --------
    RCAIDE.Library.Components.Thermal_Management.Heat_Exchangers.Cryogenic_Heat_Exchanger
        Alternative heat exchanger for cryogenic applications
    """

    def __defaults__(self):
        """
        Sets default values for the cross-flow heat exchanger attributes.
        """         
        self.tag                                                    = 'cross_flow_heat_exchanger'
        self.coolant                                                = Glycol_Water() 
        self.air                                                    = Air() 
        self.design_heat_removed                                    = 0.0
        self.minimum_air_speed                                      = 105 * Units.knots 
        
        # heat exchanger: thermophysical properties                        
        self.heat_exchanger_efficiency                              = 0.8381
        self.density                                                = 8440   # kg/m^3
        self.thermal_conductivity                                   = 121    # W/m.K
        self.specific_heat_capacity                                 = 871    # J/kg.K 


        self.stack_length                                           = 1
        self.stack_width                                            = 1
        self.stack_height                                           = 1
                         
        self.design_air_flow_rate                                   = None 
        self.design_air_inlet_pressure                              = None 
        self.design_coolant_inlet_pressure                          = None 

        # heat exchanger: geometric properties      
        # Plate thickness                       
        self.t_w                                                    = 5e-4   # m

        # Fin Thickness
        self.t_f                                                    = 1e-4   # m

        self.fin_spacing_cold                                       = 2.54e-3 #m
        self.fin_spacing_hot                                        = 2.54e-3 #m

        # Fin metal thickness
        self.fin_metal_thickness_hot                                = 0.102e-3 #m
        self.fin_metal_thickness_cold                               = 0.102e-3 #m

        # Strip edge exposed 
        self.fin_exposed_strip_edge_hot                             = 3.175e-3 #m
        self.fin_exposed_strip_edge_cold                            = 3.175e-3 #m

        # Finned area density 
        self.fin_area_density_hot                                   = 2254 # m^2/m^3 
        self.fin_area_density_cold                                  = 2254 # m^2/m^3 

        # Ratio of finned area to total area 
        self.finned_area_to_total_area_hot                          = 0.785
        self.finned_area_to_total_area_cold                         = 0.785

        # Hydraullic Diameter
        self.coolant_hydraulic_diameter                             = 1.54e-3  #m
        self.air_hydraulic_diameter                                 = 1.54e-3  #m

        # Fin and wall Conductivity 
        self.fin_conductivity                                        = 121    # W/m.K
        self.wall_conductivity                                       = 121    # W/m.K        

        # Fan
        self.fan                                                    = Data() # Replace with RCAIDE Fan 
        self.fan.efficiency                                         = 0.7
        self.fan.active                                             = True 
        
        # Pump 
        self.pump                                                   = Data() # Replace with RCAIDE Fan 
        self.pump.efficiency                                        = 0.7 

        # Operating Conditions 
        self.percent_operation                                      = 1.0
        self.atmospheric_air_inlet_to_outlet_area_ratio             = 0.5 
        self.duct_losses                                            = 0.98 # Inlet, Duct and Nozzle Losses
        self.fan_operation                                          = True


        # Limiting Pressure Drop
        self.pressure_drop_hot                                      = 9.05e3 #Pa 
        self.pressure_drop_cold                                     = 8.79e3 #Pa 


        # Enterance and Exit pressure loss coefficients  
        self.kc_values                                              = load_kc_values()
        self.ke_values                                              = load_ke_values()
        return  

    def append_operating_conditions(self, segment, coolant_line):
        """
        Adds operating conditions for the heat exchanger to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        coolant_line : Data
            Cooling system flow path information
        """
        append_cross_flow_heat_exchanger_conditions(self, segment, coolant_line)
        return
  
    def append_segment_conditions(self, segment, bus, coolant_line, conditions):
        """
        Adds specific segment conditions to the heat exchanger analysis.

        Parameters
        ----------
        segment : Data
            Mission segment being analyzed
        bus : Data
            Electrical bus data
        coolant_line : Data
            Cooling system flow path information
        conditions : Data
            Operating conditions for the segment
        """
        append_cross_flow_hex_segment_conditions(self, segment, bus, coolant_line, conditions)
        return
       
    def compute_heat_exchanger_performance(self, state, bus, coolant_line, delta_t, t_idx):
        """
        Calculates thermal performance of the heat exchanger.

        Parameters
        ----------
        state : Data
            Current system state
        bus : Data
            Electrical bus data
        coolant_line : Data
            Cooling system flow path information
        delta_t : float
            Time step size
        t_idx : int
            Time index in the simulation
        """
        cross_flow_hex_rating_model(self, state, bus, coolant_line, delta_t, t_idx)
        return

    def plot_operating_conditions(self, results, coolant_line, save_filename, save_figure, 
                                show_legend, file_type, width, height):
        """
        Creates visualization plots of the heat exchanger operating conditions.

        Parameters
        ----------
        results : Data
            Simulation results data
        coolant_line : Data
            Cooling system flow path information
        save_filename : str
            Path for saving the plot
        save_figure : bool
            Flag to save the figure
        show_legend : bool
            Flag to display plot legend
        file_type : str
            Output file format
        width : float
            Plot width
        height : float
            Plot height
        """
        plot_cross_flow_heat_exchanger_conditions(self, results, coolant_line, save_filename,
                                                save_figure, show_legend, file_type, width, height)     
        return    

    def load_kc_values(): 
        """
        Loads entrance loss coefficient data from file.

        Returns
        -------
        ndarray
            Array of entrance loss coefficients
        """
        ospath    = os.path.abspath(__file__)
        separator = os.path.sep
        rel_path  = os.path.dirname(ospath) + separator   
        x         = np.loadtxt(rel_path + 'rectangular_passage_Kc.csv', dtype=float, 
                             delimiter=',', comments='Kc') 
        return x 
    
    def load_ke_values():  
        """
        Loads exit loss coefficient data from file.

        Returns
        -------
        ndarray
            Array of exit loss coefficients
        """
        ospath    = os.path.abspath(__file__)
        separator = os.path.sep
        rel_path  = os.path.dirname(ospath) + separator 
        x         = np.loadtxt(rel_path +'rectangular_passage_Ke.csv', dtype=float, 
                             delimiter=',', comments='Ke')
        return x 