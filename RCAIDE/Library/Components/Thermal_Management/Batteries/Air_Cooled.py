# RCAIDE/Library/Components/Thermal_Management/Batteries/Air_Cooled.py
# 
# Created:  Jul 2023, M. Clarke
# Modified: Aug 2024, S. Shekar 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Components import Component  
from RCAIDE.Library.Methods.Thermal_Management.Batteries.Air_Cooled import append_air_cooled_conditions, air_cooled_performance, append_air_cooled_segment_conditions
from RCAIDE.Library.Attributes.Gases import Air
from RCAIDE.Library.Plots.Thermal_Management.plot_air_cooled_conditions import plot_air_cooled_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  Air_Cooled
# ----------------------------------------------------------------------------------------------------------------------
class Air_Cooled(Component):
    """
    A class representing an air-cooled thermal management system for battery packs using 
    direct convection cooling.

    Attributes
    ----------
    tag : str
        Unique identifier for the cooling system, defaults to 'air_cooled_heat_acquisition'
        
    cooling_fluid : Air
        Air properties for cooling calculations, defaults to standard Air()
        
    cooling_fluid.flowspeed : float
        Air flow velocity through the cooling channels, defaults to 0.01
        
    convective_heat_transfer_coefficient : float
        Heat transfer coefficient between air and battery surface, defaults to 35.0
        
    heat_transfer_efficiency : float
        Overall efficiency of the heat transfer process, defaults to 1.0

    Notes
    -----
    The air-cooled system provides direct convective cooling for battery thermal 
    management. It includes functionality for:
    
    * Operating condition analysis
    * Thermal performance calculation
    * Condition plotting and visualization

    **Definitions**

    'Convective Heat Transfer Coefficient'
        Rate of heat transfer between the battery surface and cooling air
        
    'Heat Transfer Efficiency'
        Ratio of actual to theoretical maximum heat transfer

    See Also
    --------
    RCAIDE.Library.Components.Thermal_Management.Batteries.Liquid_Cooled_Wavy_Channel
        Alternative cooling approach using liquid coolant
    RCAIDE.Library.Components.Thermal_Management.Batteries.Cryocooler
        Alternative cooling approach for extreme temperatures
    """
    
    def __defaults__(self):
        """
        Sets default values for the air cooling system attributes.
        """                 
        self.tag                                  = 'air_cooled_heat_acquisition'
        self.cooling_fluid                        = Air()    
        self.cooling_fluid.flowspeed              = 0.01                                          
        self.convective_heat_transfer_coefficient = 35.     
        self.heat_transfer_efficiency             = 1.0      
   
    def append_operating_conditions(self, segment, coolant_line):
        """
        Adds operating conditions for the cooling system to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        coolant_line : Data
            Cooling system flow path information
        """
        append_air_cooled_conditions(self, segment, coolant_line)
        return

    def append_segment_conditions(self, segment, coolant_line, conditions):
        """
        Adds specific segment conditions to the cooling system analysis.

        Parameters
        ----------
        segment : Data
            Mission segment being analyzed
        coolant_line : Data
            Cooling system flow path information
        conditions : Data
            Operating conditions for the segment
        """
        append_air_cooled_segment_conditions(self, segment, coolant_line, conditions)
        return
    
    def compute_thermal_performance(self, battery, bus, coolant_line, Q_heat_gen, T_cell, state, delta_t, t_idx): 
        """
        Calculates thermal performance of the air cooling system.

        Parameters
        ----------
        battery : Data
            Battery pack information
        bus : Data
            Electrical bus data
        coolant_line : Data
            Cooling system flow path information
        Q_heat_gen : float
            Heat generation rate
        T_cell : float
            Current cell temperature
        state : Data
            Current system state
        delta_t : float
            Time step size
        t_idx : int
            Time index in the simulation

        Returns
        -------
        float
            Updated battery temperature
        """
        T_battery_current = air_cooled_performance(self, battery, bus, coolant_line, 
                                                 Q_heat_gen, T_cell, state, delta_t, t_idx)
        return T_battery_current

    def plot_operating_conditions(self, results, coolant_line, save_filename, save_figure, 
                                show_legend, file_type, width, height):
        """
        Creates visualization plots of the cooling system operating conditions.

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
        plot_air_cooled_conditions(self, results, coolant_line, save_filename, 
                                 save_figure, show_legend, file_type, width, height)
        return
        
