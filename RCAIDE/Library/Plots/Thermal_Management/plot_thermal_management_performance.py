# RCAIDE/Library/Plots/Thermal_Management/plot_thermal_management_performance.py
# 
# 
# Created:  Sep 2024, S. Shekar
# ----------------------------------------------------------------------------------------------------------------------
#   plot_thermal_management_performance
# ----------------------------------------------------------------------------------------------------------------------   
def plot_thermal_management_performance(results,
                        save_figure   = False,
                        show_legend   = True,
                        file_type     =".png",
                        width         = 12,
                        height        = 7):
    """
    Checks and plots all components of a thermal management system.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
            - segments[i].analyses.vehicle.networks
                Network data containing:
                    - coolant_lines
                        List of coolant circuits with:
                            - battery_modules
                                List of battery thermal management systems
                            - heat_exchangers
                                List of heat exchanger components
                            - reservoirs
                                List of thermal reservoir components
                            - identical_battery_modules : bool
                                Flag indicating if batteries are identical
                            
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag to display component legends (default: True)
        
    file_type : str, optional
        File extension for saved figures (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 12)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    None
        Function generates and displays/saves plots for each component

    Notes
    -----
    Creates visualizations showing:
        * Battery thermal management system performance
        * Heat exchanger operating conditions
        * Reservoir thermal states
        * Overall system behavior
    
    For each component type:
        * Calls appropriate plotting function
        * Passes component-specific data
        * Maintains consistent formatting
        * Handles identical/unique components
    
    **Definitions**
    
    'Thermal Management System'
        Network of components managing heat transfer
    'Battery Module'
        Battery with thermal management system
    'Heat Exchanger'
        Component transferring heat between fluids
    'Reservoir'
        Component storing thermal energy
    
    See Also
    --------
    RCAIDE.Library.Plots.Thermal_Management.plot_air_cooled_conditions : Air-cooled system analysis
    RCAIDE.Library.Plots.Thermal_Management.plot_cross_flow_heat_exchanger_conditions : Heat exchanger analysis
    RCAIDE.Library.Plots.Thermal_Management.plot_reservoir_conditions : Reservoir analysis
    """     
    
    for network in  results.segments[0].analyses.vehicle.networks:
        for coolant_line in  network.coolant_lines:
            for tag, item in  coolant_line.items():
                if coolant_line.identical_battery_modules:
                    if tag == 'battery_modules':
                            for i, battery in enumerate(item):
                                for btms in  (battery):
                                    if i ==  0:
                                        btms.plot_operating_conditions(results,coolant_line,save_figure,show_legend,btms.tag,file_type,width, height)
                else:
                    for _, battery in enumerate(item):
                            for btms in  (battery):
                                btms.plot_operating_conditions(results,coolant_line,save_figure,show_legend,btms.tag,file_type,width, height)
                if tag == 'heat_exchangers':
                    for heat_exchanger in  item:
                        heat_exchanger.plot_operating_conditions(results,coolant_line,save_figure,show_legend,heat_exchanger.tag,file_type,width, height)
                if tag == 'reservoirs':
                    for reservoir in  item:
                        reservoir.plot_operating_conditions(results,coolant_line,save_figure,show_legend,reservoir.tag,file_type,width, height)             
    return