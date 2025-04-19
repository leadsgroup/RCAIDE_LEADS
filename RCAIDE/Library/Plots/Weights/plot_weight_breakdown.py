# RCAIDE/Library/Plots/Weights/plot_weight_breakdown.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE 
import numpy as np     
import plotly.express as px 
import pandas as pd

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
def plot_weight_breakdown(vehicle,
                            save_figure    = False,
                            show_figure    = True, 
                            show_legend    = True,
                            SI_Units       = True,
                            save_filename  = "Weight_Breakdown",
                            aircraft_name  = None,
                            file_type      = ".png",
                            width          = 10, height = 7.2): 
  

    """
    Creates an interactive sunburst visualization of aircraft weight breakdown.

    Parameters
    ----------
    vehicle : Vehicle
        RCAIDE vehicle data structure containing:
        
        * weight_breakdown : Data
            Hierarchical weight data with structure:
                * zero_fuel_weight : float
                    Aircraft weight without fuel
                * max_takeoff : float
                    Maximum takeoff weight
                * systems : Data
                    System weights containing:
                        * total : float
                            Total systems weight
                        * [system_name] : float/Data
                            Individual system weights/subcomponents
                * fuel : float
                    Total fuel weight
                
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_figure : bool, optional
        Flag to display interactive plot (default: True)
        
    show_legend : bool, optional
        Flag to display weight legend (default: True)
        
    SI_Units : bool, optional
        If True, display weights in kg
        If False, display weights in lbs (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Weight_Breakdown")
        
    aircraft_name : str, optional
        Name to display in plot title (default: None)
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 10)
        
    height : float, optional
        Figure height in inches (default: 7.2)

    Returns
    -------
    fig : plotly.graph_objects.Figure
        Handle to the generated figure containing:

    Notes
    -----
    Creates visualization showing:
        * Hierarchical weight breakdown
        * Weight percentages
        * Multi-level weight relationships
        * Interactive exploration
    
    **Major Assumptions**
        * All weights are positive
        * Hierarchy is properly structured
        * No duplicate component names
        * Subcomponent weights sum to totals
    
    **Definitions**
    
    'Weight Breakdown'
        Hierarchical decomposition of vehicle mass
    'Weight Fraction'
        Component weight divided by total weight
    'Zero Fuel Weight'
        Aircraft weight excluding fuel
    'Maximum Takeoff Weight'
        Maximum allowable total weight
    
    See Also
    --------
    RCAIDE.Library.Analysis.Weights : Weight analysis tools
    """

    breakdown =  vehicle.weight_breakdown     
    
    level_1 = []
    level_2 = []
    level_3 = []
    values  = []
      
    for tag ,  item in  breakdown.items():
        if tag == 'zero_fuel_weight' or   tag == 'max_takeoff':
            pass
        else:
            if type(item)  == RCAIDE.Framework.Core.Data: 
                for  sub_tag  , sub_item in item.items():
                    if type(sub_item) == RCAIDE.Framework.Core.Data: 
                        for sub_sub_tag ,  sub_sub_item in sub_item.items():
                            if sub_sub_tag != 'total':
                                level_1.append(tag.replace("_", " "))
                                level_2.append(sub_tag.replace("_", " "))
                                level_3.append(sub_sub_tag.replace("_", " "))
                                values.append(sub_sub_item )
                    elif sub_tag != 'total':
                        level_1.append(tag.replace("_", " "))
                        level_2.append(sub_tag.replace("_", " "))
                        level_3.append(np.nan)
                        values.append(sub_item) 
            elif tag == 'fuel':
                level_1.append(tag.replace("_", " "))
                level_2.append(np.nan)
                level_3.append(np.nan)
                values.append(sub_item) 
                  
    df = pd.DataFrame(
        dict(level_1=level_1, level_2=level_2, level_3=level_3, values=values)
    ) 
    fig = px.sunburst(df,
                      path=['level_1', 'level_2', 'level_3'], 
                      values='values',  
                      color_discrete_sequence=px.colors.qualitative.G10)
    
    # Add a dummy inner layer for the hole
    fig.update_traces(
        textfont=dict(size=20), 
        insidetextorientation='horizontal', 
        textinfo='label+percent entry', 
        marker=dict(colors=['rgba(0,0,0,0)'] + px.colors.qualitative.G10)
    )
    fig.update_layout( 
    uniformtext=dict(minsize=12, mode='hide'),
    )
 
    if save_figure:
        fig.write_image(save_filename + file_type)
    
    if show_figure:
        fig.write_html( save_filename + '.html', auto_open=True)        
    return  fig 
