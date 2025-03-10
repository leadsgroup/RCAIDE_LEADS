# RCAIDE/Library/Plots/Geometry/plot_rotor.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units

from plotly.subplots import make_subplots
import pandas as pd
import plotly.graph_objects as go

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------  
def plot_rotor(prop, face_color = 'red', edge_color = 'black', show_figure = True, 
               save_figure = False, save_filename = "Propeller_Geometry", file_type = ".png"):
    """
    Creates a 2D visualization of rotor/propeller geometry distributions.

    Parameters
    ----------
    prop : Propeller
        RCAIDE propeller/rotor data structure containing geometry information
        
    face_color : str, optional
        Color specification for plot faces (default: 'red')
        
    edge_color : str, optional
        Color specification for plot edges (default: 'black')
        
    show_figure : bool, optional
        Flag to display the figure (default: True)
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Propeller_Geometry")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")

    Returns
    -------
    fig : plotly.graph_objects.Figure

    Notes
    -----
    Creates a four-panel plot showing:
        1. Twist angle vs radial position
        2. Chord length vs radial position
        3. Maximum thickness vs radial position
        4. Mid-chord alignment vs radial position
    
    **Major Assumptions**
    
    * Distributions are defined at consistent radial stations
    * Twist is in degrees
    * Chord and thickness are in meters
    * Mid-chord alignment is in meters
    
    **Definitions**
    
    'Twist'
        Local blade angle relative to rotation plane
    'Chord'
        Local blade section length
    'Thickness'
        Maximum thickness of local blade section
    'Mid-chord Alignment'
        Offset of section mid-chord from reference line
    """
    # Initialize figure
    fig = make_subplots(rows=2, cols=2)

    df1 = pd.DataFrame(dict(x=prop.radius_distribution, y=prop.twist_distribution/Units.degrees))
    df2 = pd.DataFrame(dict(x=prop.radius_distribution, y=prop.chord_distribution))
    df3 = pd.DataFrame(dict(x=prop.radius_distribution, y=prop.max_thickness_distribution))
    df4 = pd.DataFrame(dict(x=prop.radius_distribution, y=prop.mid_chord_alignment))

    fig.append_trace(go.Line(df1), row=1, col=1)
    fig.append_trace(go.Line(df2), row=1, col=2)
    fig.append_trace(go.Line(df3), row=2, col=1)
    fig.append_trace(go.Line(df4), row=2, col=2)

    fig.update_xaxes(title_text="Radial Station", row=1, col=1)
    fig.update_yaxes(title_text="Twist (Deg)", row=1, col=1)
    fig.update_xaxes(title_text="Radial Station", row=1, col=2)
    fig.update_yaxes(title_text="Chord (m)", row=1, col=2)
    fig.update_xaxes(title_text="Radial Station", row=2, col=1)
    fig.update_yaxes(title_text="Thickness (m)", row=2, col=1)
    fig.update_xaxes(title_text="Radial Station", row=2, col=2)
    fig.update_yaxes(title_text="Mid Chord Alignment (m)", row=2, col=2)

    fig.update_layout(title_text="Propeller Geometry", height=700, showlegend=False)

    if save_figure:
        fig.write_image(save_filename + '_2D' + file_type)
    
    if show_figure:
        fig.write_html( save_filename + '.html', auto_open=True)

    return fig
