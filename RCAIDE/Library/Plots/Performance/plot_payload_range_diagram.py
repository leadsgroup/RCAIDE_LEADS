# RCAIDE/Library/Plots/Performance/plot_payload_range_diagram.py
#
# Created: Feb 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style

# Package imports
from matplotlib import pyplot as plt

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_payload_range_diagram(payload_range_result,
                               save_figure   = False,
                               show_legend   = False,
                               save_filename = "Payload_Range_Diagram",
                               file_type     = ".png",
                               width         = 8,
                               height        = 6):
    """Plot a payload-range diagram.

    Parameters
    ----------
    payload_range_result : Data
        Must contain:
            .range    [m]   – array of range values
            .payload  [kg]  – array of payload values
    save_figure   : bool, optional
        Write the figure to disk.  Default False.
    show_legend   : bool, optional
        Display a legend.  Default False.
    save_filename : str, optional
        Base filename (no extension) used for the figure window title and
        the output file when save_figure is True.
    file_type     : str, optional
        Image format extension, e.g. ".png" or ".pdf".  Default ".png".
    width, height : float, optional
        Figure size in inches.  Default 8 × 6.

    Returns
    -------
    fig : matplotlib.figure.Figure
    """
    ps = plot_style()

    plt.rcParams.update({
        'axes.labelsize':  ps.axis_font_size,
        'xtick.labelsize': ps.axis_font_size,
        'ytick.labelsize': ps.axis_font_size,
        'axes.titlesize':  ps.title_font_size,
    })

    fig  = plt.figure(save_filename)
    fig.set_size_inches(width, height)
    axis = fig.add_subplot(1, 1, 1)

    axis.plot(payload_range_result.range / Units.nmi,
              payload_range_result.payload,
              color=ps.color, linewidth=ps.line_width)

    axis.set_xlabel('Range (nmi)')
    axis.set_ylabel('Payload (kg)')
    axis.set_title('Payload-Range Diagram')
    set_axes(axis)

    if show_legend:
        axis.legend()

    fig.tight_layout()

    if save_figure:
        fig.savefig(save_filename + file_type)

    return fig
