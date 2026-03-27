## @ingroup Library-Plots-Performance-Common
# RCAIDE/Library/Plots/Performance/Common/set_axes.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------    
## @ingroup Library-Plots-Performance-Common
def set_axes(axes):
    """
    Apply a standardized RCAIDE formatting style to matplotlib plot axes.

    Parameters
    ----------
    axes : matplotlib.axes.Axes
        Axes object to be formatted with RCAIDE standard style

    Returns
    -------
    None

    Notes
    -----
    Applies the following formatting:
        - Minor tick marks enabled
        - Major grid lines: solid grey, width 0.5
        - Minor grid lines: dotted grey, width 0.5
        - Grid lines enabled
        - Scientific notation disabled for y-axis
        - Axis offset disabled for y-axis

    This function ensures consistent axis appearance across all RCAIDE plots
    and should be called after plotting data but before displaying the figure.

    **Definitions**

    'Major Grid'
        Primary grid lines at major tick marks
    
    'Minor Grid'
        Secondary grid lines at minor tick marks
    
    'Tick Marks'
        Small lines indicating axis scale divisions

    See Also
    --------
    RCAIDE.Library.Plots.Common.plot_style : Complementary plot styling
    """

    axes.minorticks_on()
    axes.grid(which='major', linestyle='-', linewidth=0.5, color='grey')
    axes.grid(which='minor', linestyle=':', linewidth=0.5, color='grey')
    axes.grid(True)
    yfmt = axes.get_yaxis().get_major_formatter()
    if hasattr(yfmt, "set_scientific"):
        yfmt.set_scientific(False)
    if hasattr(yfmt, "set_useOffset"):
        yfmt.set_useOffset(False)

    return
