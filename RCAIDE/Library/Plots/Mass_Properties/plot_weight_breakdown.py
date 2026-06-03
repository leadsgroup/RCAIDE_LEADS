# RCAIDE/Library/Plots/Mass_Properties/plot_weight_breakdown.py
#
#
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_weight_breakdown(vehicle,
                            save_figure    = False,
                            show_legend    = True,
                            save_filename  = "Weight_Breakdown",
                            aircraft_name  = None,
                            file_type      = ".png",
                            width          = 10, height = 7.2):


    """
    Creates a pie chart visualization of aircraft weight breakdown.

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
        Flag to display plot (default: True)

    show_legend : bool, optional
        Flag to display weight legend (default: True)

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
    fig : matplotlib.figure.Figure
        Handle to the generated figure

    Notes
    -----
    Creates visualization showing:
        * Top-level weight breakdown as pie slices
        * Weight percentages per component
        * Total mass in title

    **Major Assumptions**
        * All weights are positive
        * Hierarchy is properly structured

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

    breakdown = vehicle.mass_properties.weight_breakdown

    # Tags to skip entirely
    SKIP_TOP = {'zero_fuel_weight', 'max_takeoff', 'payload'}
    # Sub-groups inside 'empty' to show as single slices (by total)
    COLLAPSE = {'structural', 'propulsion', 'systems'}

    labels = []
    values = []

    def _leaf_sum(data_obj):
        total = 0.0
        for k, v in data_obj.items():
            if k == 'total':
                continue
            if isinstance(v, RCAIDE.Framework.Core.Data):
                total += _leaf_sum(v)
            else:
                try:
                    f = float(v)
                    if np.isfinite(f) and f > 0:
                        total += f
                except (TypeError, ValueError):
                    pass
        return total

    for tag, item in breakdown.items():
        if tag in SKIP_TOP:
            continue

        if tag == 'empty' and isinstance(item, RCAIDE.Framework.Core.Data):
            for sub_tag, sub_item in item.items():
                if sub_tag == 'total':
                    continue
                if sub_tag in COLLAPSE and isinstance(sub_item, RCAIDE.Framework.Core.Data):
                    val = sub_item.total if (hasattr(sub_item, 'total') and sub_item.total) else _leaf_sum(sub_item)
                    if val > 0:
                        labels.append(sub_tag.replace('_', ' ').title())
                        values.append(val)
        elif isinstance(item, RCAIDE.Framework.Core.Data):
            val = item.total if (hasattr(item, 'total') and item.total) else _leaf_sum(item)
            if val > 0:
                labels.append(tag.replace('_', ' ').title())
                values.append(val)
        else:
            try:
                val = float(item)
            except (TypeError, ValueError):
                continue
            if np.isfinite(val) and val > 0:
                labels.append(tag.replace('_', ' ').title())
                values.append(val)

    values = np.array(values, dtype=float)
    total_lbs = np.sum(values) / 0.453592  # kg to lbs

    palette = ['#5B9BD5', '#C0504D', '#4BACC6', '#8064A2', '#F79646',
               '#9BBB59', '#4F81BD', '#1F497D', '#E46C0A', '#76923C', '#17375E', '#833C00']
    colors = [palette[i % len(palette)] for i in range(len(labels))]

    fig, ax = plt.subplots(figsize=(width, height))

    wedges, _, autotexts = ax.pie(
        values,
        labels=None,
        colors=colors,
        autopct=lambda pct: f'({pct:.2f}%)' if pct > 0.5 else '',
        startangle=90,
        pctdistance=0.75,
        wedgeprops=dict(linewidth=0.8, edgecolor='white'),
    )

    # Style autotext (percentage labels inside slices)
    for at in autotexts:
        at.set_fontsize(14)
        at.set_color('white')

    # Place labels pointing outward from each wedge
    for wedge, label in zip(wedges, labels):
        angle = (wedge.theta2 + wedge.theta1) / 2
        x = np.cos(np.radians(angle))
        y = np.sin(np.radians(angle))
        ha = 'left' if x >= 0 else 'right'
        ax.annotate(
            label,
            xy=(x * 1.02, y * 1.02),
            xytext=(x * 1.25, y * 1.25),
            ha=ha,
            va='center',
            fontsize=15,
            arrowprops=dict(arrowstyle='-', color='gray', lw=0.6),
        )

    title = f'All (OEW) Mass Collection: Mass = {total_lbs:,.0f} lbs'
    if aircraft_name:
        title = f'{aircraft_name} — ' + title
    # ax.set_title(title, fontsize=15, fontweight='bold', pad=20)

    fig.tight_layout()

    if save_figure:
        fig.savefig(save_filename + file_type, dpi=150, bbox_inches='tight')


    return fig
