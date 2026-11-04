# RCAIDE/Library/Plots/Emissions/plot_emission_species_masses
#
#
# Created:  Jul 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_emission_species_masses(results,
                    save_figure = False,
                    show_legend = True,
                    save_filename = "Emission_Species_Masses" ,
                    file_type = ".png",
                    width = 8, height = 5):
    """
    Generate a stacked area plot showing cumulative emissions mass by species over the mission.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
        results.segments[i].conditions.emissions.mass with fields:
            - CO2 : array, carbon dioxide mass [kg]
            - NOx : array, nitrogen oxide mass [kg]
            - H2O : array, water vapor mass [kg]
            - CO  : array, carbon monoxide mass [kg]

    save_figure : bool, optional
        Save figure to file if True, default False

    show_legend : bool, optional
        Display species legend if True, default True

    save_filename : str, optional
        Name for saved figure file, default "Emission_Species_Masses"

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 8

    height : float, optional
        Figure height in inches, default 5

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure showing stacked cumulative species emissions mass over time
    """

    ps = plot_style()

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)

    fig = plt.figure(save_filename)
    fig.set_size_inches(width, height)

    species_colors = {
        'CO2': '#d62728',
        'CO' : '#ff7f0e',
        'NOx': '#2ca02c',
        'H2O': '#1f77b4',
    }
    species_labels = {
        'CO2': r'$CO_2$',
        'CO' : r'$CO$',
        'NOx': r'$NO_x$',
        'H2O': r'$H_2O$',
    }

    time_all = []
    mass     = {k: [] for k in ['CO2', 'CO', 'NOx', 'H2O']}
    running  = {k: 0.0 for k in ['CO2', 'CO', 'NOx', 'H2O']}
    for seg in results.segments:
        t = seg.conditions.frames.inertial.time[:, 0] / Units.min
        time_all.append(t)
        for k, attr in [('CO2', 'CO2'), ('CO', 'CO'), ('NOx', 'NOx'), ('H2O', 'H2O')]:
            if hasattr(seg.conditions.emissions.mass, attr):
                seg_vals = getattr(seg.conditions.emissions.mass, attr)[:, 0] / 1E3
            else:
                # ground segment (e.g. Dormancy, Refuel): no combustion, zero emissions
                seg_vals = np.zeros_like(t)
            mass[k].append(seg_vals + running[k])
            running[k] += seg_vals[-1]

    time_all = np.concatenate(time_all)
    for k in mass:
        mass[k] = np.concatenate(mass[k])

    axis_1 = plt.subplot(1, 1, 1)
    bottom = np.zeros_like(time_all)
    for species in ['CO2', 'CO', 'NOx', 'H2O']:
        axis_1.fill_between(time_all, bottom, bottom + mass[species],
                            label=species_labels[species],
                            color=species_colors[species], alpha=0.85)
        bottom += mass[species]

    axis_1.set_ylabel(r'Species Mass (Metric Tons)')
    axis_1.set_xlabel(r'Time (mins)')
    set_axes(axis_1)

    if show_legend:
        leg = fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)
        leg.set_title('Emission Species', prop={'size': ps.legend_font_size, 'weight': 'heavy'})

    fig.tight_layout()
    fig.suptitle('Emission Species Masses')
    fig.subplots_adjust(top=0.8)

    if save_figure:
        plt.savefig(save_filename + file_type)
    return fig
