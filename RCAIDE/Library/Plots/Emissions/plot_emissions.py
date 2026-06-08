# RCAIDE/Library/Plots/Emissions/plot_emissions
#
#
# Created:  Jul 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_emissions(results,
                    save_figure = False,
                    show_legend = True,
                    save_filename = "Emissions" ,
                    file_type = ".png",
                    width = 11, height = 5):
    """
    Generate plots showing emissions mass breakdown by species and emission indexes over the mission.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
        results.segments[i].conditions.emissions.total with fields:
            - CO2 : array
                Carbon dioxide emissions [kg]
            - NOx : array
                Nitrogen oxide emissions [kg]
            - H2O : array
                Water vapor emissions [kg]
            - CO : array
                Carbon monoxide emissions [kg]
            - SO2 : array
                Sulfur dioxide emissions [kg]

    save_figure : bool, optional
        Save figure to file if True, default False

    show_legend : bool, optional
        Display segment legend if True, default True

    save_filename : str, optional
        Name for saved figure file, default "Emissions"

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 11

    height : float, optional
        Figure height in inches, default 5

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure showing stacked species emissions and emission indexes

    Notes
    -----
    Left plot: stacked area chart of cumulative emissions mass per species (CO2, CO, NOx, H2O)
    over the full mission timeline. Vertical dashed lines mark segment boundaries.

    Right plot: emission index (g species / kg fuel) per species over time on a semi-log scale.
    """

    # get plotting style
    ps      = plot_style()

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)

    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)

    # fixed colors per species so stacking is visually consistent
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
    ei_markers = {
        'CO2': ps.markers[0],
        'CO' : ps.markers[1],
        'NOx': ps.markers[2],
        'H2O': ps.markers[3],
        'SO2': ps.markers[4],
    }

    # concatenate all segments
    time_all  = []
    mass      = {k: [] for k in ['CO2', 'CO', 'NOx', 'H2O']}
    ei        = {k: [] for k in ['CO2', 'CO', 'NOx', 'H2O', 'SO2']}
    for i, seg in enumerate(results.segments):
        t = seg.conditions.frames.inertial.time[:, 0] / Units.min
        time_all.append(t)
        mass['CO2'].append(seg.conditions.emissions.mass.CO2[:, 0] / 1E3)
        mass['CO' ].append(seg.conditions.emissions.mass.CO[:, 0]  / 1E3)
        mass['NOx'].append(seg.conditions.emissions.mass.NOx[:, 0] / 1E3)
        mass['H2O'].append(seg.conditions.emissions.mass.H2O[:, 0] / 1E3)
        ei['CO2'].append(seg.conditions.emissions.index.CO2[:, 0])
        ei['CO' ].append(seg.conditions.emissions.index.CO[:, 0])
        ei['NOx'].append(seg.conditions.emissions.index.NOx[:, 0])
        ei['H2O'].append(seg.conditions.emissions.index.H2O[:, 0])
        ei['SO2'].append(seg.conditions.emissions.index.SO2[:, 0])

    time_all = np.concatenate(time_all)
    for k in mass:
        mass[k] = np.cumsum(np.concatenate(mass[k]))
    for k in ei:
        ei[k] = np.concatenate(ei[k])

    # --- left plot: stacked area by species ---
    axis_1 = plt.subplot(1, 2, 1)
    bottom = np.zeros_like(time_all)
    for species in ['CO2', 'CO', 'NOx', 'H2O']:
        axis_1.fill_between(time_all, bottom, bottom + mass[species],
                            label=species_labels[species],
                            color=species_colors[species], alpha=0.85)
        bottom += mass[species]

    axis_1.set_ylabel(r'CO2e (Metric Tons)')
    axis_1.set_xlabel(r'Time (mins)')
    set_axes(axis_1)

    # --- right plot: emission index per species ---
    axis_2 = plt.subplot(1, 2, 2)
    ei_line_colors = cm.inferno(np.linspace(0, 0.9, len(results.segments)))

    labels_map = {
        'CO2': r'$CO_2$', 'CO': r'$CO$', 'NOx': r'$NO_x$',
        'H2O': r'$H_2O$', 'SO2': r'$SO_2$'
    }

    for i, seg in enumerate(results.segments):
        t = seg.conditions.frames.inertial.time[:, 0] / Units.min
        seg_ei = seg.conditions.emissions.index
        data_map = {
            'CO2': seg_ei.CO2[:, 0], 'CO': seg_ei.CO[:, 0],
            'NOx': seg_ei.NOx[:, 0], 'H2O': seg_ei.H2O[:, 0],
            'SO2': seg_ei.SO2[:, 0],
        }

        for species in ['CO2', 'CO', 'NOx', 'H2O', 'SO2']:
            axis_2.semilogy(t, data_map[species],
                            color=ei_line_colors[i],
                            marker=ei_markers[species],
                            markersize=ps.marker_size,
                            linewidth=ps.line_width,
                            label=labels_map[species] if i == 0 else None)

    axis_2.set_ylabel(r'Emission Index (g/kg fuel)')
    axis_2.set_xlabel(r'Time (mins)')
    axis_2.minorticks_on()
    axis_2.grid(which='major', linestyle='-', linewidth=0.5, color='grey')
    axis_2.grid(which='minor', linestyle=':', linewidth=0.5, color='grey')
    axis_2.grid(True)

    if show_legend:
        leg = fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=6)
        leg.set_title('Emission Species', prop={'size': ps.legend_font_size, 'weight': 'heavy'})

    fig.tight_layout()
    fig.suptitle('Emissions')
    fig.subplots_adjust(top=0.7)

    if save_figure:
        plt.savefig(save_filename + file_type)
    return fig
