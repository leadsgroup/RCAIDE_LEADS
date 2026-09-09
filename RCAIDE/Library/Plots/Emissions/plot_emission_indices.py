# RCAIDE/Library/Plots/Emissions/plot_emissions
#
#
# Created:  Jul 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import plot_style
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_emission_indices(results,
                    save_figure = False,
                    show_legend = True,
                    save_filename = "Emission_Indices" ,
                    file_type = ".png",
                    width = 8, height = 5):
    """
    Generate a plot showing emission indices by species over the mission.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
        results.segments[i].conditions.emissions.index with fields:
            - CO2 : array, emission index [g/kg fuel]
            - NOx : array, emission index [g/kg fuel]
            - H2O : array, emission index [g/kg fuel]
            - CO  : array, emission index [g/kg fuel]
            - SO2 : array, emission index [g/kg fuel]

    save_figure : bool, optional
        Save figure to file if True, default False

    show_legend : bool, optional
        Display species legend if True, default True

    save_filename : str, optional
        Name for saved figure file, default "Emission_Indices"

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 8

    height : float, optional
        Figure height in inches, default 5

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure showing emission index per species over time on a semi-log scale
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
        'SO2': '#9467bd',
    }
    species_labels = {
        'CO2': r'$CO_2$',
        'CO' : r'$CO$',
        'NOx': r'$NO_x$',
        'H2O': r'$H_2O$',
        'SO2': r'$SO_2$',
    }
    ei_markers = {
        'CO2': ps.markers[0],
        'CO' : ps.markers[1],
        'NOx': ps.markers[2],
        'H2O': ps.markers[3],
        'SO2': ps.markers[4],
    }

    axis_1 = plt.subplot(1, 1, 1)

    first_plotted = False
    for seg in results.segments:
        seg_ei = seg.conditions.emissions.index
        if not hasattr(seg_ei, 'CO2'):
            # ground segment (e.g. Dormancy, Refuel): no fuel burn, index undefined
            continue

        t      = seg.conditions.frames.inertial.time[:, 0] / Units.min
        data_map = {
            'CO2': seg_ei.CO2[:, 0],
            'CO' : seg_ei.CO[:, 0],
            'NOx': seg_ei.NOx[:, 0],
            'H2O': seg_ei.H2O[:, 0],
            'SO2': seg_ei.SO2[:, 0],
        }

        for species in ['CO2', 'CO', 'NOx', 'H2O', 'SO2']:
            axis_1.semilogy(t, data_map[species],
                            color=species_colors[species],
                            marker=ei_markers[species],
                            markersize=ps.marker_size,
                            linewidth=ps.line_width,
                            label=species_labels[species] if not first_plotted else None)
        first_plotted = True

    axis_1.set_ylabel(r'Emission Index (g/kg fuel)')
    axis_1.set_xlabel(r'Time (mins)')
    axis_1.minorticks_on()
    axis_1.grid(which='major', linestyle='-', linewidth=0.5, color='grey')
    axis_1.grid(which='minor', linestyle=':', linewidth=0.5, color='grey')
    axis_1.grid(True)

    if show_legend:
        leg = fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=5)
        leg.set_title('Emission Species', prop={'size': ps.legend_font_size, 'weight': 'heavy'})

    fig.tight_layout()
    fig.suptitle('Emission Indices')
    fig.subplots_adjust(top=0.8)

    if save_figure:
        plt.savefig(save_filename + file_type)
    return fig
