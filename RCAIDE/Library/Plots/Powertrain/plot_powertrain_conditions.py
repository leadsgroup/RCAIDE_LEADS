# RCAIDE/Library/Plots/Performance/plot_powertrain_conditions.py
#
#
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style, segment_colors
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_powertrain_conditions(results,
                                save_figure=False,
                                show_legend=True,
                                save_filename="Powertrain_Power_Balance",
                                file_type=".png",
                                width=11, height=5):
    """
    Creates signed power balance plots for each energy domain that has activity.

    For each domain (electrical, chemical, thermal, etc.), a single figure shows:
    - Provider power as positive (sources feeding the network)
    - Consumer power as negative (loads drawing from the network)
    - A solid black line showing the net power (should be ~zero when balanced)

    Segments are plotted with different colors (matching other RCAIDE plots).
    Components are distinguished by markers. Providers use solid lines,
    consumers use dashed lines.

    Only domains with nonzero power flow generate a figure.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and powertrain conditions

    save_figure : bool, optional
        Flag for saving the figure (default: False)

    show_legend : bool, optional
        Flag for displaying plot legend (default: True)

    save_filename : str, optional
        Base name of file for saved figure (default: "Powertrain_Power_Balance")

    file_type : str, optional
        File extension for saved figure (default: ".png")

    width : float, optional
        Figure width in inches (default: 11)

    height : float, optional
        Figure height in inches (default: 5)

    Returns
    -------
    figs : dict
        Dictionary of {domain_name: matplotlib.figure.Figure} for generated plots
    """

    ps = plot_style()
    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)

    power_domains = ['propulsive', 'mechanical', 'electrical', 'chemical',
                     'pneumatic', 'hydraulic', 'thermal']

    line_colors = segment_colors(len(results.segments))

    # ------------------------------------------------------------------
    # First pass: identify which components are providers vs consumers
    # and which domains have activity
    # ------------------------------------------------------------------
    component_roles = {}  # {(tag, domain): 'provider' | 'consumer' | 'both'}
    active_domains = set()

    for segment in results.segments:
        energy_conditions = segment.state.conditions.energy
        for network in segment.analyses.vehicle.networks:
            for group_key, components in [('propulsors', network.propulsors),
                                           ('converters', network.converters),
                                           ('modulators', network.modulators),
                                           ('sources', network.sources),
                                           ('systems', network.systems)]:
                for component in components:
                    tag = component.tag
                    if tag not in energy_conditions[group_key]:
                        continue
                    cond = energy_conditions[group_key][tag]
                    if not hasattr(cond, 'inputs') or not hasattr(cond.inputs, 'power'):
                        continue
                    for domain in power_domains:
                        if hasattr(cond.inputs.power, domain) and hasattr(cond.outputs.power, domain):
                            out_val = cond.outputs.power[domain][:, 0]
                            in_val  = cond.inputs.power[domain][:, 0]
                            has_out = np.any(np.abs(out_val) > 1e-10)
                            has_in  = np.any(np.abs(in_val) > 1e-10)
                            if has_out or has_in:
                                active_domains.add(domain)
                                key = (tag, domain)
                                if has_out and has_in:
                                    component_roles[key] = 'both'
                                elif has_out:
                                    component_roles.setdefault(key, 'provider')
                                elif has_in:
                                    component_roles.setdefault(key, 'consumer')

    # Build ordered list of unique component tags per domain
    domain_components = {}
    for (tag, domain), role in component_roles.items():
        domain_components.setdefault(domain, [])
        if tag not in domain_components[domain]:
            domain_components[domain].append(tag)

    # ------------------------------------------------------------------
    # Create one figure per active domain
    # ------------------------------------------------------------------
    figs = {}

    for domain in power_domains:
        if domain not in active_domains:
            continue

        fig, ax = plt.subplots(1, 1, figsize=(width, height))
        fig.canvas.manager.set_window_title(f'{domain}_power_balance')

        # Classify components as providers or consumers for this domain
        provider_tags = []
        consumer_tags = []
        for tag in domain_components.get(domain, []):
            role = component_roles.get((tag, domain), 'consumer')
            if role == 'provider':
                provider_tags.append(tag)
            else:
                consumer_tags.append(tag)

        # Assign colors: blues for providers, reds for consumers
        blue_shades = cm.Blues(np.linspace(0.4, 0.9, max(len(provider_tags), 1)))
        red_shades  = cm.Reds(np.linspace(0.4, 0.9, max(len(consumer_tags), 1)))
        color_map = {}
        for i, tag in enumerate(provider_tags):
            color_map[tag] = blue_shades[i]
        for i, tag in enumerate(consumer_tags):
            color_map[tag] = red_shades[i]

        marker_map = {tag: ps.markers[i % len(ps.markers)] for i, tag in enumerate(provider_tags + consumer_tags)}

        for seg_i, segment in enumerate(results.segments):
            time = segment.conditions.frames.inertial.time[:, 0] / Units.min
            energy_conditions = segment.state.conditions.energy
            net_power = np.zeros_like(time)

            for network in segment.analyses.vehicle.networks:
                for group_key, components in [('propulsors', network.propulsors),
                                               ('converters', network.converters),
                                               ('modulators', network.modulators),
                                               ('sources', network.sources),
                                               ('systems', network.systems)]:
                    for component in components:
                        tag = component.tag
                        if tag not in energy_conditions[group_key]:
                            continue
                        cond = energy_conditions[group_key][tag]
                        if not hasattr(cond, 'inputs') or not hasattr(cond.inputs, 'power'):
                            continue
                        if not hasattr(cond.inputs.power, domain):
                            continue

                        out_power = cond.outputs.power[domain][:, 0] / 1e3
                        in_power  = cond.inputs.power[domain][:, 0] / 1e3
                        signed_power = out_power - in_power
                        net_power += signed_power

                        if np.all(np.abs(signed_power) < 1e-10):
                            continue

                        label = tag if seg_i == 0 else None

                        ax.plot(time, signed_power, color=color_map.get(tag, 'gray'),
                                marker=marker_map.get(tag, 'o'),
                                linewidth=ps.line_width,
                                label=label)

            # Net power per segment
            label_net = 'Net Balance' if seg_i == 0 else None
            ax.plot(time, net_power, color='black',
                    linewidth=ps.line_width,
                    linestyle='-', label=label_net, alpha=0.8)

        # Zero line
        ax.axhline(y=0, color='gray', linewidth=0.5, linestyle='-', alpha=0.5)

        ax.set_xlabel('Time (min)')
        ax.set_ylabel(f'{domain.capitalize()} Power (kW)')
        ax.set_title(f'{domain.capitalize()} Power Balance')

        if show_legend:
            ax.legend(loc='best', fontsize=ps.legend_font_size - 2, ncol=2)

        set_axes(ax)
        fig.tight_layout()

        if save_figure:
            fig.savefig(f'{save_filename}_{domain}{file_type}')

        figs[domain] = fig

    return figs
