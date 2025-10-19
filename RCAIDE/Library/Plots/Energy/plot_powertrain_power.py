# RCAIDE/Library/Plots/Energy/plot_powertrain_power.py
#
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------

def plot_powertrain_power(results,
                          save_figure = False,
                          show_legend = True,
                          save_filename_prefix = "Distributor_Power",
                          file_type = ".png",
                          width = 14, height = 9):
    
    create_network_diagram(results.segments[0].analyses.energy.vehicle)

    # get plotting style 
    ps = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters) 
    
    # color map across segments (color by segment)
    line_colors = cm.inferno(np.linspace(0,0.9,len(results.segments)))     

    figs = {}

    for network in results.segments[0].analyses.energy.vehicle.networks:  
        for distributor in network.distributors:

            # choose domain by distributor type
            is_elec = isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus)
            is_fuel = isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line)
            if not (is_elec or is_fuel):
                continue  # only plot for electrical buses and fuel lines

            # figure and axis for this distributor
            fig = plt.figure(f"{save_filename_prefix}_{distributor.tag}")
            fig.set_size_inches(width, height)   
            ax  = plt.subplot(1,1,1)

            # plot distributor net power (one curve per segment, same marker)
            for i in range(len(results.segments)):  
                time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
                dcon = results.segments[i].conditions.energy.distributors[distributor.tag]
                if is_elec:
                    net = dcon.net_electrical_power[:,0]
                    ylab = 'Electrical Power (W)'
                    title = f'{distributor.tag} – Electrical'
                else:
                    net = dcon.net_chemical_power[:,0]
                    ylab = 'Chemical Power (W)'
                    title = f'{distributor.tag} – Chemical'

                if i == 0:
                    ax.plot(time, net, color=line_colors[i], marker='o',
                            markersize=ps.marker_size, linewidth=ps.line_width,
                            label=f'{distributor.tag} (net)')
                else:
                    ax.plot(time, net, color=line_colors[i], marker='o',
                            markersize=ps.marker_size, linewidth=ps.line_width)

            # helper to flatten assigned_distributors lists (simple, inline)
            def _has_dist(component, tag):
                for entry in getattr(component, 'assigned_distributors', []):
                    if isinstance(entry, (list, tuple, set)):
                        if tag in entry:
                            return True
                    else:
                        if tag == entry:
                            return True
                return False

            # plot components connected to this distributor (different markers per component)
            mark_idx = 0

            # Propulsors
            for comp in network.propulsors:
                if _has_dist(comp, distributor.tag):
                    for i in range(len(results.segments)):
                        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
                        ccon = results.segments[i].conditions.energy.propulsors[comp.tag].power
                        if is_elec and hasattr(ccon, 'electrical'):
                            y = ccon.electrical[:,0]
                        elif is_fuel and hasattr(ccon, 'chemical'):
                            y = ccon.chemical[:,0]
                        else:
                            continue
                        if i == 0:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width, label=comp.tag)
                        else:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width)
                    mark_idx += 1

            # Converters
            for tag in getattr(network, 'non_propulsive_converters', []):
                comp = network.converters[tag]
                if _has_dist(comp, distributor.tag):
                    for i in range(len(results.segments)):
                        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
                        ccon = results.segments[i].conditions.energy.converters[tag].power
                        if is_elec and hasattr(ccon, 'electrical'):
                            y = ccon.electrical[:,0]
                        elif is_fuel and hasattr(ccon, 'chemical'):
                            y = ccon.chemical[:,0]
                        else:
                            continue
                        if i == 0:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width, label=tag)
                        else:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width)
                    mark_idx += 1

            # Modulators
            for comp in network.modulators:
                if _has_dist(comp, distributor.tag):
                    for i in range(len(results.segments)):
                        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
                        ccon = results.segments[i].conditions.energy.modulators[comp.tag].power
                        if is_elec and hasattr(ccon, 'electrical'):
                            y = ccon.electrical[:,0]
                        elif is_fuel and hasattr(ccon, 'chemical'):
                            y = ccon.chemical[:,0]
                        else:
                            continue
                        if i == 0:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width, label=comp.tag)
                        else:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width)
                    mark_idx += 1

            # Systems
            for comp in network.systems:
                if _has_dist(comp, distributor.tag):
                    for i in range(len(results.segments)):
                        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
                        ccon = results.segments[i].conditions.energy.systems[comp.tag].power
                        if is_elec and hasattr(ccon, 'electrical'):
                            y = ccon.electrical[:,0]
                        elif is_fuel and hasattr(ccon, 'chemical'):
                            y = ccon.chemical[:,0]
                        else:
                            continue
                        if i == 0:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width, label=comp.tag)
                        else:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width)
                    mark_idx += 1

            # Sources
            for comp in network.sources:
                if _has_dist(comp, distributor.tag):
                    for i in range(len(results.segments)):
                        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
                        ccon = results.segments[i].conditions.energy.sources[comp.tag].power
                        if is_elec and hasattr(ccon, 'electrical'):
                            y = ccon.electrical[:,0]
                        elif is_fuel and hasattr(ccon, 'chemical'):
                            y = ccon.chemical[:,0]
                        else:
                            continue
                        if i == 0:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width, label=comp.tag)
                        else:
                            ax.plot(time, y, color=line_colors[i], marker=ps.markers[mark_idx % len(ps.markers)],
                                    markersize=ps.marker_size, linewidth=ps.line_width)
                    mark_idx += 1

            ax.set_xlabel('Time (mins)')
            ax.set_ylabel(ylab)
            set_axes(ax)

            if show_legend:
                fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)

            fig.tight_layout()
            fig.subplots_adjust(top=0.85)
            fig.suptitle(title)

            if save_figure:
                plt.savefig(f"{save_filename_prefix}_{distributor.tag}" + file_type)

            figs[distributor.tag] = fig

    return figs

def create_network_diagram(vehicle):
    # -------------------- constants --------------------
    BOX_W, BOX_H = 17, 3
    WIDTH = 160
    ROW_GAP = 14
    VERTICAL_OFFSET = 3
    BUS_LEFT_MARGIN = 4

    # -------------------- get network --------------------
    network = vehicle.networks.network

    # -------------------- collect by type --------------------
    distributors = {getattr(d, "tag", f"bus_{i}"): d for i, d in enumerate(network.distributors)}
    modulators   = {getattr(m, "tag", f"mod_{i}"): m for i, m in enumerate(network.modulators)}
    converters   = {getattr(c, "tag", f"conv_{i}"): c for i, c in enumerate(network.converters)}
    sources      = {getattr(s, "tag", f"src_{i}"):  s for i, s in enumerate(network.sources)}
    systems      = {getattr(y, "tag", f"sys_{i}"):  y for i, y in enumerate(network.systems)}
    propulsors   = {getattr(p, "tag", f"prop_{i}"): p for i, p in enumerate(network.propulsors)}

    # quick lookup by tag
    def get_obj_by_tag(tag):
        if tag in distributors: return distributors[tag]
        if tag in modulators:   return modulators[tag]
        if tag in converters:   return converters[tag]
        if tag in sources:      return sources[tag]
        if tag in systems:      return systems[tag]
        if tag in propulsors:   return propulsors[tag]
        return None

    # tiny flatten for assigned_distributors
    def flat(x):
        if x is None: return []
        if isinstance(x, str): return [x]
        if isinstance(x, (list, tuple, set)):
            out = []
            for xi in x:
                if isinstance(xi, (list, tuple, set)):
                    out.extend(list(xi))
                else:
                    out.append(xi)
            return out
        return [x]

    # clamp label to box width
    def clamp_label(s, w=BOX_W-2):
        s = str(s)
        return s if len(s) <= w else s[:max(0, w-3)] + "..."

    # -------------------- reverse indexes (from component.assigned_distributors) --------------------
    from collections import defaultdict
    mod_to_bus  = defaultdict(list)
    conv_to_bus = defaultdict(list)
    src_to_bus  = defaultdict(list)
    sys_to_bus  = defaultdict(list)
    prop_to_bus = defaultdict(list)
    bus_to_bus  = defaultdict(list)  # if buses reference other buses

    for tag, obj in modulators.items():
        for dtag in flat(getattr(obj, "assigned_distributors", None)):
            if dtag in distributors and dtag not in mod_to_bus[tag]:
                mod_to_bus[tag].append(dtag)

    for tag, obj in converters.items():
        for dtag in flat(getattr(obj, "assigned_distributors", None)):
            if dtag in distributors and dtag not in conv_to_bus[tag]:
                conv_to_bus[tag].append(dtag)

    for tag, obj in sources.items():
        for dtag in flat(getattr(obj, "assigned_distributors", None)):
            if dtag in distributors and dtag not in src_to_bus[tag]:
                src_to_bus[tag].append(dtag)

    for tag, obj in systems.items():
        for dtag in flat(getattr(obj, "assigned_distributors", None)):
            if dtag in distributors and dtag not in sys_to_bus[tag]:
                sys_to_bus[tag].append(dtag)

    for tag, obj in propulsors.items():
        for dtag in flat(getattr(obj, "assigned_distributors", None)):
            if dtag in distributors and dtag not in prop_to_bus[tag]:
                prop_to_bus[tag].append(dtag)

    # optional bus->bus links (if present)
    for dtag, dobj in distributors.items():
        for other in flat(getattr(dobj, "assigned_distributors", None)):
            if other in distributors and other not in bus_to_bus[dtag]:
                bus_to_bus[dtag].append(other)

    # -------------------- arrow direction rules --------------------
    def is_tru(obj):
        cls = type(obj).__name__.lower()
        tag = str(getattr(obj, "tag", "")).lower()
        return ("transformer" in cls and "rectifier" in cls) or tag.startswith("tru")

    def conn_dir_for(component_tag, bus_tag):
        """
        Returns:
          'from' -> component -> bus (arrow at bus side)
          'to'   -> bus -> component (arrow at component side)
          'line' -> draw line only (for bus-to-bus link)
          None   -> nothing
        """
        if component_tag is None:
            return None
        if component_tag == "__CAPS_LINE__":
            return "line"
        obj = get_obj_by_tag(component_tag)
        if obj is None:
            return None

        # honor explicit connection hint
        conn = getattr(obj, "connection", None)
        if isinstance(conn, str):
            c = conn.strip().lower()
            if c in ("from", "to"):
                return c

        # TRU convention: AC side 'to', DC side 'from'
        if is_tru(obj) and bus_tag in distributors:
            btype = str(getattr(distributors[bus_tag], "bus_type", "")).upper()
            if btype == "AC": return "to"
            if btype == "DC": return "from"

        # generic supplier/consumer
        sup = bool(getattr(obj, "energy_supplier", False))
        con = bool(getattr(obj, "energy_consumer", False))
        if sup and not con:  return "from"
        if con and not sup:  return "to"
        if sup and con:
            name = (type(obj).__name__ + " " + str(getattr(obj, "tag", ""))).lower()
            return "from" if "generator" in name else "to"
        return None

    # -------------------- canvas --------------------
    num_buses  = len(distributors)
    height_est = 2 + (num_buses + 1) * ROW_GAP + 12
    canvas     = [[" "] * WIDTH for _ in range(height_est)]
    title = "AIRCRAFT NETWORK".center(WIDTH)
    for i, ch in enumerate(title): canvas[0][i] = ch
    for i in range(WIDTH): canvas[1][i] = "."

    # draw helpers for connectors
    def draw_connector_top(top_y, bus_row, col, mode):
        if mode is None:
            return
        line_only = (mode == "line")
        for rr in range(top_y + BOX_H, bus_row):
            canvas[rr][col] = "|"
        if not line_only and mode == "to":
            if top_y + BOX_H < bus_row:
                canvas[top_y + BOX_H][col] = "^"
        if not line_only and mode == "from":
            if bus_row - 1 >= top_y + BOX_H:
                canvas[bus_row - 1][col] = "v"

    def draw_connector_bottom(bus_row, bot_y, col, mode):
        if mode is None:
            return
        line_only = (mode == "line")
        for rr in range(bus_row + 1, bot_y):
            canvas[rr][col] = "|"
        if not line_only and mode == "to":
            if bot_y - 1 > bus_row:
                canvas[bot_y - 1][col] = "v"
        if not line_only and mode == "from":
            if bus_row + 1 < bot_y:
                canvas[bus_row + 1][col] = "^"

    # simple placement along the row
    def place_positions(n, bus_left, bus_w_min):
        if n <= 0: return []
        if n == 1: return [bus_left + bus_w_min // 2]
        step = max(BOX_W + 2, bus_w_min // (n + 1))
        xs, x = [], bus_left + step
        for _ in range(n):
            xs.append(x)
            x += step
        return xs

    # -------------------- render each bus row --------------------
    current_top_row = 2
    for dtag, d in distributors.items():
        dtype     = getattr(d, "bus_type", "BUS")
        bus_label = f"{dtag.upper()}  [{dtype}]"

        # upstream (sources, converters)
        upstream = [t for t, bs in src_to_bus.items()  if dtag in bs] + \
                   [t for t, bs in conv_to_bus.items() if dtag in bs]
        # modulators on this bus
        mods_here = [t for t, bs in mod_to_bus.items() if dtag in bs]
        # cross-bus links as CAPS (line only)
        linked_caps_items = []
        for other in bus_to_bus.get(dtag, []):
            if other in distributors:
                linked_caps_items.append((other.upper(), "__CAPS_LINE__"))
        # loads (systems, propulsors)
        loads = [t for t, bs in sys_to_bus.items()  if dtag in bs] + \
                [t for t, bs in prop_to_bus.items() if dtag in bs]

        # turn into uniform (label, tag) items
        def as_item(tag_or_text):
            if isinstance(tag_or_text, str) and get_obj_by_tag(tag_or_text) is not None:
                return (tag_or_text, tag_or_text)
            return (tag_or_text, None)

        attachments = [as_item(t) for t in upstream] \
                    + [as_item(t) for t in mods_here] \
                    + linked_caps_items \
                    + [as_item(t) for t in loads]

        # split onto top/bottom
        tops, bottoms, flip = [], [], True
        for item in attachments:
            if flip: tops.append(item)
            else:    bottoms.append(item)
            flip = not flip

        # bus row geometry
        bus_left  = BUS_LEFT_MARGIN
        bus_width = min(WIDTH - 2 * BUS_LEFT_MARGIN, 160)
        bus_row   = current_top_row + 6

        # draw bus title bar
        title_text = f" {bus_label} "
        bus_w_min  = max(bus_width, len(title_text) + 6)
        L = (bus_w_min - len(title_text)) // 2
        R = bus_w_min - len(title_text) - L
        bus_string = "=" * L + title_text + "=" * R
        for i, ch in enumerate(bus_string):
            if bus_left + i < WIDTH:
                canvas[bus_row][bus_left + i] = ch

        top_cols = place_positions(len(tops), bus_left, bus_w_min)
        bot_cols = place_positions(len(bottoms), bus_left, bus_w_min)

        # TOP attachments
        for (label_item, tag_item), cx in zip(tops, top_cols):
            c0 = max(bus_left, min(cx - BOX_W // 2, bus_left + bus_w_min - BOX_W))
            label = clamp_label(label_item)
            top = "." + "-" * (BOX_W - 2) + "."
            mid = "|" + label.center(BOX_W - 2) + "|"
            bot = "'" + "-" * (BOX_W - 2) + "'"
            top_y = bus_row - BOX_H - VERTICAL_OFFSET
            for j, ch in enumerate(top): canvas[top_y + 0][c0 + j] = ch
            for j, ch in enumerate(mid): canvas[top_y + 1][c0 + j] = ch
            for j, ch in enumerate(bot): canvas[top_y + 2][c0 + j] = ch
            conn_c = c0 + BOX_W // 2
            draw_connector_top(top_y, bus_row, conn_c, conn_dir_for(tag_item, dtag))

        # BOTTOM attachments
        for (label_item, tag_item), cx in zip(bottoms, bot_cols):
            c0 = max(bus_left, min(cx - BOX_W // 2, bus_left + bus_w_min - BOX_W))
            label = clamp_label(label_item)
            top = "." + "-" * (BOX_W - 2) + "."
            mid = "|" + label.center(BOX_W - 2) + "|"
            bot = "'" + "-" * (BOX_W - 2) + "'"
            bot_y = bus_row + VERTICAL_OFFSET + 1
            for j, ch in enumerate(top): canvas[bot_y + 0][c0 + j] = ch
            for j, ch in enumerate(mid): canvas[bot_y + 1][c0 + j] = ch
            for j, ch in enumerate(bot): canvas[bot_y + 2][c0 + j] = ch
            conn_c = c0 + BOX_W // 2
            draw_connector_bottom(bus_row, bot_y, conn_c, conn_dir_for(tag_item, dtag))

        current_top_row += ROW_GAP

    # trim & print
    while canvas and all(ch == " " for ch in canvas[-1]): canvas.pop()
    for row in canvas:
        print("".join(row).rstrip())
    return
