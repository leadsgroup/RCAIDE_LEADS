# RCAIDE/Library/Plots/Energy/plot_powertrain_diagram.py
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

def plot_powertrain_diagram(results,
                          save_figure=False,
                          show_legend=True,
                          save_filename_prefix="Distributor_Power",
                          file_type=".png",
                          width=12, height=7):

    create_network_diagram(results.segments[0].analyses.vehicle)

    ps = plot_style()
    params = {'axes.labelsize': ps.axis_font_size,
              'xtick.labelsize': ps.axis_font_size,
              'ytick.labelsize': ps.axis_font_size,
              'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(params)

    line_colors    = cm.viridis(np.linspace(0, 0.9, len(results.segments)))
    unique_markers = ['v', 'D', '^', 's', 'o', '>', '<', 'p', '*', 'X', 'h']

    figs = {}

    for network in results.segments[0].analyses.vehicle.networks:

        # explicit tag -> distributor
        dist_by_tag = {}
        for d in network.distributors:
            dist_by_tag[d.tag] = d

        for distributor in network.distributors:

            is_elec = isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus)
            is_fuel = isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line)
            is_cool = isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line)
            if not (is_elec or is_fuel or is_cool):
                continue

            # domain for this distributor (also used for links)
            if is_fuel:
                fld = "chemical"
            elif is_cool:
                fld = "thermal"
            else:
                fld = "electrical"

            fig = plt.figure(f"{save_filename_prefix}_{distributor.tag}")
            fig.set_size_inches(width, height)
            ax_supply = plt.subplot(1, 2, 1)
            ax_draw   = plt.subplot(1, 2, 2)

            legend_dict = {}
            mark_index  = 0
            ymax_seen   = 0.0

            # -------------------------------
            # Components (non-distributors)
            # -------------------------------
            groups_names = ["propulsors", "converters", "modulators", "sources", "systems"]
            groups_iters = [network.propulsors, network.converters, network.modulators, network.sources, network.systems]

            for gi in range(len(groups_names)):
                group_name = groups_names[gi]
                comps      = groups_iters[gi]

                # iterator (tag, obj)
                if isinstance(comps, dict):
                    iterator = []
                    for k, v in comps.items():
                        iterator.append((k, v))
                else:
                    iterator = []
                    for c in comps:
                        iterator.append((c.tag, c))

                for comp_tag, comp in iterator:

                    # membership: distributor.tag present in assigned_distributors (by tag)
                    assigned_tags = []
                    entries = getattr(comp, 'assigned_distributors', [])
                    for entry in entries:
                        if isinstance(entry, (list, tuple, set)):
                            for e in entry:
                                if isinstance(e, (list, tuple, set)):
                                    for ee in e:
                                        if isinstance(ee, str):
                                            assigned_tags.append(ee)
                                        else:
                                            t = getattr(ee, 'tag', None)
                                            if isinstance(t, str):
                                                assigned_tags.append(t)
                                else:
                                    if isinstance(e, str):
                                        assigned_tags.append(e)
                                    else:
                                        t = getattr(e, 'tag', None)
                                        if isinstance(t, str):
                                            assigned_tags.append(t)
                        else:
                            if isinstance(entry, str):
                                assigned_tags.append(entry)
                            else:
                                t = getattr(entry, 'tag', None)
                                if isinstance(t, str):
                                    assigned_tags.append(t)

                    if distributor.tag not in assigned_tags:
                        continue

                    m = unique_markers[mark_index % len(unique_markers)]
                    mark_index += 1

                    for si in range(len(results.segments)):
                        seg  = results.segments[si]
                        time = seg.conditions.frames.inertial.time[:, 0] / Units.min
                        npts = seg.conditions.frames.inertial.time.shape[0]

                        # pick component energy record
                        if group_name == "propulsors":
                            crec = seg.conditions.energy.propulsors[comp_tag]
                        elif group_name == "converters":
                            crec = seg.conditions.energy.converters[comp_tag]
                        elif group_name == "modulators":
                            crec = seg.conditions.energy.modulators[comp_tag]
                        elif group_name == "systems":
                            crec = seg.conditions.energy.systems[comp_tag]
                        else:  # sources
                            crec = seg.conditions.energy.sources[comp_tag]

                        # read matching domain as separate supply/draw series (MW)
                        if fld == "chemical":
                            y_sup  = crec.outputs.power.chemical[:npts, 0] / 1e6  # supply to bus
                            y_draw = crec.inputs .power.chemical[:npts, 0] / 1e6  # draw   from bus
                        elif fld == "thermal":
                            y_sup  = crec.outputs.power.thermal[:npts, 0] / 1e6
                            y_draw = crec.inputs .power.thermal[:npts, 0] / 1e6
                        else:
                            y_sup  = crec.outputs.power.electrical[:npts, 0] / 1e6
                            y_draw = crec.inputs .power.electrical[:npts, 0] / 1e6

                        y_sup  = np.asarray(y_sup).reshape(-1)
                        y_draw = np.asarray(y_draw).reshape(-1)

                        # read matching domain as separate supply/draw series (MW)
                        if fld == "chemical":
                            y_sup  = crec.outputs.power.chemical[:npts, 0] / 1e6
                            y_draw = crec.inputs .power.chemical[:npts, 0] / 1e6
                        elif fld == "thermal":
                            y_sup  = crec.outputs.power.thermal[:npts, 0] / 1e6
                            y_draw = crec.inputs .power.thermal[:npts, 0] / 1e6
                        else:
                            y_sup  = crec.outputs.power.electrical[:npts, 0] / 1e6
                            y_draw = crec.inputs .power.electrical[:npts, 0] / 1e6

                        # --- TRU special case: AC side = draw only, DC side = supply only ---
                        if group_name == "modulators":
                            if isinstance(comp, RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit):
                                if distributor.type == 'AC':
                                    y_draw = crec.inputs.power.electrical[:npts, 0] / 1e6
                                    y_sup  = np.zeros_like(y_draw)
                                elif distributor.type == 'DC':
                                    y_sup  = crec.outputs.power.electrical[:npts, 0] / 1e6
                                    y_draw = np.zeros_like(y_sup)

                        eps = 1e-6
                        mask_supply = y_sup  > eps
                        mask_draw   = y_draw > eps
                        if not (np.any(mask_supply) or np.any(mask_draw)):
                            continue

                        if comp_tag in legend_dict:
                            label = None
                        else:
                            label = comp_tag.replace('_', ' ').title()

                        line_for_legend = None

                        if np.any(mask_supply):
                            yy = y_sup[mask_supply]
                            ymax_seen = max(ymax_seen, float(np.max(yy)))
                            line_supply = ax_supply.plot(time[mask_supply], yy,
                                                         color=line_colors[si],
                                                         marker=m,
                                                         linewidth=ps.line_width,
                                                         markersize=ps.marker_size,
                                                         label=label)[0]
                            if line_for_legend is None:
                                line_for_legend = line_supply

                        if np.any(mask_draw):
                            yy = y_draw[mask_draw]
                            ymax_seen = max(ymax_seen, float(np.max(yy)))
                            line_draw = ax_draw.plot(time[mask_draw], yy,
                                                     color=line_colors[si],
                                                     marker=m,
                                                     linewidth=ps.line_width,
                                                     markersize=ps.marker_size,
                                                     label=label)[0]
                            if line_for_legend is None:
                                line_for_legend = line_draw

                        if (comp_tag not in legend_dict) and (label is not None) and (line_for_legend is not None):
                            legend_dict[comp_tag] = line_for_legend

            # -------------------------------
            # Distributor links (signed)
            # -------------------------------
            # Only plot links stored at distributors[this].links[other].power[fld]
            m = unique_markers[mark_index % len(unique_markers)]
            for si in range(len(results.segments)):
                seg  = results.segments[si]
                time = seg.conditions.frames.inertial.time[:, 0] / Units.min
                npts = seg.conditions.frames.inertial.time.shape[0]

                dred = seg.conditions.energy.distributors[distributor.tag]

                # the distributor object itself carries assigned_distributors; gather their tags
                assigned_bus_tags = []
                entries = getattr(distributor, 'assigned_distributors', [])
                for entry in entries:
                    if isinstance(entry, (list, tuple, set)):
                        for e in entry:
                            if isinstance(e, (list, tuple, set)):
                                for ee in e:
                                    if isinstance(ee, str):
                                        assigned_bus_tags.append(ee)
                                    else:
                                        t = getattr(ee, 'tag', None)
                                        if isinstance(t, str):
                                            assigned_bus_tags.append(t)
                            else:
                                if isinstance(e, str):
                                    assigned_bus_tags.append(e)
                                else:
                                    t = getattr(e, 'tag', None)
                                    if isinstance(t, str):
                                        assigned_bus_tags.append(t)
                    else:
                        if isinstance(entry, str):
                            assigned_bus_tags.append(entry)
                        else:
                            t = getattr(entry, 'tag', None)
                            if isinstance(t, str):
                                assigned_bus_tags.append(t)

                # iterate those assigned buses and plot if a link value exists
                for other_tag in assigned_bus_tags:
                    if other_tag not in dred.links:
                        continue
                    y = dred.links[other_tag].power[fld][:npts, 0] / 1e6  # signed: + supply, - draw
                    y = np.asarray(y).reshape(-1)

                    eps = 1e-6
                    mask_supply = y >  eps
                    mask_draw   = y < -eps
                    if not (np.any(mask_supply) or np.any(mask_draw)):
                        continue

                    label_key = f"Link: {other_tag.replace('_',' ').title()}"
                    label = None if label_key in legend_dict else label_key

                    line_for_legend = None

                    if np.any(mask_supply):
                        yy = y[mask_supply]
                        ymax_seen = max(ymax_seen, float(np.max(yy)))
                        line_supply = ax_supply.plot(time[mask_supply], yy,
                                                     color=line_colors[si],
                                                     marker=m,
                                                     linewidth=ps.line_width,
                                                     markersize=ps.marker_size,
                                                     label=label)[0]
                        if line_for_legend is None:
                            line_for_legend = line_supply

                    if np.any(mask_draw):
                        yy = -y[mask_draw]
                        ymax_seen = max(ymax_seen, float(np.max(yy)))
                        line_draw = ax_draw.plot(time[mask_draw], yy,
                                                 color=line_colors[si],
                                                 marker=m,
                                                 linewidth=ps.line_width,
                                                 markersize=ps.marker_size,
                                                 label=label)[0]
                        if line_for_legend is None:
                            line_for_legend = line_draw

                    if (label_key not in legend_dict) and (label is not None) and (line_for_legend is not None):
                        legend_dict[label_key] = line_for_legend

            ax_supply.set_xlabel('Time (min)')
            ax_draw.set_xlabel('Time (min)')
            ax_supply.set_ylabel('Power Supply (MW)')
            ax_draw.set_ylabel('Power Draw (MW)')
            set_axes(ax_supply)
            set_axes(ax_draw)

            if ymax_seen <= 0.0:
                ymax = 1.05
            else:
                ymax = 1.05 * ymax_seen
            ax_supply.set_ylim(0.0, ymax)
            ax_draw.set_ylim(0.0, ymax)

            if show_legend and len(legend_dict) > 0:
                handles = []
                labels  = []
                for _, h in legend_dict.items():
                    handles.append(h)
                    labels.append(h.get_label())
                fig.legend(handles, labels, bbox_to_anchor=(0.5, 0.96),
                           loc='upper center', ncol=8)

            title_text = f"{distributor.tag.replace('_', ' ').title()} Power Profile"
            fig.tight_layout()
            fig.subplots_adjust(top=0.90)
            fig.suptitle(title_text)

            if save_figure:
                plt.savefig(f"{save_filename_prefix}_{distributor.tag}{file_type}")

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
    # Be tolerant to different shapes (list vs single)
    nets = getattr(vehicle, "energy_networks", None) or getattr(vehicle, "networks", None)
    if nets is None:
        print("No energy network on vehicle.")
        return
    network = getattr(nets, "network", None)
    if network is None:
        # maybe nets is already the network or a list of them
        if isinstance(nets, (list, tuple)) and nets:
            network = getattr(nets[-1], "network", nets[-1])
        else:
            network = nets

    # -------------------- collect by type --------------------
    distributors = {str(getattr(d, "tag", f"bus_{i}")): d for i, d in enumerate(getattr(network, "distributors", []))}
    modulators   = {str(getattr(m, "tag", f"mod_{i}")): m for i, m in enumerate(getattr(network, "modulators", []))}
    converters   = {str(getattr(c, "tag", f"conv_{i}")): c for i, c in enumerate(getattr(network, "converters", []))}
    sources      = {str(getattr(s, "tag", f"src_{i}")):  s for i, s in enumerate(getattr(network, "sources", []))}
    systems      = {str(getattr(y, "tag", f"sys_{i}")):  y for i, y in enumerate(getattr(network, "systems", []))}
    propulsors   = {str(getattr(p, "tag", f"prop_{i}")): p for i, p in enumerate(getattr(network, "propulsors", []))}

    # quick lookup by tag
    def get_obj_by_tag(tag):
        if tag in distributors: return distributors[tag]
        if tag in modulators:   return modulators[tag]
        if tag in converters:   return converters[tag]
        if tag in sources:      return sources[tag]
        if tag in systems:      return systems[tag]
        if tag in propulsors:   return propulsors[tag]
        return None

    # -------- NORMALIZATION HELPERS (critical fix) --------
    # Turn any "bus reference" (tag string OR object) into a bus tag string
    def to_bus_tag(ref):
        if ref is None:
            return None
        if isinstance(ref, str):
            return ref if ref in distributors else None
        # Try object with .tag
        tag = getattr(ref, "tag", None)
        if isinstance(tag, str) and tag in distributors:
            return tag
        # Last resort: identity search (slow but safe for small graphs)
        for t, obj in distributors.items():
            if ref is obj:
                return t
        return None

    # tiny flatten
    def flat(x):
        if x is None: return []
        if isinstance(x, (list, tuple, set)):
            out = []
            for xi in x:
                out.extend(flat(xi))
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

    # map helper: add only normalized tags
    def map_component(comp_map, comp_tag, comp_obj):
        for ref in flat(getattr(comp_obj, "assigned_distributors", None)):
            btag = to_bus_tag(ref)
            if btag and btag not in comp_map[comp_tag]:
                comp_map[comp_tag].append(btag)

    for tag, obj in modulators.items(): map_component(mod_to_bus,  tag, obj)
    for tag, obj in converters.items(): map_component(conv_to_bus, tag, obj)
    for tag, obj in sources.items():    map_component(src_to_bus,  tag, obj)
    for tag, obj in systems.items():    map_component(sys_to_bus,  tag, obj)
    for tag, obj in propulsors.items(): map_component(prop_to_bus, tag, obj)

    # optional bus->bus links (if present)
    for dtag, dobj in distributors.items():
        for ref in flat(getattr(dobj, "assigned_distributors", None)):
            other = to_bus_tag(ref)
            if other and other != dtag and other not in bus_to_bus[dtag]:
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
          'line' -> draw line only
        """
        if component_tag is None:
            return "line"
        if component_tag == "__CAPS_LINE__":
            return "line"

        obj = get_obj_by_tag(component_tag)
        if obj is None:
            return "line"

        # explicit hint on the component
        conn = getattr(obj, "connection", None)
        if isinstance(conn, str):
            c = conn.strip().lower()
            if c in ("from", "to"):  # honor if provided
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

        # Default: at least draw a line so the connection is visible
        return "line"

    # -------------------- canvas --------------------
    num_buses  = len(distributors)
    height_est = 2 + (num_buses + 1) * ROW_GAP + 12
    canvas     = [[" "] * WIDTH for _ in range(height_est)]
    title = "AIRCRAFT NETWORK".center(WIDTH)
    for i, ch in enumerate(title): canvas[0][i] = ch
    for i in range(WIDTH): canvas[1][i] = "."

    # draw helpers for connectors
    def draw_connector_top(top_y, bus_row, col, mode):
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
            # For cross-bus CAPS or unknowns, just show text and draw a line
            return (tag_or_text, None)

        attachments = [as_item(t) for t in upstream] \
                    + [as_item(t) for t in mods_here] \
                    + linked_caps_items \
                    + [as_item(t) for t in loads]

        # split onto top/bottom to avoid overlap
        tops, bottoms, flip = [], [], True
        for item in attachments:
            (tops if flip else bottoms).append(item)
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
