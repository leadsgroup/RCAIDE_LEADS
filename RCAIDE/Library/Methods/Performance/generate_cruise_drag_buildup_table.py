# RCAIDE/Library/Methods/Performance/generate_cruise_drag_buildup_table.py
# 
# 
# Created:  Feb 2026, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Library.Plots.Common import set_axes, plot_style    
from RCAIDE.Library.Plots import *
 
# Pacakge imports 
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Patch
import os,sys
import pandas as pd
 
# ----------------------------------------------------------------------
#  Calculate vehicle Payload Range Diagram
# ----------------------------------------------------------------------  
def generate_cruise_drag_buildup_table(mission = None, cruise_segment_tag = "cruise", save_filepath = None):

    if mission == None:
        raise AssertionError('Mission not specifed!')
    mission.tag = "cruise_drag_buildup"
    
    results = mission.evaluate()
    drag = results.segments[cruise_segment_tag].conditions.aerodynamics.coefficients.drag
    eps = 1e-12
    name_map = {
        "main_wing": "Main Wing",
        "vertical_stabilizer": "Vertical\n Stabilizer",
        "nacelle_1": "Nacelle 1",
        "propulsor_2_nacelle": "Nacelle 2",
        "nacelle_1_pylon": "Pylon 1",
        "propulsor_2_nacelle_pylon": "Pylon 2",
        "viscous": "Viscous",
        "inviscid": "Inviscid",
    }

    # --- unpack settings/geometry for parasite normalization (match parasite_total.py logic)
    vehicle  = mission.segments[cruise_segment_tag].analyses.vehicle
    settings = mission.segments[cruise_segment_tag].analyses.aerodynamics.settings
    vehicle_reference_area = vehicle.reference_area

    component_reference_areas = {}
    for wing in vehicle.wings:
        component_reference_areas[wing.tag] = wing.areas.reference
    for fuselage in vehicle.fuselages:
        component_reference_areas[fuselage.tag] = fuselage.areas.front_projected
    for boom in vehicle.booms:
        component_reference_areas[boom.tag] = boom.areas.front_projected
    for network in vehicle.networks:
        for propulsor in network.propulsors:
            if propulsor.nacelle != None:
                nacelle = propulsor.nacelle
                front_area = np.pi * (nacelle.diameter ** 2) / 4
                component_reference_areas[nacelle.tag] = front_area
                component_reference_areas[nacelle.tag + "_pylon"] = front_area

    # --- parasite subcomponents from available keys (excluding "total")
    parasite_sub = []
    for key in drag.parasite.keys():
        if key == "total":
            continue
        item = drag.parasite[key]
        arr = np.asarray(item)
        if arr.ndim == 2:
            val = float(np.mean(arr[:, 0]))
        elif arr.ndim == 1:
            val = float(np.mean(arr))
        elif hasattr(item, "total"):
            arr = np.asarray(item.total)
            if arr.ndim == 2:
                arr = arr[:, 0]
            val = float(np.mean(arr))
        else:
            val = float(np.mean(arr))

        component_ref_area = component_reference_areas[key] if key in component_reference_areas else vehicle_reference_area
        val = val * component_ref_area / vehicle_reference_area
        val = val * (1 - settings.drag_reduction_factors.parasite_drag)

        if abs(val) > eps:
            parasite_sub.append((key, val))

    cd_parasite_total = 0.0
    for _, val in parasite_sub:
        cd_parasite_total += val

    # --- totals (mean over cruise nodes)
    cd_total          = float(np.mean(drag.total[:, 0]))
    cd_induced_total  = float(np.mean(drag.induced.total[:, 0]))
    cd_comp_total     = float(np.mean(drag.compressible.total[:, 0]))
    cd_misc_total     = float(np.mean(drag.miscellaneous.total[:, 0])) 
    cd_form_total     = float(np.mean(drag.form.total[:, 0]))
    cd_cool_total     = float(np.mean(drag.cooling.total[:, 0]))

    item_vis = drag.induced["viscous"]
    arr_vis = np.asarray(item_vis)
    if arr_vis.ndim == 2:
        val_vis = float(np.mean(arr_vis[:, 0]))
    elif arr_vis.ndim == 1:
        val_vis = float(np.mean(arr_vis))
    elif hasattr(item_vis, "total"):
        arr_vis = np.asarray(item_vis.total)
        if arr_vis.ndim == 2:
            arr_vis = arr_vis[:, 0]
        val_vis = float(np.mean(arr_vis))
    else:
        val_vis = float(np.mean(arr_vis))

    item_inv = drag.induced["inviscid"]
    arr_inv = np.asarray(item_inv)
    if arr_inv.ndim == 2:
        val_inv = float(np.mean(arr_inv[:, 0]))
    elif arr_inv.ndim == 1:
        val_inv = float(np.mean(arr_inv))
    elif hasattr(item_inv, "total"):
        arr_inv = np.asarray(item_inv.total)
        if arr_inv.ndim == 2:
            arr_inv = arr_inv[:, 0]
        val_inv = float(np.mean(arr_inv))
    else:
        val_inv = float(np.mean(arr_inv))

    induced_sub = [
        ("viscous", val_vis),
        ("inviscid", val_inv),
    ]
    induced_sub = [(name, val) for name, val in induced_sub if abs(val) > eps]

    categories_raw = [
        ("parasite",      parasite_sub,      cd_parasite_total),
        ("induced",       induced_sub,       cd_induced_total),
        ("compressible",  [("total", cd_comp_total)],  cd_comp_total),
        ("miscellaneous", [("total", cd_misc_total)],  cd_misc_total), 
        ("form",          [("total", cd_form_total)],  cd_form_total),
        ("cooling",       [("total", cd_cool_total)],  cd_cool_total),
        ("TOTAL",         [("total", cd_total)],       cd_total),
    ]
    categories = []
    for cat, subs, tot in categories_raw:
        filtered_subs = [(name, val) for name, val in subs if abs(val) > eps]
        if abs(tot) > eps or cat == "TOTAL":
            categories.append((cat, filtered_subs, tot))

    # -------------------------
    # Terminal print
    # -------------------------
    print("\nCruise Drag Buildup (mean over segment)")
    print("-" * 72)
    for cat, subs, tot in categories:
        print(f"{cat:14s}  total = {tot: .6e}")
        for (name, val) in subs:
            if name != "total":
                print(f"  - {name:28s} {val: .6e}")
    print("-" * 72)

    # -------------------------
    # DataFrame for Excel
    # -------------------------
    rows = []
    for cat, subs, tot in categories:
        for (name, val) in subs:
            if name == "total":
                continue
            rows.append({"category": cat, "component": name, "CD": float(val)})
        rows.append({"category": cat, "component": "total", "CD": float(tot)})
    df = pd.DataFrame(rows)

    # -------------------------
    # Plot (stacked bars, hatched subcomponents)
    # -------------------------
    ps = plot_style()
    plt.rcParams.update({
        "axes.labelsize": ps.axis_font_size + 4,
        "xtick.labelsize": ps.axis_font_size + 4,
        "ytick.labelsize": ps.axis_font_size + 4,
        "axes.titlesize": ps.title_font_size,
    })
    fig, ax = plt.subplots(figsize=(10, 8))

    hatch_list = ["///", "\\\\\\", "xx", "..", "++", "--", "oo", "**", "||", "//"]
    cat_colors = plt.cm.tab10(np.linspace(0, 1, max(len(categories), 1)))
    cat_to_idx = {cat: i for i, (cat, _, _) in enumerate(categories)}

    for i, (cat, subs, tot) in enumerate(categories):
        bottom = 0.0
        base_color = cat_colors[i]

        # stack subcomponents (if any), each with a different hatch
        if len(subs) > 1 or (len(subs) == 1 and subs[0][0] != "total"):
            for j, (name, val) in enumerate(subs):
                ax.barh(i, val, height=0.7, left=bottom,
                        color=base_color, alpha=0.65,
                        hatch=hatch_list[j % len(hatch_list)], edgecolor="k")
                bottom += val
        else:
            bar_val = subs[0][1] if len(subs) > 0 else tot
            ax.barh(i, bar_val, height=0.7, color=base_color, alpha=0.65,
                    hatch=hatch_list[0], edgecolor="k")

        # outline to the category total
        ax.barh(i, tot, height=0.7, fill=False, edgecolor=base_color, linewidth=2.0)

    ax.set_yticks(np.arange(len(categories)))
    ax.set_yticklabels([c[0].capitalize() for c in categories])
    ax.set_xlabel(r"c$_D$")
    ax.set_title("Cruise Drag Buildup")
    parasite_color = cat_colors[cat_to_idx["parasite"]] if "parasite" in cat_to_idx else "0.85"
    induced_color = cat_colors[cat_to_idx["induced"]] if "induced" in cat_to_idx else "0.85"
    parasite_handles = [
        Patch(
            facecolor=parasite_color,
            edgecolor="k",
            alpha=0.65,
            hatch=hatch_list[i % len(hatch_list)],
            label=name_map[name] if name in name_map else name.replace("_", " ").title(),
        )
        for i, (name, val) in enumerate(parasite_sub) if abs(val) > eps
    ]
    induced_handles = [
        Patch(
            facecolor=induced_color,
            edgecolor="k",
            alpha=0.65,
            hatch=hatch_list[i % len(hatch_list)],
            label=name_map[name] if name in name_map else name.replace("_", " ").title(),
        )
        for i, (name, val) in enumerate(induced_sub) if abs(val) > eps
    ]
    if parasite_handles:
        leg1 = ax.legend(
            handles=parasite_handles,
            title="Parasite subcomponents",
            loc="lower right",
            bbox_to_anchor=(0.99, 0.02),
            fontsize=14,
        )
        ax.add_artist(leg1)
    if induced_handles:
        ax.legend(
            handles=induced_handles,
            title="Induced subcomponents",
            loc="lower right",
            bbox_to_anchor=(0.99, 0.42),
            fontsize=14,
        )
    set_axes(ax)
    plt.tight_layout()
    fig.tight_layout()

    # -------------------------
    # Pie chart (major component contribution to total drag)
    # -------------------------
    pie_entries = [
        ("Parasite", cd_parasite_total),
        ("Induced", cd_induced_total),
        ("Compressible", cd_comp_total),
        ("Miscellaneous", cd_misc_total), 
        ("Form", cd_form_total),
        ("Cooling", cd_cool_total),
    ]
    pie_entries = [(label, val) for (label, val) in pie_entries if abs(val) > eps]
    pie_labels = [x[0] for x in pie_entries]
    pie_values = [x[1] for x in pie_entries]
    pie_colors = [cat_colors[cat_to_idx[label.lower()]] for label in pie_labels if label.lower() in cat_to_idx]
    if len(pie_colors) != len(pie_labels):
        pie_colors = plt.cm.tab10(np.linspace(0, 1, len(pie_labels)))
    fig_pie, ax_pie = plt.subplots(figsize=(7, 6))
    ax_pie.pie(
        pie_values,
        labels=pie_labels,
        autopct="%1.1f%%",
        startangle=90,
        colors=pie_colors,
        pctdistance=0.78,
        labeldistance=1.06,
        wedgeprops={"edgecolor": "white", "linewidth": 1.0},
        textprops={"fontsize": 12},
    )
    # ax_pie.set_title("Main Drag Component Contribution (%)")
    ax_pie.axis("equal")
    fig_pie.tight_layout()

    # -------------------------
    # Separate parasite zoom figure
    # -------------------------
    fig_parasite_zoom = None
    if len(parasite_sub) > 0:
        fig_parasite_zoom, ax_pz = plt.subplots(figsize=(6, 4.8))
        parasite_zoom_data = [(n, v) for (n, v) in parasite_sub if n != "main_wing" and abs(v) > eps]
        if len(parasite_zoom_data) == 0:
            parasite_zoom_data = parasite_sub

        x_pz = np.arange(len(parasite_zoom_data))
        for i, (name, val) in enumerate(parasite_zoom_data):
            ax_pz.barh(
                i, val,
                color=parasite_color,
                alpha=0.75,
                edgecolor="k",
                hatch=hatch_list[i % len(hatch_list)],
            )
        ax_pz.set_yticks(x_pz)
        nice_labels = [name_map[n] if n in name_map else n.replace("_", " ").title() for n, _ in parasite_zoom_data]
        ax_pz.set_yticklabels(nice_labels, fontsize=13)
        ax_pz.set_xlabel(r"c$_D$")
        ax_pz.set_title("Parasite Drag Subcomponents")
        ax_pz.grid(True, axis="x", linestyle="--", alpha=0.4)
        zoom_handles = [
            Patch(
                facecolor=parasite_color,
                edgecolor="k",
                alpha=0.75,
                hatch=hatch_list[i % len(hatch_list)],
                label=nice_labels[i],
            )
            for i in range(len(parasite_zoom_data))
        ]
        ax_pz.legend(handles=zoom_handles, title="Parasite subcomponents", loc="upper right", fontsize=14)
        set_axes(ax_pz)
        fig_parasite_zoom.tight_layout()

    # -------------------------
    # Save Excel + image (if save_filepath provided)
    # -------------------------
    if save_filepath is not None:
        if os.path.isdir(save_filepath):
            base_path = os.path.join(save_filepath, "cruise_drag_buildup")
        else:
            base_path, _ = os.path.splitext(save_filepath)

        excel_path = f"{base_path}.xlsx"
        img_path = f"{base_path}.png"
        pie_img_path = f"{base_path}_pie.png"
        parasite_zoom_img_path = f"{base_path}_parasite_zoom.png"

        os.makedirs(os.path.dirname(excel_path) or ".", exist_ok=True)

        df.to_excel(excel_path, index=False)
        fig.savefig(img_path, dpi=200)
        fig_pie.savefig(pie_img_path, dpi=200)
        if fig_parasite_zoom is not None:
            fig_parasite_zoom.savefig(parasite_zoom_img_path, dpi=200)

    return df
