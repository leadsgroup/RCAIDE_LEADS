# RCAIDE/Library/Plots/Geometry/plot_Layout_of_Passenger_Accommodations.py
#
# Created:  Mar 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Geometry.LOPA.compute_layout_of_passenger_accommodations import compute_layout_of_passenger_accommodations

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import numpy as np
import os
import sys

# ----------------------------------------------------------------------------------------------------------------------
#  plot_layout_of_passenger_accommodations
# ----------------------------------------------------------------------------------------------------------------------
def plot_layout_of_passenger_accommodations(fuselage,
                                            save_figure   = False,
                                            show_axes     = False,
                                            fontsize      = 12,
                                            save_filename = "Aircraft_LOPA",):
    '''Plot aircraft layout of passenger accommodations using matplotlib.'''

    if type(fuselage.layout_of_passenger_accommodations) != np.ndarray:
        compute_layout_of_passenger_accommodations(fuselage)

    LOPA = fuselage.layout_of_passenger_accommodations.object_coordinates

    # ── Color palette ──────────────────────────────────────────────────────────
    COLORS = {
        'economy':          '#5B9BD5',   # calm blue
        'economy_exit':     '#2E75B6',   # darker blue for exit rows
        'business':         '#70AD47',   # green
        'business_exit':    '#375623',   # dark green for exit rows
        'first':            '#ED7D31',   # warm orange
        'first_exit':       '#843C0C',   # dark orange for exit rows
        'galley_lav':       '#A5A5A5',   # neutral grey
        'cabin_fill':       '#F5F5F0',   # off-white cabin interior
        'cabin_edge':       '#1F3864',   # dark navy outline
    }

    # ── Build cabin boundary ────────────────────────────────────────────────────
    x_min_locs  = np.where(LOPA[:, 2] == min(LOPA[:, 2]))[0]
    x_min       = LOPA[x_min_locs[0], 2] - LOPA[x_min_locs[0], 5] / 2
    x_min_y_max = max(LOPA[x_min_locs, 3] + LOPA[x_min_locs, 6] / 2)
    x_min_y_min = min(LOPA[x_min_locs, 3] - LOPA[x_min_locs, 6] / 2)
    x_border_pts = [x_min, x_min]
    y_border_pts = [x_min_y_min, x_min_y_max]

    y_max_locs  = np.where(LOPA[:, 3] == max(LOPA[:, 3]))[0]
    y_max       = LOPA[y_max_locs[0], 3] + LOPA[y_max_locs[0], 6] / 2
    y_max_x_max = max(LOPA[y_max_locs, 2] + LOPA[y_max_locs[0], 5] / 2)
    y_max_x_min = min(LOPA[y_max_locs, 2] - LOPA[y_max_locs[0], 5] / 2)
    x_border_pts += [y_max_x_min, y_max_x_max]
    y_border_pts += [y_max, y_max]

    x_max_locs  = np.where(LOPA[:, 2] == max(LOPA[:, 2]))[0]
    x_max       = LOPA[x_max_locs[0], 2] + LOPA[x_max_locs[0], 5] / 2
    x_max_y_max = max(LOPA[x_max_locs, 3] + LOPA[x_max_locs, 6] / 2)
    x_max_y_min = min(LOPA[x_max_locs, 3] - LOPA[x_max_locs, 6] / 2)
    x_border_pts += [x_max, x_max]
    y_border_pts += [x_max_y_max, x_max_y_min]

    y_min_locs  = np.where(LOPA[:, 3] == min(LOPA[:, 3]))[0]
    y_min       = LOPA[y_min_locs[0], 3] - LOPA[y_min_locs[0], 6] / 2
    y_min_x_max = max(LOPA[y_min_locs, 2] + LOPA[y_min_locs[0], 5] / 2)
    y_min_x_min = min(LOPA[y_min_locs, 2] - LOPA[y_min_locs[0], 5] / 2)
    x_border_pts += [y_min_x_max, y_min_x_min]
    y_border_pts += [y_min, y_min]

    y_border_pts = np.array(y_border_pts)
    x_border_pts = np.array(x_border_pts)
    port_idxs    = np.where(y_border_pts < 0)[0]
    sb_x = np.delete(x_border_pts, port_idxs)
    sb_y = np.delete(y_border_pts, port_idxs)

    # ── Figure setup ───────────────────────────────────────────────────────────
    x_span = max(LOPA[:, 2]) - min(LOPA[:, 2])
    y_span = max(LOPA[:, 3]) - min(LOPA[:, 3])
    fig_w  = max(14, x_span * 0.5)
    fig_h  = max(5,  y_span * 1 + 1.5)   # extra room for legend

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    # ── Draw cabin fill ────────────────────────────────────────────────────────
    cabin_x = np.concatenate([sb_x, sb_x[::-1]])
    cabin_y = np.concatenate([sb_y, -sb_y[::-1]])
    ax.fill(cabin_x, cabin_y, color=COLORS['cabin_fill'], zorder=0)
    ax.plot(sb_x,  sb_y,  color=COLORS['cabin_edge'], linewidth=1.8, zorder=1)
    ax.plot(sb_x, -sb_y,  color=COLORS['cabin_edge'], linewidth=1.8, zorder=1)

    # ── Draw seats & galleys ────────────────────────────────────────────────────
    legend_shown = {'economy': False,
                    'business': False, 'business_exit': False,
                    'first': False, 'first_exit': False,
                    'galley_lav': False}

    def _draw_rect(ax, x_c, y_c, s_l, s_w, facecolor, edgecolor, label_key):
        x0, y0 = x_c - s_l / 2, y_c - s_w / 2
        rect = FancyBboxPatch(
            (x0 + 0.01, y0 + 0.01), s_l - 0.02, s_w - 0.02,
            boxstyle="round,pad=0.01",
            linewidth=0.8,
            edgecolor=edgecolor,
            facecolor=facecolor,
            zorder=2,
        )
        ax.add_patch(rect)
        legend_shown[label_key] = True

    for i in range(len(LOPA)):
        x_c     = LOPA[i, 2]
        y_c     = LOPA[i, 3]
        s_l     = LOPA[i, 5]
        s_w     = LOPA[i, 6]
        F_c     = LOPA[i, 7]
        B_c     = LOPA[i, 8]
        E_c     = LOPA[i, 9]
        seat    = LOPA[i, 10]
        em_row  = LOPA[i, 11]
        gal_lav = LOPA[i, 12]

        if E_c == 1.0 and seat == 1.0:
            _draw_rect(ax, x_c, y_c, s_l, s_w,
                       COLORS['economy'], COLORS['economy_exit'], 'economy')

        elif B_c == 1 and seat == 1:
            key  = 'business_exit' if em_row == 1 else 'business'
            fc   = COLORS[key]
            ec   = COLORS['business_exit']
            _draw_rect(ax, x_c, y_c, s_l, s_w, fc, ec, key)

        elif F_c == 1 and seat == 1:
            key  = 'first_exit' if em_row == 1 else 'first'
            fc   = COLORS[key]
            ec   = COLORS['first_exit']
            _draw_rect(ax, x_c, y_c, s_l, s_w, fc, ec, key)

        elif gal_lav == 1:
            _draw_rect(ax, x_c, y_c, s_l, s_w,
                       COLORS['galley_lav'], '#606060', 'galley_lav')

    # ── Axes formatting ────────────────────────────────────────────────────────
    ax.set_aspect('equal')
    pad_x = (max(LOPA[:, 2]) - min(LOPA[:, 2])) * 0.04 + 0.5
    pad_y = (max(LOPA[:, 3]) - min(LOPA[:, 3])) * 0.15 + 0.5
    ax.set_xlim(min(LOPA[:, 2]) - pad_x - LOPA[0, 5],
                max(LOPA[:, 2]) + pad_x + LOPA[0, 5])
    ax.set_ylim(min(LOPA[:, 3]) - pad_y - LOPA[0, 6],
                max(LOPA[:, 3]) + pad_y + LOPA[0, 6])

    for spine in ax.spines.values():
        spine.set_visible(show_axes)

    if show_axes:
        ax.set_xlabel('x (m)', fontsize=fontsize, fontfamily='DejaVu Serif')
        ax.set_ylabel('y (m)', fontsize=fontsize, fontfamily='DejaVu Serif')
        ax.tick_params(labelsize=fontsize - 2)
    else:
        ax.set_xticks([])
        ax.set_yticks([])



    # ── Legend ─────────────────────────────────────────────────────────────────
    legend_labels = {
        'first':         ('First Class',            COLORS['first'],         COLORS['first_exit']),
        'first_exit':    ('First Class – Exit Row',  COLORS['first_exit'],    COLORS['first_exit']),
        'business':      ('Business Class',          COLORS['business'],      COLORS['business_exit']),
        'business_exit': ('Business Class – Exit Row', COLORS['business_exit'], COLORS['business_exit']),
        'economy':       ('Economy Class',           COLORS['economy'],       COLORS['economy_exit']),
        'galley_lav':    ('Galley / Lavatory',       COLORS['galley_lav'],    '#606060'),
    }

    handles = []
    for key, (label, fc, ec) in legend_labels.items():
        if legend_shown[key]:
            handles.append(mpatches.Patch(
                facecolor=fc, edgecolor=ec, linewidth=1.2, label=label))

    if handles:
        leg = ax.legend(
            handles=handles,
            loc='upper left',
            bbox_to_anchor=(1.01, 1.0),
            borderaxespad=0,
            frameon=True,
            framealpha=0.95,
            edgecolor='#CCCCCC',
            fontsize=fontsize - 1,
            title='Seat Class',
            title_fontsize=fontsize,
        )
        leg.get_title().set_fontweight('bold')

    fig.tight_layout()

    # ── Save / show ────────────────────────────────────────────────────────────
    save_path = os.path.join(sys.path[0], save_filename)
    if save_figure:
        fig.savefig(save_path + '.png', dpi=200, bbox_inches='tight',
                    facecolor='white')

    return fig, ax
