# RCAIDE/Library/Plots/Geometry/plot_Layout_of_Passenger_Accommodations.py
#
# Created:  Mar 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Geometry.LOPA.compute_layout_of_passenger_accommodations import compute_layout_of_passenger_accommodations

import plotly.graph_objects as go
import numpy as np
import os
import sys

# ----------------------------------------------------------------------------------------------------------------------
#  plot_Layout_of_Passenger_Accommodations
# ----------------------------------------------------------------------------------------------------------------------
def plot_layout_of_passenger_accommodations(fuselage,
                                            save_figure   = False,
                                            show_axes     = False,
                                            fontsize      = 20,
                                            save_filename = "Aircraft_LOPA",
                                            show_figure   = True):
    '''Plot aircraft layout of passenger accommodations.'''

    if type(fuselage.layout_of_passenger_accommodations) != np.ndarray:
        compute_layout_of_passenger_accommodations(fuselage)

    LOPA = fuselage.layout_of_passenger_accommodations.object_coordinates

    fig = go.Figure()
    fig.update_xaxes(range=[LOPA[:, 2].min() - 1, LOPA[:, 2].max() + 1], showgrid=True)
    fig.update_yaxes(range=[LOPA[:, 3].min() - 1, LOPA[:, 3].max() + 1],
                     showgrid=True, scaleanchor="x", scaleratio=1)

    # ------------------------------------------------------------------
    # Cabin boundary
    # ------------------------------------------------------------------
    x_min_locs  = np.where(LOPA[:, 2] == LOPA[:, 2].min())[0]
    x_min       = LOPA[x_min_locs[0], 2] - LOPA[x_min_locs[0], 5] / 2
    x_min_y_max = (LOPA[x_min_locs, 3] + LOPA[x_min_locs, 6] / 2).max()
    x_min_y_min = (LOPA[x_min_locs, 3] - LOPA[x_min_locs, 6] / 2).min()

    y_max_locs  = np.where(LOPA[:, 3] == LOPA[:, 3].max())[0]
    y_max       = LOPA[y_max_locs[0], 3] + LOPA[y_max_locs[0], 6] / 2
    y_max_x_max = (LOPA[y_max_locs, 2] + LOPA[y_max_locs[0], 5] / 2).max()
    y_max_x_min = (LOPA[y_max_locs, 2] - LOPA[y_max_locs[0], 5] / 2).min()

    x_max_locs  = np.where(LOPA[:, 2] == LOPA[:, 2].max())[0]
    x_max       = LOPA[x_max_locs[0], 2] + LOPA[x_max_locs[0], 5] / 2
    x_max_y_max = (LOPA[x_max_locs, 3] + LOPA[x_max_locs, 6] / 2).max()
    x_max_y_min = (LOPA[x_max_locs, 3] - LOPA[x_max_locs, 6] / 2).min()

    y_min_locs  = np.where(LOPA[:, 3] == LOPA[:, 3].min())[0]
    y_min       = LOPA[y_min_locs[0], 3] - LOPA[y_min_locs[0], 6] / 2
    y_min_x_max = (LOPA[y_min_locs, 2] + LOPA[y_min_locs[0], 5] / 2).max()
    y_min_x_min = (LOPA[y_min_locs, 2] - LOPA[y_min_locs[0], 5] / 2).min()

    x_border = np.array([x_min, x_min, y_max_x_min, y_max_x_max,
                          x_max, x_max, y_min_x_max, y_min_x_min])
    y_border = np.array([x_min_y_min, x_min_y_max, y_max, y_max,
                          x_max_y_max, x_max_y_min, y_min, y_min])

    starboard = y_border >= 0
    fig.add_trace(go.Scatter(x=x_border[starboard], y= y_border[starboard],
                             mode='lines', line_color='darkblue', fill=None, showlegend=False))
    fig.add_trace(go.Scatter(x=x_border[starboard], y=-y_border[starboard],
                             mode='lines', line_color='darkblue', fill='tonexty', showlegend=False))

    # ------------------------------------------------------------------
    # Seat / galley rectangles — batched by visual category to minimise
    # the number of Plotly objects (7 traces instead of one per seat).
    #
    # Each batch accumulates closed-polygon points with None separators
    # so a single go.Scatter trace renders every rect of that category.
    # ------------------------------------------------------------------
    economy_seat_colors  = ["steelblue",  "deepskyblue", "skyblue"]
    business_seat_colors = ["seagreen",   "mediumseagreen", "lightseagreen"]
    first_seat_colors    = ["indianred",  "lightcoral",  "lightpink"]
    lavatory_color       = "sandybrown"

    # key → [x_pts, y_pts, line_color, fill_color]
    batches = {
        ('E', True):  ([], [], economy_seat_colors[0],  economy_seat_colors[1]),
        ('E', False): ([], [], economy_seat_colors[0],  economy_seat_colors[2]),
        ('B', True):  ([], [], business_seat_colors[0], business_seat_colors[1]),
        ('B', False): ([], [], business_seat_colors[0], business_seat_colors[2]),
        ('F', True):  ([], [], first_seat_colors[0],    first_seat_colors[1]),
        ('F', False): ([], [], first_seat_colors[0],    first_seat_colors[2]),
        'Lav':        ([], [], lavatory_color,           lavatory_color),
    }

    for row in LOPA:
        xc, yc      = row[2], row[3]
        sl, sw      = row[5], row[6]
        F_c, B_c    = row[7], row[8]
        E_c, seat   = row[9], row[10]
        em_row      = row[11]
        gal_lav     = row[12]

        # Closed-polygon corners + None separator
        x0, x1 = xc - sl / 2, xc + sl / 2
        y0, y1 = yc - sw / 2, yc + sw / 2
        xs = [x0, x1, x1, x0, x0, None]
        ys = [y0, y0, y1, y1, y0, None]

        if   E_c == 1.0 and seat == 1.0:
            key = ('E', em_row == 1.0)
        elif B_c == 1   and seat == 1:
            key = ('B', em_row == 1)
        elif F_c == 1   and seat == 1:
            key = ('F', em_row == 1)
        elif gal_lav == 1:
            key = 'Lav'
        else:
            continue

        batches[key][0].extend(xs)
        batches[key][1].extend(ys)

    for xs, ys, line_color, fill_color in batches.values():
        if not xs:
            continue
        fig.add_trace(go.Scatter(
            x=xs, y=ys,
            mode='lines',
            line=dict(color=line_color, width=2),
            fill='toself',
            fillcolor=fill_color,
            showlegend=False,
        ))

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------
    common = dict(showlegend=False, font=dict(family="Times New Roman", size=fontsize, color="black"))
    if show_axes:
        fig.update_layout(**common,
            xaxis_title='x', yaxis_title='y',
            xaxis=dict(showline=True, linewidth=1, linecolor='black',
                       showticklabels=True, ticks="outside", tickcolor='black'),
            yaxis=dict(showline=True, linewidth=1, linecolor='black',
                       showticklabels=True, ticks="outside", tickcolor='black'))
    else:
        fig.update_layout(**common,
            plot_bgcolor="white", paper_bgcolor="white",
            xaxis=dict(showline=False, linecolor='white', showticklabels=False, tickcolor='white'),
            yaxis=dict(showline=False, linecolor='white', showticklabels=False, tickcolor='white'))

    save_filename = os.path.join(sys.path[0], save_filename)
    if save_figure:
        fig.write_image(save_filename + ".png")
    if show_figure:
        fig.write_html(save_filename + '.html', auto_open=True)

    del fig
