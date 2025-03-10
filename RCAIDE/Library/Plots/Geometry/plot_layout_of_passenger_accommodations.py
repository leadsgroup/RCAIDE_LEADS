# RCAIDE/Library/Plots/Geometry/plot_Layout_of_Passenger_Accommodations.py
#  
# Created:  Mar 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Geometry.LOPA.compute_layout_of_passenger_accommodations import  compute_layout_of_passenger_accommodations

import plotly.graph_objects as go
import  numpy as  np 
# ----------------------------------------------------------------------------------------------------------------------
#  plot_Layout_of_Passenger_Accommodations
# ---------------------------------------------------------------------------------------------------------------------- 
def plot_layout_of_passenger_accommodations(fuselage):
    '''
    Plot aircraft layout of passenger accommodations
    '''  
    if  type(fuselage.layout_of_passenger_accommodations) != np.ndarray: 
        compute_layout_of_passenger_accommodations(fuselage)   
    plot_LOPA(fuselage.layout_of_passenger_accommodations)
    return 
        
def plot_LOPA(LOPA):
    fig = go.Figure() 
    # Set axes properties
    fig.update_xaxes(range=[min(LOPA[:,2]) - 1 , max(LOPA[:,2]) + 1], showgrid=False)
    fig.update_yaxes(range=[ min(LOPA[:,3]) - 1, max(LOPA[:,3])  + 1], showgrid=False)  
    fig.update_yaxes(
        scaleanchor = "x",
        scaleratio = 1,)    
        
    # Step 1: plot cabin bounds  
    # get points at x min 
    x_min_locs   =  np.where( LOPA[:,2] == min(LOPA[:,2]))[0]
    x_min        =  LOPA[x_min_locs[0],2] -  LOPA[x_min_locs[0],5]/2
    x_min_y_max  =  max(LOPA[x_min_locs,3] + LOPA[x_min_locs,6]/2 )
    x_min_y_min  =  min(LOPA[x_min_locs,3] - LOPA[x_min_locs,6]/2 ) 
    x_border_pts = [x_min, x_min] 
    y_border_pts = [x_min_y_min, x_min_y_max] 

    # get points at y max 
    y_max_locs   =  np.where( LOPA[:,3] == max(LOPA[:,3]))[0]
    y_max        =  LOPA[y_max_locs[0],3] + LOPA[y_max_locs[0],6]/2 
    y_max_x_max  =  max(LOPA[y_max_locs,2] + LOPA[y_max_locs[0],5]/2)
    y_max_x_min  =  min(LOPA[y_max_locs,2] - LOPA[y_max_locs[0],5]/2) 
    x_border_pts.append(y_max_x_min)
    x_border_pts.append(y_max_x_max)
    y_border_pts.append(y_max)
    y_border_pts.append(y_max) 

    # get points at x max 
    x_max_locs   =  np.where( LOPA[:,2] == max(LOPA[:,2]))[0]
    x_max        =  LOPA[x_max_locs[0],2] + LOPA[x_max_locs[0],5]/2
    x_max_y_max  =  max(LOPA[x_max_locs,3] + LOPA[x_max_locs,6]/2)
    x_max_y_min  =  min(LOPA[x_max_locs,3] - LOPA[x_max_locs,6]/2)  
    x_border_pts.append(x_max)
    x_border_pts.append(x_max)
    y_border_pts.append(x_max_y_max)
    y_border_pts.append(x_max_y_min)
    
    # get points at y min  
    y_min_locs   =  np.where( LOPA[:,3] == min(LOPA[:,3]))[0]
    y_min        =  LOPA[y_min_locs[0],3] - LOPA[y_min_locs[0],6]/2 
    y_min_x_max  =  max(LOPA[y_min_locs,2] + LOPA[y_min_locs[0],5]/2)
    y_min_x_min  =  min(LOPA[y_min_locs,2] - LOPA[y_min_locs[0],5]/2)
    x_border_pts.append(y_min_x_max)  
    x_border_pts.append(y_min_x_min)
    y_border_pts.append(y_min)
    y_border_pts.append(y_min)    
    
    # loop through points and determine if there are duplicates
    y_border_pts = np.array(y_border_pts)
    x_border_pts = np.array(x_border_pts)
    # cut where y is negative
    port_idxs  =  np.where(y_border_pts<0)[0]
    starboard_x_points = np.delete(x_border_pts, port_idxs) 
    starboard_y_points = np.delete(y_border_pts, port_idxs)
     
    fig.add_trace(go.Scatter(
        x=starboard_x_points, 
        y=starboard_y_points, 
        mode='lines',
        name='Line 1',
        line_color="darkblue", 
        fill=None 
    ))
    
    fig.add_trace(go.Scatter(
        x=starboard_x_points, 
        y=-starboard_y_points, 
        mode='lines',
        name='Line 2',
        line_color="darkblue", 
        fill='tonexty'
    ))
    
     
    # Step 2: plot seats
    economoy_seat_colors = ["steelblue", "deepskyblue", "skyblue" ]
    business_seat_colors = ["seagreen", "mediumseagreen", "lightseagreen" ]
    first_seat_colors    = ["indianred", "lightcoral", "lightpink" ]
    lavatory_color       = ["sandybrown"]
   
    for i in range(len(LOPA)):
        x_center     = LOPA[i,2]
        y_center     = LOPA[i,3] 
        s_l          = LOPA[i,5]
        s_w          = LOPA[i,6] 
        F_c          = LOPA[i,7]       
        B_c          = LOPA[i,8] 
        E_c          = LOPA[i,9]
        seat         = LOPA[i,10] 
        Em_row       = LOPA[i,11]
        Gal_Lav      = LOPA[i,12]
        
        if E_c == 1.0: 
            if seat == 1.0: 
                x0_pt = x_center - s_l / 2
                x1_pt = x_center + s_l / 2
                y0_pt = y_center - s_w / 2
                y1_pt = y_center + s_w / 2
                
                if Em_row == 1.0:
                    
                    fig.add_shape(type="rect",
                        x0=x0_pt, y0=y0_pt, x1=x1_pt, y1=y1_pt,
                        line=dict(
                            color=economoy_seat_colors[0],
                            width=2,
                        ),
                        fillcolor= economoy_seat_colors[1],
                    )
                    
                else: 
                    fig.add_shape(type="rect",
                        x0=x0_pt, y0=y0_pt, x1=x1_pt, y1=y1_pt,
                        line=dict(
                            color=economoy_seat_colors[0],
                            width=2,
                        ),
                        fillcolor=economoy_seat_colors[2],
                    )
        if B_c == 1: 
            if seat == 1: 
                x0_pt = x_center - s_l / 2
                x1_pt = x_center + s_l / 2
                y0_pt = y_center - s_w / 2
                y1_pt = y_center + s_w / 2
                
                if Em_row == 1:
                    
                    fig.add_shape(type="rect",
                        x0=x0_pt, y0=y0_pt, x1=x1_pt, y1=y1_pt,
                        line=dict(
                            color=business_seat_colors[0],
                            width=2,
                        ),
                        fillcolor=business_seat_colors[1],
                    )
                    
                else:
                    
                    fig.add_shape(type="rect",
                        x0=x0_pt, y0=y0_pt, x1=x1_pt, y1=y1_pt,
                        line=dict(
                            color=business_seat_colors[0],
                            width=2,
                        ),
                        fillcolor=business_seat_colors[2],
                    )                    
    
        if F_c == 1: 
            if seat == 1: 
                x0_pt = x_center - s_l / 2
                x1_pt = x_center + s_l / 2
                y0_pt = y_center - s_w / 2
                y1_pt = y_center + s_w / 2
                
                if Em_row == 1:
                    
                    fig.add_shape(type="rect",
                        x0=x0_pt, y0=y0_pt, x1=x1_pt, y1=y1_pt,
                        line=dict(
                            color=first_seat_colors[0],
                            width=2,
                        ),
                        fillcolor=first_seat_colors[1],
                    )
                    
                else:
                    
                    fig.add_shape(type="rect",
                        x0=x0_pt, y0=y0_pt, x1=x1_pt, y1=y1_pt,
                        line=dict(
                            color=first_seat_colors[0],
                            width=2,
                        ),
                        fillcolor=first_seat_colors[2],
                    )    
     
        if Gal_Lav == 1:
            x0_pt = x_center - s_l / 2
            x1_pt = x_center + s_l / 2
            y0_pt = y_center - s_w / 2
            y1_pt = y_center + s_w / 2
             
            fig.add_shape(type="rect",
                x0=x0_pt, y0=y0_pt, x1=x1_pt, y1=y1_pt,
                line=dict(
                    color=lavatory_color[0],
                    width=2,
                ),
                fillcolor=lavatory_color[0],
            )
    fig.show()
    
    return 