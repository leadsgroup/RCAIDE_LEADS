# RCAIDE/Library/Methods/Geometry/LOPA/LOPA_functions.py
# 
# 
# Created:  Mar 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  get_seat_y_coords
# ----------------------------------------------------------------------------------------------------------------------  
def get_seat_y_coords(cabin,cabin_class,cabin_class_origin): 

    n      = cabin_class.number_of_seats_abrest
    s_w    = cabin_class.seat_width
    ar_w   = cabin_class.seat_arm_rest_width 
    n      = cabin_class.number_of_seats_abrest 
    s_w    = cabin_class.seat_width
    ar_w   = cabin_class.seat_arm_rest_width  
    a_w    = cabin_class.aile_width      

    # determine number of aisles
    if n > 7:
        n_a = 1
    else:
        n_a = 2
        
    # max cabin class width
    total_arm_rest_width =  ((n_a + 1) + n) * ar_w
    total_seat_width     = n * s_w  
    total_aisle_width    = n_a * a_w
    cabin_class_width    = total_arm_rest_width + total_seat_width + total_aisle_width 
    cabin_class.cabin_class_width  = cabin_class_width
    
    ccw    = cabin_class.cabin_class_width
     
    if n == 1:
        s_y_coord = ccw/ 2 - ar_w - s_w /2 
    elif n == 2:
        y_1 = ccw/ 2 - ar_w - s_w /2 
        y_2 = -y_1
        s_y_coord = np.array([y_1, y_2 ])
    elif n == 3:
        y_1 =  ccw / 2 - s_w /2 - ar_w
        y_2 = -y_1 
        y_3 =  y_1 - s_w - ar_w  
        s_y_coord = np.array([ y_1, y_2, y_3 ])  
    elif n == 4:
        if cabin.wide_body:
            y_1 = ccw / 2 - s_w /2 - ar_w   
            y_2 = -y_1    
            y_3 =  s_w /2 + ar_w
            y_4 = -y_3   
            s_y_coord = np.array([ y_1, y_2, y_3,y_4 ])         
        else:
            y_1 = ccw / 2 - s_w /2 - ar_w   
            y_2 = -y_1    
            y_3 = y_1 - s_w - ar_w    
            y_4 = -y_3   
            s_y_coord = np.array([ y_1, y_2, y_3,y_4 ]) 
        
    elif n == 5: 
        y_1 =  ccw / 2 - s_w /2 - ar_w
        y_2 =  -y_1 
        y_3 =  y_1 - s_w - ar_w 
        y_4 =  -y_3
        y_5 =  y_4 + s_w + ar_w  
        s_y_coord = np.array([ y_1, y_2, y_3,y_4,y_5 ])  

    elif n == 6: 
        y_1 =  ccw / 2 - s_w /2 - ar_w
        y_2 =  -y_1 
        y_3 =  y_1 - s_w - ar_w 
        y_4 =  -y_3 
        y_5 =  y_3 - s_w - ar_w 
        y_6 = -y_5   
        s_y_coord = np.array([ y_1, y_2, y_3,y_4, y_5,y_6 ])  

    elif n == 7: 
        y_1 =  ccw / 2 - s_w /2 - ar_w
        y_2 =  -y_1 
        y_3 =  y_1 - s_w - ar_w 
        y_4 =  -y_3 
        y_5 =  s_w + ar_w  
        y_6 =  -y_5 
        y_7 = 0 
        s_y_coord = np.array([ y_1, y_2, y_3,y_4, y_5,y_6,y_7]) 

    elif n == 8:
        y_1 =  ccw / 2 - s_w /2 - ar_w
        y_2 =  -y_1 
        y_3 =  y_1 - s_w - ar_w 
        y_4 =  -y_3 
        y_5 = s_w /2 + ar_w
        y_6 = -y_5 
        y_7 = y_5 + s_w + ar_w 
        y_8 = -y_7  
        
        s_y_coord = np.array([ y_1, y_2, y_3,y_4, y_5,y_6,y_7,y_8 ]) 

    elif n == 9: 
        y_1 =  ccw / 2 - s_w /2 - ar_w
        y_2 =  -y_1 
        y_3 =  y_1 - s_w - ar_w 
        y_4 =  -y_3 
        y_5 =  y_3 - s_w - ar_w  
        y_6 = -y_5 
        y_7 =  s_w + ar_w  
        y_8 =  -y_7 
        y_9 = 0 
        s_y_coord = np.array([ y_1, y_2, y_3,y_4, y_5,y_6,y_7,y_8,y_9])

    elif n == 10:
        y_1 =  ccw / 2 - s_w /2 - ar_w
        y_2 =  -y_1 
        y_3 =  y_1 - s_w - ar_w 
        y_4 =  -y_3 
        y_5 =  y_3 - s_w - ar_w  
        y_6 = -y_5 
        y_7 = s_w /2 + ar_w
        y_8 = -y_7 
        y_9 = y_7 + s_w + ar_w 
        y_10 = -y_9 
        s_y_coord = np.array([ y_1, y_2, y_3,y_4, y_5,y_6,y_7,y_8,y_9,y_10 ])
    
    cabin.width = 2 * (np.max(s_y_coord) + s_w /2 + ar_w)
    s_y_coord  += cabin_class_origin[1] 
    return s_y_coord , cabin_class_origin

# ----------------------------------------------------------------------------------------------------------------------
#  get_seat_x_coords
# ----------------------------------------------------------------------------------------------------------------------  
def get_seat_x_coords(cabin,cabin_class,cabin_class_origin): 
 
    n_r     = cabin_class.number_of_rows
    s_p     = cabin_class.seat_pitch 
    gl_l    = cabin.galley_lavatory_length 
    A_l     = cabin.type_A_door_length       
    ex_p    = cabin.emergency_exit_seat_pitch       
    gl_loc  = cabin_class.galley_lavatory_percent_x_locations         
    ex_loc  = cabin_class.emergency_exit_percent_x_locations 
    A_loc   = cabin_class.type_A_exit_percent_x_locations  
    
    # place seats
    s_x_coord = np.arange(0, n_r) * (s_p)
    normalized_s_x_coord = s_x_coord / s_x_coord[-1]
    
    # create vector of object types (seat,emergency exit,galley/lav,type-A exit)
    object_type = np.zeros((len(s_x_coord),4)) 
    object_type[:,0] =  1 # assign all seats 
    
    # shift for galley and lavatories 
    for i in range(len(gl_loc)):
        var =  normalized_s_x_coord -gl_loc[i]
        loc = np.argmin(abs(var))
        if loc == (n_r -1):
            new_loc =  s_x_coord[-1]  + gl_l 
            s_x_coord = np.append( s_x_coord,new_loc  )
            object_type = np.vstack((object_type, np.array([0, 0, 1, 0])   ))
        else:
            object_type = np.insert(object_type,loc, np.array([0, 0, 1, 0]) , axis=0)
            s_x_coord[loc:] +=  gl_l
            s_x_coord   = np.insert(s_x_coord, loc, s_x_coord[loc] - gl_l)
            if object_type[loc+1, 0] == 1: 
                s_x_coord[loc+1:] +=  s_p / 2
            
    # shift for type A exit
    for i in range(len(A_loc)):
        var = normalized_s_x_coord -A_loc[i]
        loc = np.argmin(abs(var))
        if loc == (n_r -1):
            new_loc =  s_x_coord[-1]  + A_l 
            s_x_coord = np.append( s_x_coord,new_loc  )
            object_type = np.vstack((object_type, np.array([0, 0, 0, 1])   ))
        else:
            object_type = np.insert(object_type,loc, np.array([0, 0, 0, 1]) , axis=0) 
            s_x_coord[loc:] += A_l
            s_x_coord   = np.insert(s_x_coord, loc, s_x_coord[loc] - A_l)
            #if object_type[loc+1, 0] == 1: 
                #s_x_coord[loc+1:] += s_p / 2 
            
    # shift for emergency rows
    for i in range(len(ex_loc)):
        var =  normalized_s_x_coord -ex_loc[i]
        loc =  np.argmin(abs(var))
        if loc == (n_r-1):
            if object_type[-1,1] == 1:
                object_type[-2,1] = 1
            else:
                object_type[-1,1] = 1 
        else:
            if object_type[loc,1] == 1:
                object_type[loc+1,1] = 1   
                s_x_coord[loc+1:] += (ex_p - s_p)  
            else:
                object_type[loc,1] = 1   
                s_x_coord[loc:] += (ex_p - s_p)  
             
    if object_type[0, 0] == 1:
        offset = s_p / 2
    if object_type[0, 2] == 1:
        offset = gl_l / 2
    if object_type[0, 3] == 1:
        offset = A_l / 2            
    s_x_coord += offset
      
    if object_type[-1, 0] == 1:
        offset_end =   s_p / 2  
    if object_type[-1, 2] == 1:
        offset_end=   gl_l / 2  
    if object_type[-1, 3] == 1: 
        offset_end = A_l / 2
     
    cabin_class.length =  s_x_coord[-1] + offset_end
    cabin.length += s_x_coord[-1] + offset_end
    s_x_coord += cabin_class_origin[0]
    cabin_class_origin[0] = s_x_coord[-1] + offset_end
    return s_x_coord , object_type, cabin_class_origin
 

# ----------------------------------------------------------------------------------------------------------------------
#  update_seat_map_layout_using_cabin_taper
# ----------------------------------------------------------------------------------------------------------------------  
def update_seat_map_layout_using_cabin_taper(seat_data,cabin):
    
    cabin_width = cabin.width
    n_fr        = cabin.nose.fineness_ratio 
    nose_length = cabin.width * n_fr
    t_fr        = cabin.tail.fineness_ratio 
    tail_length = cabin.width * t_fr 
    
    # remove components that fall outside of tapered nose region 
    theta1 = np.arctan(cabin_width/nose_length) 
    n_idxs =  np.where(nose_length > seat_data[:,2])[0]
    removed_indexes =  [] 
    for n_i in  range(len(n_idxs)):
        x0    = seat_data[n_i,2] - seat_data[n_i,5]/2
        x1    = seat_data[n_i,2] + seat_data[n_i,5]/2
        y0    = seat_data[n_i,3] - seat_data[n_i,6]/2
        y1    = seat_data[n_i,3] + seat_data[n_i,6]/2
        
        x_pts = np.array([x0, x1])
        y_pts = np.array([y0, y1])
        
        y_border = max(x_pts * np.tan(theta1))
        
        if np.any( y_pts > y_border):
            removed_indexes.append(n_idxs[n_i])
    seat_data = np.delete(seat_data,(removed_indexes), axis=0)
            
        
    # remove components that fall outside of tapered tail region 
    theta2     = np.arctan(cabin_width/tail_length) 
    tail_start = seat_data[-1,2] -  tail_length 
    t_idxs     = np.where(tail_start < seat_data[:,2])[0]
    removed_indexes =  []
    for t_i in  range(len(t_idxs)):
        x0    = seat_data[t_i,2] - seat_data[t_i,5]/2
        x1    = seat_data[t_i,2] + seat_data[t_i,5]/2
        y0    = seat_data[t_i,3] - seat_data[t_i,6]/2
        y1    = seat_data[t_i,3] + seat_data[t_i,6]/2
        
        x_pts = np.array([x0, x1])
        y_pts = np.array([y0, y1])
        
        y_border = max(x_pts * np.tan(theta2)) 
        if np.any( y_pts > y_border): # or remove_feature == True:
            removed_indexes.append(t_idxs[t_i])
    seat_data = np.delete(seat_data,(removed_indexes), axis=0)
            
    return seat_data