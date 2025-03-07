# RCAIDE/Library/Methods/Geometry/LOPA/LOPA_functions.py
# 
# 
# Created: Mar 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------     

import RCAIDE 
from .LOPA_functions import *

import  numpy as  np 
from copy import  deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  compute_layout_of_passenger_accommodations
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_layout_of_passenger_accommodations(fuselage):  
    #  [ x, y, z , length, width, first-cl flag, business-cl flag, economy-cl flag, seat, emergency-row flag, galley/lav flag, type-A exit flag]    
    LOPA = np.empty(( 0, 14)) 
    side_cabin_offset = 0
    for cabin in fuselage.cabins: 
        cabin_class_origin  = [0, 0, 0]
        for cabin_class in cabin.classes: 
            seat_data ,cabin_class_origin  = create_class_seating_map_layout(cabin, cabin_class,cabin_class_origin, side_cabin_offset)
            side_cabin_offset =  cabin.width / 2
            LOPA = np.vstack((LOPA,seat_data))
    
    fuselage.number_of_passengers 
    fuselage.layout_of_passenger_accommodations =  LOPA
    return  
 
def create_class_seating_map_layout(cabin,cabin_class,cabin_class_origin, side_cabin_offset):  
    
    s_y_coord, cabin_class_origin = get_seat_y_coords(cabin, cabin_class,cabin_class_origin)
    s_x_coord,object_type, cabin_class_origin = get_seat_x_coords(cabin, cabin_class,cabin_class_origin) 
         
    # concatenate arrays 
    length   =  cabin_class.seat_length * np.ones_like(s_x_coord) 
    length[object_type[:,2] == 1] = cabin.galley_lavatory_length
    length[object_type[:,3] == 1] = cabin.type_A_door_length
    
    X_coords   = np.atleast_2d((np.tile(s_x_coord[:,None], (1, len(s_y_coord)))).flatten()).T
    Y_coords   = np.atleast_2d((np.tile(s_y_coord[None,:], (len(s_x_coord), 1))).flatten()).T
    Z_coords   = np.atleast_2d((np.zeros_like(Y_coords)).flatten()).T 
    length     = np.atleast_2d((np.tile(length[:,None], (1, len(s_y_coord)))).flatten()).T 
    width      = cabin_class.seat_width * np.ones_like(Z_coords) 
    n_rows     = cabin_class.number_of_rows * np.ones_like(Z_coords) 
    n_seats_y  = cabin_class.number_of_seats_abrest  * np.ones_like(Z_coords) 
    object_vec = np.repeat(object_type, len(s_y_coord), axis=0)
    
    
    # cabin class flags 
    F_c  =  np.zeros_like(Z_coords)
    B_c  =  np.zeros_like(Z_coords)
    E_c  =  np.zeros_like(Z_coords)     
    if type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.First: 
        F_c  =  np.ones_like(Z_coords) 
    elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business: 
        B_c  =  np.ones_like(Z_coords) 
    elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy: 
        E_c  =  np.ones_like(Z_coords)  
    
    # [n_rows , n_seats_y, x, y, z , length, width, first-cl flag, business-cl flag, economy-cl flag, seat, emergency-row flag, galley/lav flag, type-A exit flag]    
    seat_data = np.hstack((n_rows , n_seats_y, X_coords,Y_coords, Z_coords, length ,width,F_c,B_c,E_c,object_vec))
     
    if type(cabin) == RCAIDE.Library.Components.Fuselages.Cabins.Side_Cabin:
        
        seat_data[:, 3] += cabin.width / 2
        seat_data  = update_seat_map_layout_using_cabin_taper(seat_data,cabin) 
        seat_data[:, 3] += side_cabin_offset  
        
        # make copy about center
        seat_data_        = deepcopy(seat_data)
        seat_data_[:, 3] *= -1 
        seat_data         = np.vstack((seat_data,seat_data_))
        
    return seat_data ,cabin_class_origin 
    
    