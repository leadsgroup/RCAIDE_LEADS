# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_cabin_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
from RCAIDE.Library.Methods.Geometry.LOPA.compute_layout_of_passenger_accommodations import compute_layout_of_passenger_accommodations

# package imports 
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Cabin Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_cabin_center_of_gravity(cabin,comp):

    if comp.layout_of_passenger_accommodations == None:
        compute_layout_of_passenger_accommodations(comp)
        
    num_seats          = cabin.number_of_seats
    num_pax            = cabin.number_of_passengers
    cabin_mass         = cabin.mass_properties.mass
    arr                = cabin.filled_seats_arrangement 
    LOPA               = cabin.layout_of_passenger_accommodations.object_coordinates
    point_mass         = cabin_mass/num_pax 
    if arr == 'random':
        idxs =  np.random.choice(range(0, num_seats), size=num_pax, replace=False)
    elif arr == 'ascending':
        idxs = np.arange(0,num_pax) 
    elif  arr == 'descending':
        idxs = np.arange(num_seats-num_pax, num_seats)  
    
    # Apply the mask to filter seats 
    seat_mask = LOPA[:, 10] == 1
    LOPA_seats = LOPA[seat_mask]

    # make sure lopa is storted by x-value of seats
    sorted_indices = LOPA_seats[:, 2].argsort()
    LOPA_sorted_seats    = LOPA_seats[sorted_indices] 
    
    # find center of gravity 
    cg_x       = np.sum(LOPA_sorted_seats[idxs,2]*point_mass)/cabin_mass
    cg_y       = 0
    cg_z       = np.sum(LOPA_sorted_seats[idxs,4]*point_mass)/cabin_mass                    
    cabin.mass_properties.center_of_gravity = [[cg_x, cg_y, cg_z]]
    return cabin.mass_properties.center_of_gravity