# RCAIDE/Methods/Stability/Center_of_Gravity/compute_vehicle_center_of_gravity.py 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports   
import RCAIDE 
from RCAIDE.Library.Components import Component   
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.mass_and_intertia_functions import *   

# package imports 
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------
#  Computer Aircraft Center of Gravity
# ----------------------------------------------------------------------------------------------------------------------   
def compute_vehicle_center_of_gravity(vehicle , update_center_of_gravity=True): 
    ''' Computes the moment of intertia of aircraft 
    
    Source:
    Simplified Mass and Inertial Estimates for Aircraft with Components of Constant Density
    Moulton, B. C., and Hunsaker, D. F., “Simplified Mass and Inertial Estimates for Aircraft with Components 
    of Constant Density,” AIAA SCITECH 2023 Forum, January 2023, AIAA-2023-2432 DOI: 10.2514/
    6.2023-2432
    
    
    Assumtions:
    Assumes simplified shapes 
    
    Inputs:
    vehicle           - vehicle data structure           [m]
    
    Outputs:
    I                 - mass moment of inertia matrix    [kg-m^2]
    
    '''
     
    C =  RCAIDE.Library.Components

    length_scale = 1.
    nose_length  = 0.
      
    if len(vehicle.fuselages) == 0.:
        for wing in vehicle.wings:
            if isinstance(wing,C.Wings.Main_Wing):
                if isinstance(wing,C.Wings.Blended_Wing_Body): 
                    length       = wing.chords.root
                    length_scale = length 
                b = wing.chords.root
                if b>length_scale:
                    length_scale = b
                    nose_length  = 0.25*b
    else:
        for fuse in vehicle.fuselages:
            nose   = fuse.lengths.nose
            length = fuse.lengths.total
            if length > length_scale:
                length_scale = length
                nose_length  = nose     

    #---------------------------------------------------------------------------------        
    # Fuselages (the frame only)
    #-------------------------------------------------------------------------------- 
    for fuselage in vehicle.fuselages:
        fuselage.mass_properties.center_of_gravity[0][0] = .51*fuselage.lengths.total
      
        for cabin in fuselage.cabins: 
            compute_cabin_center_of_gravity(cabin,fuselage,length_scale)   
    
    #---------------------------------------------------------------------------------        
    # Wings
    #---------------------------------------------------------------------------------
    for wing in vehicle.wings:    
        if isinstance(wing, C.Wings.Blended_Wing_Body):
            for cabin in wing.cabins:
                compute_cabin_center_of_gravity(cabin, wing,length_scale)        

    #---------------------------------------------------------------------------------
    # Landing Gear 
    #--------------------------------------------------------------------------------- 
    for landing_gear in vehicle.landing_gears:
        if isinstance(landing_gear, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            if landing_gear.origin[0][0] == 0:  
                landing_gear.origin[0][0]   = 0.51 * length_scale
                landing_gear.mass_properties.center_of_gravity[0][0]  = 0.0 
        elif isinstance(landing_gear, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):
            if landing_gear.origin[0][0] == 0: 
                landing_gear.origin[0][0]   = 0.25*nose_length 
                landing_gear.mass_properties.center_of_gravity[0][0]  = 0.0  
            
    #---------------------------------------------------------------------------------
    # Cargo Bays 
    #---------------------------------------------------------------------------------
    for cargo_bay in vehicle.cargo_bays:
        if cargo_bay.origin[0][0] == 0: 
            cargo_bay.origin[0][0] = 0.51 * length_scale  
        cargo_bay.mass_properties.center_of_gravity = [[cargo_bay.length / 2 ,0,cargo_bay.height / 2 ]] 
    
    #---------------------------------------------------------------------------------
    # Finally, compute aircraft center of gravity  
    #---------------------------------------------------------------------------------     
    # compute total aircraft center of grabity 
    aircraft_total_moment = np.array([[0.0,0.0,0.0]])
    aircraft_total_mass   = 0

    for key in vehicle.keys():
        item = vehicle[key]
        if isinstance(item,Component.Container):
            Moment = np.array([[0.0,0.0,0.0]])
            Mass   = 0
            Moment, Mass  = sum_moment(item, Mass, Moment) 
            aircraft_total_moment += Moment
            aircraft_total_mass   += Mass 
    
    if update_center_of_gravity and aircraft_total_mass != 0.0:
        CG = aircraft_total_moment/aircraft_total_mass 
        vehicle.mass_properties.center_of_gravity = CG.tolist() 
     
    return vehicle.mass_properties.center_of_gravity, aircraft_total_moment, aircraft_total_mass 

def compute_cabin_center_of_gravity(cabin, comp,length_scale):  
    num_seats          = cabin.number_of_seats
    num_pax            = cabin.number_of_passengers
    cabin_mass         = cabin.mass_properties.mass
    arr                = cabin.filled_seats_arrangement
    if comp.layout_of_passenger_accommodations == None:
        cabin.mass_properties.center_of_gravity[0][0] = 0.51 * length_scale
    else: 
        LOPA       = comp.layout_of_passenger_accommodations.object_coordinates
        point_mass = cabin_mass/num_pax 
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
    return 