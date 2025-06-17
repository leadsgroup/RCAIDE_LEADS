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
from RCAIDE.Library.Methods.Geometry.Planform import compute_span_location_from_chord_length
from RCAIDE.Library.Methods.Geometry.Planform import compute_chord_length_from_span_location  
from RCAIDE.Library.Components.Component      import Container 

# package imports 
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------
#  Computer Aircraft Center of Gravity
# ----------------------------------------------------------------------------------------------------------------------   
def compute_vehicle_center_of_gravity(vehicle, nose_load = 0.06, update_CG=True): 
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
    for fuse in vehicle.fuselages:
        fuse.mass_properties.center_of_gravity[0][0] = .45*fuse.lengths.total
    
    #---------------------------------------------------------------------------------        
    # Wings
    #---------------------------------------------------------------------------------
    for wing in vehicle.wings:  
        if isinstance(wing,C.Wings.Main_Wing) or isinstance(wing,C.Wings.Blended_Wing_Body):
            wing.mass_properties.center_of_gravity[0][0] = .05*wing.chords.mean_aerodynamic +wing.aerodynamic_center[0]           
            
        elif isinstance(wing,C.Wings.Horizontal_Tail):
            chord_length_h_tail_35_percent_semi_span  = compute_chord_length_from_span_location(wing,.35*wing.spans.projected*.5)
            h_tail_35_percent_semi_span_offset        = np.tan(wing.sweeps.quarter_chord)*.35*.5*wing.spans.projected   
            wing.mass_properties.center_of_gravity[0][0] = .3*chord_length_h_tail_35_percent_semi_span + \
                                                                          h_tail_35_percent_semi_span_offset            

        elif isinstance(wing,C.Wings.Vertical_Tail):
            chord_length_v_tail_35_percent_semi_span  = compute_chord_length_from_span_location(wing,.35*wing.spans.projected)
            v_tail_35_percent_semi_span_offset        = np.tan(wing.sweeps.quarter_chord)*.35*.5*wing.spans.projected
            wing.mass_properties.center_of_gravity[0][0] = .3*chord_length_v_tail_35_percent_semi_span + \
                                                                        v_tail_35_percent_semi_span_offset
        else:
            span_location_mac = compute_span_location_from_chord_length(wing, wing.chords.mean_aerodynamic)
            mac_le_offset     = np.tan(wing.sweeps.leading_edge)*span_location_mac 
            wing.mass_properties.center_of_gravity[0][0] = .3*wing.chords.mean_aerodynamic + mac_le_offset

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
    # Cabin (Includes Systems,Operating Items)
    #---------------------------------------------------------------------------------
    for fuselage in vehicle.fuselages:
        for cabin in fuselage.cabins:
            cabin.origin[0][0] = fuselage.lengths.nose 
            num_seats  = cabin.number_of_passengers
            cabin_mass = cabin.mass_properties.mass
            if len(cabin.layout_of_passenger_accommodations) == 0:
                cabin.mass_properties.center_of_gravity[0][0] = 0.51 * length_scale
            else:
                LOPA       = cabin.layout_of_passenger_accommodations.object_coordinates
                point_mass = cabin_mass/num_seats
                cg_x       = (LOPA[:,2]*point_mass)/cabin_mass
                cg_y       = (LOPA[:,3]*point_mass)/cabin_mass
                cg_z       = (LOPA[:,4]*point_mass)/cabin_mass  
                cabin.mass_properties.center_of_gravity = [[cg_x, cg_y, cg_z]] 
            
    for wing in  vehicle.wings:
        if isinstance(wing, C.Wings.Blended_Wing_Body):
            for cabin in wing.cabins:
                cabin.origin[0][0] = wing.lengths.nose 
                num_seats  = cabin.number_of_passengers
                cabin_mass = cabin.mass_properties.mass
                if len(cabin.layout_of_passenger_accommodations) == 0:
                    cabin.mass_properties.center_of_gravity[0][0] = 0.51 * length_scale
                else:
                    LOPA       = cabin.layout_of_passenger_accommodations.object_coordinates
                    point_mass = cabin_mass/num_seats
                    cg_x       = (LOPA[:,2]*point_mass)/cabin_mass
                    cg_y       = (LOPA[:,3]*point_mass)/cabin_mass
                    cg_z       = (LOPA[:,4]*point_mass)/cabin_mass  
                    cabin.mass_properties.center_of_gravity = [[cg_x, cg_y, cg_z]] 

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
    total_moment = np.array([[0.0,0.0,0.0]])
    total_mass   = 0

    for key in vehicle.keys():
        item = vehicle[key]
        if isinstance(item,Component.Container):
            Moment, Mass  = sum_moment(item)
            if Mass != 0: 
                total_moment += Moment
                total_mass   += Mass         
    
    if update_CG and total_mass != 0.0:
        vehicle.mass_properties.center_of_gravity = total_moment/total_mass 
     
    return vehicle.mass_properties.center_of_gravity, total_mass 
