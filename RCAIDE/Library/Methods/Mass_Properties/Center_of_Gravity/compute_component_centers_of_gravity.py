# RCAIDE/Methods/Stability/Center_of_Gravity/compute_component_centers_of_gravity.py
# 
# 
# Created:  Dec 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
import RCAIDE 
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.mass_and_intertia_functions import * 
from RCAIDE.Library.Methods.Geometry.Planform import compute_span_location_from_chord_length
from RCAIDE.Library.Methods.Geometry.Planform import compute_chord_length_from_span_location 
from RCAIDE.Library.Methods.Geometry.Planform import convert_sweep
from RCAIDE.Library.Components                import Component
from RCAIDE.Library.Components.Component      import Container 

# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Computer Aircraft Center of Gravity
# ----------------------------------------------------------------------------------------------------------------------  
def compute_component_centers_of_gravity(vehicle, nose_load = 0.06):
    """ computes the center of gravity of all of the vehicle components based on correlations 
    from Stanford University AA241 Lecture Notes 

    Assumptions:
    None

    Source:
    AA 241 Notes

    Inputs:
    vehicle

    Outputs:
    None

    Properties Used:
    N/A
    """  
    
    C =  RCAIDE.Library.Components

    # Go through all the fuselages
    for fuse in vehicle.fuselages:
        fuse.mass_properties.center_of_gravity[0][0]   = .45*fuse.lengths.total
        
    # Go through all wings
    for wing in vehicle.wings:
    
        if wing.sweeps.leading_edge == None:
            wing.sweeps.leading_edge = convert_sweep(wing,old_ref_chord_fraction = 0.25 ,new_ref_chord_fraction = 0.0)
        
        if isinstance(wing,C.Wings.Main_Wing):
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
            
            
    # Compute collective network center of gravity
    network_moment = 0.
    network_mass   = 0.
    for network in vehicle.networks:
        for p_tag, p_item in network.items():
            network_moment,network_mass = compute_properties(network_moment,network_mass,p_item)
        network.mass_properties.mass   = network_mass
        network.mass_properties.center_of_gravity = (network_moment / network_mass).tolist()
        
    if network_mass!= 0.:
        propulsion_cg = network_moment/network_mass
    else:
        propulsion_cg = np.array([[.45*fuse.lengths.total,0.,0.]]) 

    #---------------------------------------------------------------------------------
    # All other components
    #---------------------------------------------------------------------------------
     
    # Select a length scale depending on what kind of vehicle this is
    length_scale = 1.
    nose_length  = 0.
     
    # Check if there is a fuselage
    if len(vehicle.fuselages) == 0.:
        for wing in vehicle.wings:
            if isinstance(wing,C.Wings.Main_Wing):
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
                
    for landing_gear in vehicle.landing_gears:
        if isinstance(landing_gear, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            if landing_gear.origin[0][0] == 0:  
                landing_gear.origin[0][0]   = 0.51 * length
                landing_gear.mass_properties.center_of_gravity[0][0]  = 0.0 
        elif isinstance(landing_gear, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):
            if landing_gear.origin[0][0] == 0: 
                landing_gear.origin[0][0]   = 0.25*nose_length 
                landing_gear.mass_properties.center_of_gravity[0][0]  = 0.0 
                        
    # unpack all components:
    try: 
        avionics                                                   = vehicle.systems.avionics 
        avionics.origin[0][0]                                      = 0.4 * nose_length
        avionics.mass_properties.center_of_gravity[0][0]           = 0.0 
    except:
        pass 

    try:  
        furnishings                                                = vehicle.systems.furnishings
        furnishings.origin[0][0]                                   = 0.51 * length_scale
        furnishings.mass_properties.center_of_gravity[0][0]        = 0.0 
    except:
        pass     

    try:   
        apu                                                        = vehicle.systems.apu 
        apu.origin[0][0]                                           = 0.9 * length_scale   
        apu.mass_properties.center_of_gravity[0][0]                = 0.0
    except:
        pass         

    try:  
        passengers                                                 = vehicle.payload.passengers
        passengers.origin[0][0]                                    = 0.51 * length_scale  
        passengers.mass_properties.center_of_gravity[0][0]         = 0.0
    except:
        pass      

    try:  
        baggage                                                    = vehicle.payload.baggage
        baggage.origin[0][0]                                       = 0.51 * length_scale  
        baggage.mass_properties.center_of_gravity[0][0]            = 0.0 
    except:
        pass         

    try:  
        cargo                                                      = vehicle.payload.cargo
        cargo.origin[0][0]                                         = 0.51 * length_scale  
        cargo.mass_properties.center_of_gravity[0][0]              = 0.0    
        
    except:
        pass         
    

    try:  
        air_conditioner                                            = vehicle.systems.air_conditioner
        air_conditioner.origin[0][0]                               = nose_length
        air_conditioner.mass_properties.center_of_gravity[0][0]    = 0.0 
    except:
        pass         
    

    try:  
        optionals                                                  = vehicle.systems.optionals 
        optionals.origin[0][0]                                     = 0.51 * length_scale  
        optionals.mass_properties.center_of_gravity[0][0]          = 0.0    
    except:
        pass         
    

    try:  
        control_systems                                            = vehicle.systems.control_systems
        control_systems.origin[0][0]                               = vehicle.wings.main_wing.origin[0][0] 
        control_systems.mass_properties.center_of_gravity[0][0]    = vehicle.wings.main_wing.mass_properties.center_of_gravity[0][0] + \
            .1*vehicle.wings.main_wing.chords.mean_aerodynamic 
    except:
        pass       

    try:   
        electrical_systems                                         = vehicle.systems.electrical_systems
        electrical_systems.origin[0][0]                            = .75*(.5*length_scale) + propulsion_cg[0][0]*.25
        electrical_systems.mass_properties.center_of_gravity[0][0] = 0.0 
    except:
        pass         
    try:  
        hydraulics                                                 = vehicle.systems.hydraulics
        hydraulics.origin[0][0]                                    = .75*(vehicle.wings.main_wing.origin[0][0] + wing.mass_properties.center_of_gravity[0][0]) + 0.25* length_scale*.95
        hydraulics.mass_properties.center_of_gravity[0][0]         = 0.0       
    except:
        pass         
         
    
    return

def compute_properties(network_moment,network_mass,p_item):  
    if isinstance(p_item,Component): 
        network_moment += p_item.mass_properties.mass*(np.array(p_item.origin) + np.array( p_item.mass_properties.center_of_gravity))
        network_mass   += p_item.mass_properties.mass
        for p_sub_tag, p_sub_item in p_item.items():
            network_moment,network_mass =  compute_properties(network_moment,network_mass,p_sub_item)    
    elif isinstance(p_item,Container): 
        for p_sub_tag, p_sub_item in p_item.items():
            network_moment,network_mass =  compute_properties(network_moment,network_mass,p_sub_item)
    
    return network_moment,network_mass
        