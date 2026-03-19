# RCAIDE/Library/Missions/Common/Pre_Process/geometry.py
# 
# 
# Created:  Apr 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Library.Methods.Geometry.LOPA      import  compute_layout_of_passenger_accommodations
from RCAIDE.Library.Methods.Geometry.Planform  import  fuselage_planform, wing_planform , compute_fuel_volume 

# python imports 
import  numpy as  np
from copy import  deepcopy
# ----------------------------------------------------------------------------------------------------------------------
#  geometry
# ----------------------------------------------------------------------------------------------------------------------  
def geometry(mission):
    """
    Initializes and processes geometry for mission segments

    Parameters
    ----------
    mission : Mission
        The mission containing segments to be analyzed
            - analyses.geometry : Analysis
                Stability analysis module
                - vehicle : Vehicle
                    Aircraft geometry definition
                    - wings : list
                        Wing geometry definitions
                - process.compute.lift.inviscid_wings : Process
                    Lift computation process
                - surrogates : Data
                    Stability surrogate models 
        
    
    Returns
    -------
    None
        Updates mission segment analyses directly
    
    Notes
    -----
    This function prepares the geometry for each mission segment 
    
    See Also
    --------
    RCAIDE.Library.Methods.Geometry.Planform
    RCAIDE.Framework.Mission.Segments
    """

    config_tags  = []
    segment_idxs = []
    
    # preprocess geometry of aircraft planform 
    for i, segment in enumerate(mission.segments):
        config_tag = segment.analyses.vehicle.tag 
        if segment.analyses.geometry is None: 
            raise AssertionError('Geometry Analyses not defined') 
        if config_tag not in config_tags: 
            planform_preprocess_routine(segment.analyses)
            config_tags.append(config_tag)
            segment_idxs.append(i) 
        else:   
            list_idx    = config_tags.index(config_tag)
            segment_idx = segment_idxs[list_idx]
            segment.analyses.vehicle = deepcopy(mission.segments[segment_idx].analyses.vehicle)
            
    # preprocess geometry of fuel tanks, since liquid hydrogen tank sizing take a while, we will only preprocess them once (i.e. the first segment)      
    for i, segment in enumerate(mission.segments):
        if i == 0: 
            powertrain_preprocess_routine(segment.analyses)
        else:
            for network in segment.analyses.vehicle.networks:
                for fuel_line in network.fuel_lines:
                    for fuel_tank in fuel_line.fuel_tanks: 
                        segment.analyses.vehicle.networks[network.tag].fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag] = deepcopy(mission.segments[0].analyses.vehicle.networks[network.tag].fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag])
                
        
    return

def powertrain_preprocess_routine(analyses):

    settings = analyses.geometry.settings
    vehicle  = analyses.vehicle        
    compute_fuel_volume(vehicle,compute_fuel_volume = settings.compute_fuel_volume, update_max_fuel=settings.update_max_fuel)
    
    return     
            
def planform_preprocess_routine(analyses):
    settings = analyses.geometry.settings
    vehicle  = analyses.vehicle
    
    # initalize variables 
    A_fuselage     = 0
    defined_cabins = False 
    NPF            = 0
    NPB            = 0
    NPE            = 0    
    # ================================================================================================================================================
    # update fuselage properties
    # ================================================================================================================================================  
    total_seats = 0
    for fuselage in vehicle.fuselages: 
        compute_layout_of_passenger_accommodations(fuselage) 
        fuselage_planform(fuselage) 
        vehicle.length = np.maximum(vehicle.length, fuselage.lengths.total)
        A_fuselage     = np.maximum(A_fuselage,fuselage.areas.front_projected) 
        
        for cabin in fuselage.cabins: 
            defined_cabins = True
            for cabin_class in cabin.classes:  
                if type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy:
                    NPE +=  cabin_class.number_of_seats 
                elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business: 
                    NPB +=  cabin_class.number_of_seats 
                elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.First:
                    NPF +=  cabin_class.number_of_seats  
            total_seats += cabin.number_of_seats 
        for cabin in fuselage.cabins:     
            if cabin.number_of_passengers == 0: # if cabin class  passengers are not defined, use ratio of cabin to aircraft
                cabin.number_of_passengers = min(total_seats,int((cabin.number_of_seats / total_seats) *  vehicle.number_of_passengers))
            
    # update landing gear properties 
    for landing_gear in  vehicle.landing_gears:
        if (landing_gear.number_of_gear_types_in_tandem != None) and  (landing_gear.number_of_wheels_in_gear_type != None):
            landing_gear.wheels = landing_gear.number_of_gear_types_in_tandem * landing_gear.number_of_wheels_in_gear_type
            if landing_gear.xz_plane_symmetric:
                landing_gear.wheels *= 2
                
    vehicle.maximum_cross_sectional_area  =  A_fuselage
    
    # ================================================================================================================================================
    # update wing properties 
    # ================================================================================================================================================
    for wing in vehicle.wings:  
        # --------------------------------------------------------------------------------------------------------------------
        #  Blended Wing Body
        # --------------------------------------------------------------------------------------------------------------------
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):            
            # compute planform properties 
            wing_planform(wing)
            compute_layout_of_passenger_accommodations(wing)
            
            # update reference properties 
            if settings.overwrite_reference:
                vehicle.reference_area = wing.areas.reference
                vehicle.LEMAC          = wing.LEMAC 
     
            for cabin in wing.cabins: 
                defined_cabins = True
                for cabin_class in cabin.classes:  
                    if type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy:
                        NPE +=  cabin_class.number_of_seats
                    elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business:
                        NPB +=  cabin_class.number_of_seats
                    elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.First:
                        NPF +=  cabin_class.number_of_seats 
                total_seats += cabin.number_of_seats 
            for cabin in wing.cabins:     
                if cabin.number_of_passengers == 0: # if cabin class  passengers are not defined, use ratio of cabin to aircraft
                    cabin.number_of_passengers = int((cabin.number_of_seats / total_seats) *  vehicle.number_of_passengers)
                        
        # --------------------------------------------------------------------------------------------------------------------
        # All other wing surfaces
        # --------------------------------------------------------------------------------------------------------------------
        else: 
            wing_planform(wing)                   
            if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing) and settings.overwrite_reference:
                vehicle.reference_area = wing.areas.reference
                vehicle.LEMAC          = wing.LEMAC 
             
        # reference chord 
        vehicle.reference_chord  = np.maximum(vehicle.reference_chord , wing.chords.mean_aerodynamic)
        
        # reference span 
        vehicle.reference_span   = np.maximum(vehicle.reference_span  , wing.spans.projected)
        
        # total length 
        vehicle.length = np.maximum(vehicle.length, wing.chords.root)                         
        
        # max cross sectional area 
        A_wing_plus_fuselage   = wing.spans.projected * wing.thickness_to_chord *  wing.chords.root +  A_fuselage
        vehicle.maximum_cross_sectional_area = np.maximum(vehicle.maximum_cross_sectional_area,A_wing_plus_fuselage) 

    # --------------------------------------------------------------------------------------------------------------------
    # Update passenger imformation 
    # --------------------------------------------------------------------------------------------------------------------
    if  vehicle.number_of_passengers == 0:
        pass 
    else:   
        if defined_cabins:
            vehicle.number_of_first_class_seats    = NPF
            vehicle.number_of_business_class_seats = NPB
            vehicle.number_of_economy_class_seats  = NPE
        else:  
            vehicle.number_of_first_class_seats    = vehicle.number_of_passengers / 20.
            vehicle.number_of_business_class_seats = vehicle.number_of_passengers / 10.
            vehicle.number_of_economy_class_seats  = vehicle.number_of_passengers - NPF - NPB 
    return 