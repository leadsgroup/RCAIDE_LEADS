# RCAIDE/Library/Missions/Common/Pre_Process/geometry.py
# 
# 
# Created:  Apr 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Library.Methods.Geometry.LOPA      import  compute_layout_of_passenger_accommodations
from RCAIDE.Library.Methods.Geometry.Planform  import  fuselage_planform, wing_planform, bwb_wing_planform , compute_fuel_volume 
# python imports
from copy import deepcopy
import  numpy as  np 
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
    for i ,  segment in enumerate(mission.segments): 
        # --------------------------------------------------------------------------------------------------------------------        
        # check if geometry analysis is defined 
        # --------------------------------------------------------------------------------------------------------------------
        if segment.analyses.geometry is None: 
            raise AssertionError('Geometry Analyses not defined') 
        if i == 0 or segment.analyses.geometry.settings.unique_geometry: 
            geometry_preprocess_routine(segment.analyses.geometry.settings, segment.analyses.vehicle)
            
        else:
            vehicle_0 = deepcopy(segment.analyses.vehicle)
            segment.analyses.vehicle = deepcopy(mission.segments[i-1].analyses.vehicle)
            for wing in segment.analyses.vehicle.wings:
                for control_surface in wing.control_surfaces:
                    control_surface.deflection = vehicle_0.wings[wing.tag].control_surfaces[control_surface.tag].deflection
            for landing_gear in segment.analyses.vehicle.landing_gears:
                landing_gear.gear_extended = vehicle_0.landing_gears[landing_gear.tag].gear_extended
            
            for network in segment.analyses.vehicle.networks: 
                for bus in network.busses:
                    bus.active = vehicle_0.networks[network.tag].busses[bus.tag].active
                for propulsor in network.propulsors:
                    if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan):
                        propulsor_0    = vehicle_0.networks[network.tag].propulsors[propulsor.tag] 
                        fan_0           = network.converters[propulsor_0.assigned_converters.fan_tag[0][0]] 
                        core_nozzle_0   = network.converters[propulsor_0.assigned_converters.core_nozzle_tag[0][0]]
                        core_nozzle     = network.converters[propulsor.assigned_converters.core_nozzle_tag[0][0]]
                        fan_nozzle_0    = network.converters[propulsor_0.assigned_converters.fan_nozzle_tag[0][0]] 
                        fan_nozzle      = network.converters[propulsor.assigned_converters.fan_nozzle_tag[0][0]]    
                        fan_0           = network.converters[propulsor_0.assigned_converters.fan_tag[0][0]]     
                        fan             = network.converters[propulsor.assigned_converters.fan_tag[0][0]]              
             
                        fan.angular_velocity        = fan_0.angular_velocity        
                        fan_nozzle.exit_velocity    = fan_nozzle_0.exit_velocity 
                        core_nozzle.exit_velocity   = core_nozzle_0.exit_velocity
                        
                    if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor):
                        propulsor_0 =  vehicle_0.networks[network.tag].propulsors[propulsor.tag]  
                        rotor_0   = network.converters[propulsor_0.assigned_converters.rotor_tag[0][0]]     
                        rotor     = network.converters[propulsor.assigned_converters.rotor_tag[0][0]]     
                        rotor.orientation_euler_angles  = rotor_0.orientation_euler_angles 
                        rotor.blade_pitch_command       = rotor_0.blade_pitch_command                 
                                  
    return 
        
def geometry_preprocess_routine(settings, vehicle):   
    
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
                cabin.number_of_passengers = int((cabin.number_of_seats / total_seats) *  vehicle.number_of_passengers)
            
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
            bwb_wing_planform(wing)
            compute_layout_of_passenger_accommodations(wing)
            
            # update reference properties 
            if settings.overwrite_reference:
                vehicle.reference_area = wing.areas.reference
                vehicle.LEMAC          = wing.LEMAC 
     
            for cabin in wing.cabins: 
                defined_cabins = True
                for cabin_class in cabin.classes: 
                    cabin.number_of_passengers += cabin_class.number_of_passengers
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
     
    # --------------------------------------------------------------------------------------------------------------------
    # Compute fuel volume  
    # -------------------------------------------------------------------------------------------------------------------- 
    compute_fuel_volume(vehicle,compute_fuel_volume = settings.compute_fuel_volume, update_max_fuel=settings.update_max_fuel)
               
    return 