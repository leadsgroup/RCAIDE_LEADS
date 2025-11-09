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
        if i == 0 or segment.analyses.geometry.settings.unique_geometry:  # If it is the first segment or if the segment has a unique geometry
            geometry_preprocess_routine(segment.analyses)
            
        else: # NEED TO REMOVE 
            vehicle_0 = deepcopy(segment.analyses.vehicle)
            segment.analyses.vehicle = deepcopy(mission.segments[i-1].analyses.vehicle)
            for wing in segment.analyses.vehicle.wings:
                for control_surface in wing.control_surfaces:
                    control_surface.deflection = vehicle_0.wings[wing.tag].control_surfaces[control_surface.tag].deflection
            for landing_gear in segment.analyses.vehicle.landing_gears:
                landing_gear.gear_extended = vehicle_0.landing_gears[landing_gear.tag].gear_extended
                
            for network in segment.analyses.vehicle.network: 
                for propulsor in network.propulsor:
                    if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan):
                        propulsor_0 =  vehicle_0.networks[network.tag].propulsors[propulsor.tag]
                        propulsor.fan.angular_velocity      = propulsor_0.fan.angular_velocity          
                        propulsor.fan.rotation              = propulsor_0.fan.rotation           
                        propulsor.fan_nozzle.noise_speed    = propulsor_0.fan_nozzle.noise_speed 
                        propulsor.core_nozzle.noise_speed   = propulsor_0.core_nozzle.noise_speed
                        
                    if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor):
                        propulsor.rotor.orientation_euler_angles =  propulsor_0.rotor.orientation_euler_angles 
                        propulsor.rotor.blade_pitch_command      =  propulsor_0.rotor.blade_pitch_command                          
                                  
    return 
        
def geometry_preprocess_routine(analyses):
    vehicle           = analyses.vehicle
    geometry_analysis = analyses.geometry
    settings          = geometry_analysis.settings
    
    # initialize variables 
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
        fuselage.number_of_seats = total_seats    
            
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
                    if type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy:
                        NPE +=  cabin_class.number_of_seats
                    elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business:
                        NPB +=  cabin_class.number_of_seats
                    elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.First:
                        NPF +=  cabin_class.number_of_seats 
                total_seats += cabin.number_of_seats
            
            wing.number_of_seats = total_seats                
                        
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
    # Update passenger information 
    # --------------------------------------------------------------------------------------------------------------------
    vehicle.number_of_seats = total_seats
    if settings.overwrite_passenger_capacity:
        vehicle.number_of_passengers = total_seats
        
    if vehicle.number_of_passengers == 0:
        pass
    else:
        # properties for weight estimation methods 
        if defined_cabins:
            vehicle.number_of_first_class_seats    = NPF
            vehicle.number_of_business_class_seats = NPB
            vehicle.number_of_economy_class_seats  = NPE
        else:  
            vehicle.number_of_first_class_seats    = vehicle.number_of_passengers // 20
            vehicle.number_of_business_class_seats = vehicle.number_of_passengers // 10
            vehicle.number_of_economy_class_seats  = vehicle.number_of_passengers - NPF - NPB 
     
    # --------------------------------------------------------------------------------------------------------------------
    # Compute fuel volume  
    # -------------------------------------------------------------------------------------------------------------------- 
    compute_fuel_volume(vehicle,overwrite_fuel_volume = settings.overwrite_fuel_volume)
               
    return 