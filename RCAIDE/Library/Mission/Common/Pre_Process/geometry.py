# RCAIDE/Library/Missions/Common/Pre_Process/geometry.py
# 
# 
# Created:  Apr 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from ast import Raise
import RCAIDE
from RCAIDE.Library.Methods.Geometry.LOPA      import  compute_layout_of_passenger_accommodations 
from RCAIDE.Library.Methods.Geometry.Planform  import  fuselage_planform, wing_planform, bwb_wing_planform , compute_fuel_volume 

# python imports
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
                - reference_values : Data
                    Reference geometry parameters
        
    
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
                
        vehicle  = segment.analyses.geometry.vehicle
        settings = segment.analyses.geometry.settings 
        
        # update passenger properties
        if  vehicle.passengers > 0: 
            NPF      = vehicle.passengers / 20.
            NPB      = vehicle.passengers / 10.
            NPT      = vehicle.passengers - NPF - NPB 
        else:
            NPF = 0
            NPB = 0
            NPT = 0
    
        # ================================================================================================================================================
        # update fuselage properties
        # ================================================================================================================================================  
        A_fuselage   = 0
        for fuselage in vehicle.fuselages: 
            if settings.update_fuselage_properties:                 
                compute_layout_of_passenger_accommodations(fuselage)
                fuselage_planform(fuselage) 
            vehicle.length = np.maximum(vehicle.length, fuselage.lengths.total)
            A_fuselage     = np.maximum(A_fuselage,fuselage.areas.front_projected) 
            
            if len(fuselage.cabins) > 0: 
                for cabin in fuselage.cabins:
                    for cabin_class in cabin.classes:
                        if type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy:
                            NPT =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows
                        elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business:
                            NPB =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows
                        elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.First:
                            NPF =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows   
        # update landing gear properties 
        for landing_gear in  vehicle.landing_gears:
            if (landing_gear.number_of_gear_types_in_tandem != None) and  (landing_gear.number_of_wheels_in_gear_type != None):
                landing_gear.wheels = landing_gear.number_of_gear_types_in_tandem * landing_gear.number_of_wheels_in_gear_type
                if landing_gear.symmetric:
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
                # compute BWB cabin properties 
                if settings.update_fuselage_properties:
                    compute_layout_of_passenger_accommodations(wing)
                
                # compute planform properties 
                bwb_wing_planform(wing)
                
                # update reference properties 
                if settings.overwrite_reference:
                    vehicle.reference_area = wing.areas.reference
        
                if len(wing.cabins) > 0:  
                    for cabin in wing.cabins:
                        for cabin_class in cabin.classes:               
                            for cabin_class in cabin.classes:
                                if type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy:
                                    NPT =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows
                                elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business:
                                    NPB =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows
                                elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.First:
                                    NPF =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows    

            # --------------------------------------------------------------------------------------------------------------------
            # All other wing surfaces
            # --------------------------------------------------------------------------------------------------------------------
            else: 
                wing_planform(wing)                   
                if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing) and settings.overwrite_reference:
                    vehicle.reference_area = wing.areas.reference
                 
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
        vehicle.first_class_passengers     = NPF
        vehicle.business_class_passengers  = NPB
        vehicle.economy_class_passengers   = NPT 
                       
        # --------------------------------------------------------------------------------------------------------------------
        # Compute fuel volume  
        # --------------------------------------------------------------------------------------------------------------------
        if settings.update_fuel_volume: 
            compute_fuel_volume(vehicle, update_max_fuel=settings.update_fuel_volume) 
                   
    return 