# RCAIDE/Library/Missions/Common/Pre_Process/geometry.py
# 
# 
# Created:  Apr 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Geometry.LOPA      import  compute_layout_of_passenger_accommodations
from RCAIDE.Library.Methods.Geometry.Planform  import  fuselage_planform, wing_planform, bwb_wing_planform , compute_fuel_volume 

# python imports 
import numpy as  np 
import os, sys
import pandas as pd
from copy import deepcopy 
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
            geometry_preprocess_routine(segment.analyses) 
        else:
            use_previous_segment_pre_processed_data(mission,segment,i)   
    return 
        
def geometry_preprocess_routine(analyses):
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
        symm               = landing_gear.xz_plane_symmetric
        landing_gear.wheels = landing_gear.number_of_gear_types_in_tandem * landing_gear.number_of_wheels_in_gear_type * (symm + 1)
        
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

    if settings.write_geometry_properties:
        write_geometry_to_excel(vehicle)
        
               
    return

def use_previous_segment_pre_processed_data(mission,segment,i):
    '''
    Reuses previous segment pre processed data to save computational time.
    Ensures that changes in configuration are not overwritten.    
    '''
    vehicle_0 = deepcopy(segment.analyses.vehicle)
    segment.analyses.vehicle = deepcopy(mission.segments[i-1].analyses.vehicle)
    for wing in segment.analyses.vehicle.wings:
        for control_surface in wing.control_surfaces:
            control_surface.deflection = vehicle_0.wings[wing.tag].control_surfaces[control_surface.tag].deflection
    for landing_gear in segment.analyses.vehicle.landing_gears:
        landing_gear.gear_extended = vehicle_0.landing_gears[landing_gear.tag].gear_extended
    
    for network in segment.analyses.vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan):
                propulsor_0 =  vehicle_0.networks[network.tag].propulsors[propulsor.tag]
                propulsor.fan.angular_velocity        = propulsor_0.fan.angular_velocity        
                propulsor.fan_nozzle.exit_velocity    = propulsor_0.fan_nozzle.exit_velocity 
                propulsor.core_nozzle.exit_velocity   = propulsor_0.core_nozzle.exit_velocity
                
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor):
                propulsor_0 =  vehicle_0.networks[network.tag].propulsors[propulsor.tag]
                propulsor.rotor.orientation_euler_angles =  propulsor_0.rotor.orientation_euler_angles 
                propulsor.rotor.blade_pitch_command      =  propulsor_0.rotor.blade_pitch_command
    return


def write_geometry_to_excel(vehicle):

    """
    THIS IS CURRENTLY MEANT ONLY FOR BWB AND THE AACES PROJECT EXCLUSIVELY 
    DO NOT LET THIS GO THROUGH A PR WITHOUT INCLUDING OTHER COMPONENTS LIKE THE FUSELAGE...... 
    """
    excel_filename = os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), os.path.splitext(os.path.basename(sys.argv[0]))[0] + "_geometry_description.xlsx")
    wing_rows     = []
    segment_rows  = []
    fuel_rows     = []
    prop_rows     = []

    # Collect wing-level properties
    for wing in vehicle.wings:
        wing_rows.append({
            "Wing Tag"                      : wing.tag,
            "Wing Origin"                   : wing.origin[0],
            "Projected Span (m)"            : wing.spans.projected,
            "Root Chord (m)"                : wing.chords.root,
            "Mean Aerodynamic Chord (m)"    : wing.chords.mean_aerodynamic,
            "Gross Aspect Ratio"            : wing.aspect_ratio,
            "Trapezoid Aspect Ratio"        : wing.spans.projected**2/wing.areas.reference,
            "LEMAC (m)"                     : wing.LEMAC,
            "Reference Area (m^2)"          : wing.areas.reference,
            "Wetted Area (m^2)"             : wing.areas.wetted,
            "XZ Symmetric"                  : wing.xz_plane_symmetric,
            "XY Symmetric"                  : wing.xy_plane_symmetric,
        })

        # Collect segment-level properties for each wing
        for segment in wing.segments:
            segment_rows.append({
                "Wing Tag"                      : wing.tag,
                "Segment Tag"                   : segment.tag,
                "Segment Origin"                : f'[{str(segment.origin[0][0])}, {str(segment.origin[0][1])}, {str(segment.origin[0][2])}]',
                "Spanwise Location (%)"         : segment.percent_span_location * 100.0,
                "Root Chord Fraction"           : segment.root_chord_percent,
                "Twist (deg)"                   : segment.twist / Units.degree,
                "Outboard Dihedral (deg)"       : segment.dihedral_outboard / Units.degree,
                "Quarter-Chord Sweep (deg)"     : segment.sweeps.quarter_chord / Units.degree,
                "Leading-Edge Sweep (deg)"      : segment.sweeps.leading_edge / Units.degree
            })

    # Collect fuel tank properties from fuel lines and busses
    for network in vehicle.networks:
        network_tag = getattr(network, "tag", None)

        # Propulsors
        for propulsor in network.propulsors:
            prop_rows.append({
                "Network Tag"           : network_tag,
                "Propulsor Origin"      : propulsor.origin[0],
                "Propulsor Tag"         : getattr(propulsor, "tag", None),
                "Type"                  : propulsor.__class__.__name__,
                "Length (m)"            : getattr(propulsor, "length", None),
                "Diameter (m)"          : getattr(propulsor, "diameter", None),
                "Bypass Ratio"          : getattr(propulsor, "bypass_ratio", None),
                "Sealevel Static Thrust [lbf]" : getattr(propulsor, "sealevel_static_thrust", None)/Units.lbf,
                "TSFC [lb/lbf-hr]"      : getattr(propulsor, "TSFC", None)[0][0]
            })
            
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                fuel_rows.append({
                    "Network Tag"                  : network_tag,
                    "Tank Type"                    : str(type(source)[0]).split('.')[-1],
                    "Container Type"               : "fuel_line", 
                    "Fuel Tank Tag"                : source.tag,
                    "Wing Tag"                     : getattr(source, "wing_tag", None),
                    "Fuselage Tag"                 : getattr(source, "fuselage_tag", None),
                    "Percent Span Location"        : getattr(source, "percent_span_location", None),
                    "Segments Bounding Tank"       : getattr(source, "segments_bounding_tank", None),
                    "Segments % Chord Start"       : getattr(source, "segments_percent_chord_start", None),
                    "Segments % Chord End"         : getattr(source, "segments_percent_chord_end", None),
                    "BWB Aft Tank"                 : getattr(source, "bwb_aft_tank", None),
                    "XZ Plane Symmetric"           : getattr(source, "xz_plane_symmetric", None),
                    "Fuel Net Volume (m^3)"        : getattr(getattr(source.fuel, "volume_properties", None), "net_volume", None) if source.fuel else None,
                    "Fuel Gross Volume (m^3)"      : getattr(getattr(source, "volume_properties", None), "gross_volume", None) if source.fuel else None,
                    "Fuel Mass (kg)"               : getattr(getattr(source.fuel, "mass_properties", None), "mass", None) if source.fuel else None,
                })

    # Write to Excel with separate sheets for wings and segments
    with pd.ExcelWriter(excel_filename) as writer:
        pd.DataFrame(wing_rows).to_excel(writer, sheet_name='Wing_Properties', index=False)
        pd.DataFrame(segment_rows).to_excel(writer, sheet_name='Segment_Properties', index=False)
        pd.DataFrame(fuel_rows).to_excel(writer, sheet_name='Fuel_Tanks', index=False)
        pd.DataFrame(prop_rows).to_excel(writer, sheet_name='Propulsors', index=False)
    
    print(f"Geometry Description written to Excel:\n  {excel_filename}")
