# RCAIDE/Library/Methods/Geometry/Planform/compute_fuel_volume.py
# 
# 
# Created:  Jul 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Framework.Core import Units , Data 
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series  
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments 

# python imports 
import numpy as np  
from copy import deepcopy
from scipy.interpolate import interp1d

# ----------------------------------------------------------------------------------------------------------------------
# compute_fuel_volume 
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_volume(vehicle, update_max_fuel =True):
    
    total_fuel_volume = 0
    total_fuel_mass   = 0
    tank_span_location = 0
    for network in vehicle.networks: 
        for fuel_line in network.fuel_lines:
            for fuel_tank in fuel_line.fuel_tanks: 
                fuel_tank.internal_volume = 0 
                
                # fuel tanks integrated into wings 
                if fuel_tank.wing != None:
                    wing = fuel_tank.wing  
                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank: 
                        
                        if len(wing.segments) > 1:
                            seg_tags = list(wing.segments.keys())
                            for i in range(len(seg_tags)-1):
                                inner_segment = wing.segments[seg_tags[i]]
                                outer_segment = wing.segments[seg_tags[i+1]]
                                if inner_segment.has_fuel_tank == True:
        
                                    # get orgin of fuel tank     
                                    fuel_tank.origin = wing.origin # NEED TO UPDATE
                                    
                                    # compute volume of fuel in wing
                                    volume = compute_segmented_wing_integral_tank_fuel_volume(fuel_tank,wing,inner_segment,outer_segment)
                                    fuel_tank.internal_volume += volume 
                                    total_fuel_volume += volume
                                    total_fuel_mass   += volume * fuel_tank.fuel.density
                        else: 
                            # get orgin of fuel tank     
                            fuel_tank.origin = wing.origin # NEED TO UPDATE 
                            
                            # assume whole wing has fuel 
                            volume = compute_wing_integral_tank_fuel_volume(fuel_tank,wing)                         
                            fuel_tank.internal_volume += volume 
                            total_fuel_volume += volume
                            total_fuel_mass   += volume * fuel_tank.fuel.density
                                        
                    elif type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank: 
                        if len(wing.segments) > 1: 
                            seg_tags = list(wing.segments.keys())
                            for i in range(len(seg_tags)-1):
                                inner_segment = wing.segments[seg_tags[i]]
                                outer_segment = wing.segments[seg_tags[i+1]]
                                if inner_segment.has_fuel_tank == True:  
                                    # compute volume and update percent span location of next non-integral tank 
                                    volume ,  tank_span_location = compute_wing_non_integral_tank_fuel_volume(fuel_tank,wing,inner_segment,outer_segment,tank_span_location)
                                    fuel_tank.internal_volume += volume 
                                    total_fuel_volume += volume  
                                    total_fuel_mass   += volume * fuel_tank.fuel.density
            
                # fuel tanks integrated into wings 
                elif fuel_tank.fuselage != None: 
                    fuselage = fuel_tank.fuselage 
                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank:

                        if len(fuselage.segments) > 1:
                            tank_section_percent_x = 0
                            seg_tags = list(fuselage.segments.keys())
                            for i in range(len(seg_tags)-1):
                                inner_segment = fuselage.segments[seg_tags[i]]
                                outer_segment = fuselage.segments[seg_tags[i+1]]
                                if inner_segment.has_fuel_tank == True: 
                                    volume = compute_fuselage_integral_tank_fuel_volume(fuel_tank,fuselage,inner_segment,outer_segment,tank_section_percent_x)
                                    fuel_tank.internal_volume += volume 
                                    total_fuel_volume += volume  
                                    total_fuel_mass   += volume * fuel_tank.fuel.density
                        
                    elif type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank:

                        # compute volume and update percent span location of next non-integral tank 
                        volume  = compute_fuselage_non_integral_tank_fuel_volume(fuel_tank,fuselage)
                        fuel_tank.internal_volume += volume 
                        total_fuel_volume += volume  
                        total_fuel_mass   += volume * fuel_tank.fuel.density
                        
                    
    if update_max_fuel:
        vehicle.mass_properties.max_fuel = total_fuel_mass
    return

def compute_fuselage_integral_tank_fuel_volume(fuel_tank,fuselage,fus_first_segment,fus_second_segment,tank_section_percent_x): 

    # volume of truncated  
    A_1    = np.pi * fus_first_segment.height /2  *  fus_first_segment.width/2
    A_2    = np.pi * fus_second_segment.height/2  *  fus_second_segment.width/2
    h      = fuselage.lengths.total * (fus_second_segment.percent_x_location  - fus_first_segment.percent_x_location)
    volume = (1 /3) * ( A_1 + A_2 + np.sqrt(A_1*A_2)) *h 
    
    return volume


def compute_fuselage_non_integral_tank_fuel_volume(fuel_tank,fuselage): 
    # get length of tank
    l = fuel_tank.length 
    fuel_tank.inner_diameter =  fuel_tank.outer_diameter - 2 * fuel_tank.wall_thickness
    
    r =  fuel_tank.inner_diameter / 2
    volume = np.pi * ( r** 2) * l +  4 / 3 * np.pi * ( r** 3)      
    
    create_non_integral_tank_sections(fuel_tank,fuel_tank.outer_diameter/2,l)
    return volume


def compute_wing_non_integral_tank_fuel_volume(fuel_tank,wing,inner_segment,outer_segment,tank_span_location):
     
    #if tank_span_location: 
    
    clearance  = fuel_tank.wall_clearance  
    delta_span = (outer_segment.percent_span_location - inner_segment.percent_span_location) /2 * wing.spans.projected 
    
    
    # update the location of the inner segment so that it is not always the wing defined segment but where the tank ends
    # to do this, you have to create a sybolic segment by interpolation that is at the tank_span_location
    # send that as the "inner segment"
    # you do this until the the tank_span_location> outer_segment
    
    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(inner_segment)
    inner_segment_chord     = wing.chords.root * inner_segment.root_chord_percent
    inner_front_rib_length  = inner_segment_chord * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl)) 
    inner_rear_rib_length   = inner_segment_chord * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl) )
    inner_wingbox_length    = inner_segment_chord * (inner_segment.structural.rear_spar_percent_chord -inner_segment.structural.front_spar_percent_chord)  
    
    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(outer_segment)
    outer_segment_chord     = wing.chords.root * outer_segment.root_chord_percent
    outer_front_rib_length  = outer_segment_chord * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl)) 
    outer_rear_rib_length   = outer_segment_chord * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl))
    outer_wingbox_length    = outer_segment_chord * (outer_segment.structural.rear_spar_percent_chord -outer_segment.structural.front_spar_percent_chord)   
     
    z_outer_center = inner_segment.origin[0][2] + delta_span *np.tan(inner_segment.dihedral_outboard)
    inner_segment_thickness =  np.minimum(inner_front_rib_length,inner_rear_rib_length)
    outer_segment_thickness =  np.minimum(outer_front_rib_length,outer_rear_rib_length)
    
    # inner segment coordinate  
    z_inner_upper  = inner_segment.origin[0][2] + (inner_segment_thickness / 2) - clearance
    z_inner_lower  = inner_segment.origin[0][2] - (inner_segment_thickness / 2) + clearance
    y_inner_upper  = inner_segment.percent_span_location * wing.spans.projected
    y_inner_lower  = y_inner_upper
     
    # outer segment coordinates  
    z_outer_upper  = z_outer_center + outer_segment_thickness / 2 -  clearance
    z_outer_lower  = z_outer_center - outer_segment_thickness / 2 + clearance
    y_outer_upper  = outer_segment.percent_span_location * wing.spans.projected
    y_outer_lower  = y_outer_upper  
    
    dz_upper   = z_outer_upper - z_inner_upper
    dy_upper   = y_outer_upper - y_inner_upper
    dz_lower   = z_outer_lower - z_inner_lower
    dy_lower   = y_outer_lower - y_inner_lower
    
    # determine slopes 
    upper_slope = np.arctan(dz_upper/dy_upper) # might need to make negative
    lower_slope = np.arctan(dz_lower/dy_lower) 
    
    # determine tank diameter and location of next spar 
    D         = 1
    epsilon_D = 10 
    
    
    while epsilon_D > 0.001:
        AD =  D / np.cos(upper_slope)
        BC =  D / np.cos(lower_slope)
        
        # get equation of upper line
        f_upper = interp1d(np.array([y_inner_upper ,y_outer_upper  ]), np.array([z_inner_upper ,z_outer_upper  ]))
        
        # get equation of lower line
        f_lower = interp1d(np.array([y_inner_lower ,y_outer_lower ]), np.array([z_inner_lower ,z_outer_lower ]))
        
        AB = f_upper(y_inner_upper)  - f_lower(y_inner_upper)
        DC = f_upper(y_inner_upper+D)  - f_lower(y_inner_upper+D)
        
        epsilon_D  =  AB + DC - AD - BC
        
        delta_AD = epsilon_D / (1 +  np.cos(upper_slope) /np.cos(lower_slope) )
        
        detla_D  = delta_AD *  np.cos(upper_slope)
        
        D += detla_D 
    
    # get orgin of fuel tank
    spar_sweep       = convert_sweep_segments(inner_segment.sweeps.quarter_chord, inner_segment, outer_segment, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=inner_segment.structural.front_spar_percent_chord) 
    origin_x         = inner_segment.origin[0][0] + (inner_segment.structural.front_spar_percent_chord * inner_segment_chord) +( np.tan(spar_sweep) * D / 2)
    origin_y         = y_inner_upper + D / 2
    origin_z         = inner_segment.origin[0][2] + (D / 2) *np.tan(inner_segment.dihedral_outboard)
    fuel_tank.origin = [[origin_x,origin_y,origin_z]]
    
    # store tank diamter (this will set the location of the next segment)
    fuel_tank.outer_diamter = D
    fuel_tank.inner_diameter = D -  2 * fuel_tank.wall_thickness 
    
    # delete original outer segment and replace with segment 
    tank_span_location = inner_segment.percent_span_location + (2 * D) / wing.spans.projected
     
    # get length of tank
    l = np.minimum(outer_wingbox_length,inner_wingbox_length) 
    
    r =  fuel_tank.inner_diameter / 2
    volume = np.pi * ( r** 2) * l +  4 / 3 * np.pi * ( r** 3) 
    if wing.symmetric:
        volume *= 2
    
    fuel_tank.length = l
    
    create_non_integral_tank_sections(fuel_tank,fuel_tank.outer_diamter/2,l)
    
    return volume ,  tank_span_location

def compute_wing_integral_tank_fuel_volume(fuel_tank,wing):     
    
    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing) 
    inner_front_rib_length  = wing.chords.root * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl))
    inner_rear_rib_length   = wing.chords.root * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl))
    inner_wingbox_length    = wing.chords.root * (wing.structural.rear_spar_percent_chord -wing.structural.front_spar_percent_chord)  
    
    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing) 
    outer_front_rib_length  = wing.chords.tip * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl))
    outer_rear_rib_length   = wing.chords.tip * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl))
    outer_wingbox_length    = wing.chords.tip * (wing.structural.rear_spar_percent_chord -wing.structural.front_spar_percent_chord)  
    
    
    # volume of truncated prism
    A_1 = inner_wingbox_length * (inner_front_rib_length + inner_rear_rib_length) / 2 
    A_2 = outer_wingbox_length * (outer_front_rib_length + outer_rear_rib_length) / 2
    h =  wing.spans.projected
    volume = (1 /3) * ( A_1 + A_2 + np.sqrt(A_1*A_2)) *h   
    
    return volume

def compute_segmented_wing_integral_tank_fuel_volume(fuel_tank,wing,inner_segment,outer_segment):   
    
    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(inner_segment)
    inner_segment_chord     = wing.chords.root * inner_segment.root_chord_percent
    inner_front_rib_length  = inner_segment_chord * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl))
    inner_rear_rib_length   = inner_segment_chord * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl) )
    inner_wingbox_length    = inner_segment_chord * (inner_segment.structural.rear_spar_percent_chord -inner_segment.structural.front_spar_percent_chord)  
    
    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(outer_segment)
    outer_segment_chord     = wing.chords.root * outer_segment.root_chord_percent
    outer_front_rib_length  = outer_segment_chord * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl) )
    outer_rear_rib_length   = outer_segment_chord * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl) )
    outer_wingbox_length    = outer_segment_chord * (outer_segment.structural.rear_spar_percent_chord -outer_segment.structural.front_spar_percent_chord)  
    
    
    # volume of truncated prism
    A_1 = inner_wingbox_length * (inner_front_rib_length + inner_rear_rib_length) / 2 
    A_2 = outer_wingbox_length * (outer_front_rib_length + outer_rear_rib_length) / 2
    h =  (outer_segment.percent_span_location -  inner_segment.percent_span_location) *  wing.spans.projected    
    volume = (1 /3) * ( A_1 + A_2 + np.sqrt(A_1*A_2)) *h  
    
    return volume
    
def compute_non_dimensional_rib_coordinates(compoment): 
    if compoment.airfoil != None: 
        if type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
            geometry = compute_naca_4series(compoment.airfoil.NACA_4_Series_code)
        elif type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
            geometry = import_airfoil_geometry(compoment.airfoil.coordinate_file)
    else:
        geometry = compute_naca_4series('0012')
   
    clearance = 1.5E-2
    front_rib_nondim_x       = compoment.structural.front_spar_percent_chord   
    rear_rib_nondim_x        = compoment.structural.rear_spar_percent_chord 
    f_upper = interp1d(geometry.x_upper_surface  ,geometry.y_upper_surface, kind='linear')
    f_lower = interp1d(geometry.x_lower_surface  , geometry.y_lower_surface, kind='linear')
    
    # non-wing box dimension coordinates 
    front_rib_nondim_y_upper = f_upper([front_rib_nondim_x])[0] - clearance
    rear_rib_nondim_y_upper  = f_upper([rear_rib_nondim_x])[0]  - clearance   
    front_rib_nondim_y_lower = f_lower([front_rib_nondim_x])[0] + clearance   
    rear_rib_nondim_y_lower  = f_lower([rear_rib_nondim_x])[0]  + clearance   
         
    return front_rib_nondim_y_upper,rear_rib_nondim_y_upper, front_rib_nondim_y_lower, rear_rib_nondim_y_lower 

def create_non_integral_tank_sections(fuel_tank,r,l): 
    n = 8
    thetas =  np.linspace(0,np.pi/2, n) 
    
    # front of tank
    sec_i =  0
    for i in range(n): 
        segment = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Segments.Segment()
        segment.tag    = 'tank_section_' + str(sec_i+1)
        segment.height = 2 * r *np.sin(thetas[i]) 
        segment.width  = 2 * r *np.sin(thetas[i])           
        segment.percent_x_location = r * (1 - np.cos(thetas[i]))
        fuel_tank.append_segment(segment)
        sec_i += 1
    
    for i in range(n):  
        segment = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Segments.Segment()
        segment.tag    = 'tank_section_' + str(sec_i+1)
        segment.height = 2 * r * np.sin(thetas[i])      
        segment.width  = 2 * r * np.sin(thetas[i])      
        segment.percent_x_location = l +  r * np.cos(thetas[i])
        sec_i += 1      
    
    return 
    

    
    
    