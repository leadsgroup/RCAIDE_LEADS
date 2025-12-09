# RCAIDE/Methods/Performance/compute_load_and_trim_diagram.py
# 
# 
# Created:  Dec 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import  Data,  Units 
from RCAIDE.Library.Methods.Mass_Properties.estimate_maximum_landing_weight import estimate_maximum_landing_weight
from RCAIDE.Library.Mission.Common.Pre_Process import  geometry, mass_properties

# Pacakge imports 
import numpy as np
from copy import  deepcopy

#------------------------------------------------------------------------------
# compute_load_and_trim_diagram
#------------------------------------------------------------------------------  
def compute_load_and_trim_diagram(mission = None, cruise_segment_tag = "cruise",discretization = 5):
    """
    Computes the loading dragram of an aircraft 
 
 
    Parameters
    --------
    vehicle : Vehicle
        The vehicle instance to be analyzed
    angle_of_attacks : ndarray
        Array of angle of attack values to evaluate [radians]
    mach_numbers : ndarray
        Array of Mach numbers to evaluate 
    altitude : float, optional
        Altitude for atmospheric properties [m], default 0 
 
    Returns
    --------
    results : Data
        Container of analysis results including:
            * Mach : ndarray
                Evaluated Mach numbers
            * alpha : ndarray
                Evaluated angles of attack [rad]
            * loading_lift_coefficient : ndarray
                Computed lift coefficients
            * loading_drag_coefficient : ndarray
                Computed drag coefficients
            * loading_moment_coefficient : ndarray
                Computed Y-moment coefficients
 
    Notes
    -----
    The function uses the US Standard Atmosphere 1976 model for atmospheric properties
    and evaluates aerodynamic coefficients using vortex lattice methods. Can use a surrogate model
    for faster evaluation or just direct evaluation of the aerodynamics. 
 
    **Major Assumptions**
        * Flow is steady and inviscid
        * Small angle approximations apply
        * Linear aerodynamics
        * Atmospheric properties follow US Standard Atmosphere 1976
 
    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method
    RCAIDE.Library.Attributes.Atmospheres.Earth.US_Standard_1976
    """  

    #------------------------------------------------------------------------  
    # Remove Takeoff mass
    #------------------------------------------------------------------------   
    for segment in mission.segments: 
        segment.analyses.vehicle.mass_properties.takeoff = None 
        segment.analyses.geometry.settings.overwrite_fuel_volume = True 
    
    #------------------------------------------------------------------------  
    # Check Input Args
    #------------------------------------------------------------------------  
    vehicle = mission.segments[cruise_segment_tag].analyses.vehicle 
    cabin_class_check = False 
    for fuselage in  vehicle.fuselages: 
        for cabin in fuselage.cabins:
            for _ in cabin.classes:
                cabin_class_check = True
    for wing in vehicle.wings: 
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            for cabin in wing.cabins:    
                for _ in cabin.classes:
                    cabin_class_check = True
                    
    if cabin_class_check == False:
        raise AttributeError('At least one cabin class must be defined to create Aircraft mass-C.G. envelope ') 

    #------------------------------------------------------------------------  
    # Run Baseline mission
    #------------------------------------------------------------------------
    base_mission = deepcopy(mission)
    results      = base_mission.evaluate() 
    
    # compute mass properties of aircraft to get weight distribution
    vehicle_0         = base_mission.segments[cruise_segment_tag].analyses.vehicle
    x_cg_0            = base_mission.segments[cruise_segment_tag].analyses.vehicle.mass_properties.center_of_gravity
    weight_breakdown  = base_mission.segments[cruise_segment_tag].analyses.vehicle.mass_properties.weight_breakdown 
    neutral_point_0   = base_mission.segments[cruise_segment_tag].analyses.vehicle.neutral_point 
      
    W_PAX         =  weight_breakdown.payload.passengers  
    W_PAX_per_pax =  W_PAX / vehicle_0.number_of_passengers 
    MTOW          =  vehicle_0.mass_properties.max_takeoff
    OEW           =  weight_breakdown.empty.total
    MLW           =  estimate_maximum_landing_weight(MTOW)   


    #------------------------------------------------------------------------  
    # Compute Loading Points 
    #------------------------------------------------------------------------ 
    # determine number is weight simulations
    cabin_tags_     = []
    cabin_x_origin_ = []
    for fuslage in vehicle_0.fuselages:
        for cabin in  fuslage.cabins: 
            cabin_tags_.append(cabin.tag)
            cabin_x_origin_.append(cabin.origin[0][0])
        
    cargo_bay_tags_     = []
    cargo_bay_x_origin_ = []
    W_CARGO = 0 
    for cargo_bay in vehicle_0.cargo_bays: 
        cargo_bay_tags_.append(cargo_bay.tag)
        cargo_bay_x_origin_.append(cargo_bay.origin[0][0])
        W_CARGO += cargo_bay.mass_properties.mass

    fuel_tank_tags_ = []
    fuel_tank_x_origin_ = []
    for network in vehicle_0.networks:
        for fuel_line in  network.fuel_lines: 
            for fuel_tank in  fuel_line.fuel_tanks: 
                fuel_tank_tags_.append(fuel_tank.tag)  
                fuel_tank_x_origin_.append(fuel_tank.origin[0][0])
                
    # total number of sumulations = 2 (loading and unloading) * discretization * total number of objects 
    total_sims =   2 * (  discretization * len(cabin_tags_) * discretization * len(fuel_tank_tags_) * discretization * len(cargo_bay_tags_) )   

    
    # define discretization 
    percent_fuel         =  np.linspace(0.0, 1, discretization)
    percent_pax          =  np.linspace(0.0, 1, discretization)
    percent_cargo        =  np.linspace(0.0, 1, discretization) 
    percent_cg_shift     = np.linspace(0.8,1.2, discretization)   
    filling_order        =  ['ascending','descending']
    
    # create empty data structures
    LT_results =  Data()
    LT_results.discretization                   = discretization
    LT_results.loading_mass                     = np.zeros(total_sims)
    LT_results.loading_CG_location              = np.zeros(total_sims)
    LT_results.loading_LEMAC_location           = np.zeros(total_sims)
    LT_results.percent_cargo                    = np.zeros(total_sims)
    LT_results.percent_pax                      = np.zeros(total_sims)
    LT_results.percent_cargo                    = np.zeros(total_sims)
    LT_results.loading_unloading_flag           = np.zeros(total_sims)
    LT_results.aerodynamic_lift_coefficient     = np.zeros((len(percent_pax),len(percent_cg_shift)))
    LT_results.aerodynamic_drag_coefficient     = np.zeros((len(percent_pax),len(percent_cg_shift)))
    LT_results.aerodynamic_moment_coefficient   = np.zeros((len(percent_pax),len(percent_cg_shift)))
    LT_results.aerodynamic_neutral_point        = np.zeros((len(percent_pax),len(percent_cg_shift)))
    LT_results.aerodynamic_static_margin        = np.zeros((len(percent_pax),len(percent_cg_shift)))
    LT_results.aerodynamic_moment               = np.zeros((len(percent_pax),len(percent_cg_shift))) 
    LT_results.aerodynamic_mass                 = np.zeros((len(percent_pax),len(percent_cg_shift))) 
    LT_results.aerodynamic_LEMAC_location       = np.zeros((len(percent_pax),len(percent_cg_shift))) 
    LT_results.MTOW                             = MTOW     
    LT_results.OEW                              = OEW
    LT_results.MLW                              = MLW    

    #------------------------------------------------------------------------  
    # Load Diagram Data
    #------------------------------------------------------------------------      
    counter = 0

    
    for f_o in range(len(filling_order)): 
        #------------------------------------------------------------------------                
        # Reset all weights to 0
        #------------------------------------------------------------------------    
        weights_analysis_mission = deepcopy(mission) 
        vehicle                         = weights_analysis_mission.segments[cruise_segment_tag].analyses.vehicle 
        vehicle.mass_properties.takeoff = None  
        vehicle.mass_properties.payload = 0
        vehicle.mass_properties.cargo   = 0   
        vehicle.number_of_passengers    = 1
        vehicle.mass_properties.fuel    = 0
         
        for network in vehicle.networks:
            for fuel_line in  network.fuel_lines:
                for fuel_tank in fuel_line.fuel_tanks: 
                    fuel_tank.fuel.mass_properties.mass = 0
                        
        for cargo_bay in vehicle.cargo_bays:  
            cargo_bay.mass_properties.mass   =  0
                 
        
        reverse_flag =  False
        if filling_order[f_o] == 'descending':
            reverse_flag = True

        # sort objects in order of x location 
        cabin_data        = zip(cabin_x_origin_, cabin_tags_) 
        sorted_cabins     = sorted(cabin_data, reverse=reverse_flag) 
        _, cabin_tags     = zip(*sorted_cabins)
    
        cargo_bay_data    = zip(cargo_bay_x_origin_, cargo_bay_tags_) 
        sorted_cargo_bays = sorted(cargo_bay_data, reverse=reverse_flag) 
        _, cargo_bay_tags = zip(*sorted_cargo_bays) 
    
        fuel_tank_data    = zip(fuel_tank_x_origin_, fuel_tank_tags_) 
        sorted_fuel_tanks = sorted(fuel_tank_data, reverse=reverse_flag) 
        _, fuel_tank_tags = zip(*sorted_fuel_tanks)
         
          
        #------------------------------------------------------------------------  
        # Compute Loading Points for passenger cabins  
        #------------------------------------------------------------------------ 
        # Update Passengers 
        for p_i in range(len(percent_pax)):     
            num_pax = 0   
            for fuselage in  vehicle.fuselages: 
                for cabin in fuselage.cabins:
                    num_pax_cabin = 0
                    cabin.filled_seats_arrangement  = filling_order[f_o] 
                    for cabin_class in cabin.classes:
                        pax =  1 if p_i == 0 else int(percent_pax[p_i] *  vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].classes[cabin_class.tag].number_of_passengers) 
                        cabin_class.number_of_passengers  = pax
                        num_pax += cabin_class.number_of_passengers
                        num_pax_cabin += cabin_class.number_of_passengers
                    cabin.number_of_passengers = num_pax_cabin
                            
            for wing in vehicle.wings: 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    for cabin in wing.cabins:   
                        num_pax_cabin = 0 
                        cabin.filled_seats_arrangement  =  filling_order[f_o]                 
                        for cabin_class in cabin.classes: 
                            pax =  1 if p_i == 0 else int(percent_pax[p_i] *  vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].classes[cabin_class.tag].number_of_passengers) 
                            cabin_class.number_of_passengers  = pax                             
                            num_pax += cabin_class.number_of_passengers
                            num_pax_cabin += cabin_class.number_of_passengers
                        cabin.number_of_passengers = num_pax_cabin 
                                           
            # run weights analysis and store results
            vehicle.number_of_passengers     =  num_pax  
            
            # loop through fuel tanks
            remaining_tank_fuel = 0
            for network in vehicle.networks:
                for fuel_line in  network.fuel_lines:
                    for tank_tag in fuel_tank_tags:
                        fuel_tank = fuel_line.fuel_tanks[tank_tag] 
                        for f_i in range(len(percent_fuel)): 
                            fuel_tank.fuel.mass_properties.mass = percent_fuel[f_i] * vehicle_0.networks[network.tag].fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag].fuel.mass_properties.mass
                            
                            # run weights analysis and store results
                            vehicle.mass_properties.fuel = fuel_tank.fuel.mass_properties.mass + remaining_tank_fuel 
                            
                            #------------------------------------------------------------------------  
                            # Compute Loading Points for cargo   
                            #------------------------------------------------------------------------     
                            previous_cargo_bays = 0
                            for cargo_bay in vehicle.cargo_bays: 
                                for c_i in range(len(percent_cargo)): 
                                    cargo_bay.mass_properties.mass   = percent_cargo[c_i] * vehicle_0.cargo_bays[cargo_bay.tag].mass_properties.mass  
                                    vehicle.mass_properties.cargo    = percent_cargo[c_i] * vehicle_0.cargo_bays[cargo_bay.tag].mass_properties.mass  + previous_cargo_bays 
                                    vehicle.mass_properties.payload  = (W_PAX_per_pax) *vehicle.number_of_passengers +  vehicle.mass_properties.cargo +   previous_cargo_bays
                                    
                                    # run weights analysis and store results
                                    counter =  compute_aircraft_load_data_point(weights_analysis_mission,LT_results,counter,total_sims, neutral_point_0,percent_pax[p_i], percent_fuel[f_i],  percent_cargo[c_i])
                                    
                                    
                                previous_cargo_bays += cargo_bay.mass_properties.mass
                                
                        remaining_tank_fuel = vehicle.mass_properties.fuel 
     
          
    # -------------------------------------------------------------------------
    # Trim Diagram Data
    # ------------------------------------------------------------------------- 

    total_sims = len(percent_pax) * len(percent_cg_shift)
    counter    = 0        
    for k in range(len(percent_pax)):
        for l in range(len(percent_cg_shift)):

            aero_analysis_mission = deepcopy(mission)
            vehicle = aero_analysis_mission.segments[cruise_segment_tag].analyses.vehicle
        
            # Aircraft-Level Properties  
            vehicle.mass_properties.takeoff                 = None # this ensures that the takeoff weight is computed 
            vehicle.mass_properties.payload                 = (W_PAX) * percent_pax[k] +  percent_cargo[k] *W_CARGO 
            vehicle.mass_properties.cargo                   = percent_cargo[k] * W_CARGO
            vehicle.mass_properties.center_of_gravity[0][0] = x_cg_0[0][0] * percent_cg_shift[l]   
            vehicle.number_of_passengers                    = pax =  1 if k == 0 else int(vehicle_0.number_of_passengers *  percent_pax[k])
        
            # Update Passengers           
            for fuselage in  vehicle.fuselages: 
                for cabin in fuselage.cabins:
                    cabin.filled_seats_arrangement  = 'ascending'
                    for cabin_class in cabin.classes: 
                        pax =  1 if k == 0 else int(percent_pax[k] *  vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].classes[cabin_class.tag].number_of_passengers) 
                        cabin_class.number_of_passengers  = pax 
            for wing in vehicle.wings: 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    for cabin in wing.cabins:    
                        cabin.filled_seats_arrangement  = 'ascending'
                        for cabin_class in cabin.classes:  
                            pax =  1 if k == 0 else int(percent_pax[k] *  vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].classes[cabin_class.tag].number_of_passengers) 
                            cabin_class.number_of_passengers  = pax 
        
            # Update Fuel            
            for network in  vehicle.networks:
                for fuel_line in  network.fuel_lines: 
                    for fuel_tank in fuel_line.fuel_tanks:
                        fuel_tank.fuel.mass_properties.mass = percent_fuel[k] * vehicle_0.networks[network.tag].fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag].fuel.mass_properties.mass
         
           
            counter =  compute_aircraft_trim_data_point(aero_analysis_mission,cruise_segment_tag,LT_results,counter,total_sims,neutral_point_0,k,l)
   
    return LT_results


def compute_aircraft_load_data_point(weights_analysis_mission,LT_results,counter,total_sims,neutral_point,percent_pax, percent_fuel, percent_cargo):
    
    # update analysis settings 
    for segment in weights_analysis_mission.segments:
        segment.analyses.geometry.settings.overwrite_fuel_volume      = False  
        segment.analyses.vehicle.mass_properties.takeoff              = None 
        segment.analyses.weights.settings.update_moment_of_inertia    = True
        segment.analyses.weights.settings.update_center_of_gravity    = True 
        segment.analyses.vehicle.neutral_point                        = neutral_point
        segment.analyses.vehicle.neutral_point                        = neutral_point
        segment.analyses.stability.settings.update_center_of_gravity  = True        

         
    # run geometry and mass properties analyes
    geometry(weights_analysis_mission)
    mass_properties(weights_analysis_mission) 
    
    # store results 
    LT_results.loading_CG_location[counter]         = weights_analysis_mission.segments[0].analyses.vehicle.mass_properties.center_of_gravity[0][0] 
    LT_results.loading_mass[counter]                = weights_analysis_mission.segments[0].analyses.vehicle.mass_properties.takeoff 
    LT_results.loading_LEMAC_location[counter]      = 100 * (LT_results.loading_CG_location[counter]  - weights_analysis_mission.segments[0].analyses.vehicle.LEMAC) / weights_analysis_mission.segments[0].analyses.vehicle.reference_chord
     
    print('***************************************')
    print('Loading Diagram Data Point: ' + str(counter+1) + ' of ' +  str(total_sims))
    print('Mass                      : ', LT_results.loading_mass[counter])
    print('Percent Fuel              : ', percent_fuel*100 )
    print('Percent Cargo             : ', percent_cargo*100 )
    print('Percent Pax               : ', percent_pax*100 )
    print('Ref. Chord                : ', weights_analysis_mission.segments[0].analyses.vehicle.reference_chord)
    print('LEMAC                     : ',  weights_analysis_mission.segments[0].analyses.vehicle.LEMAC )
    print('C.G. Location             : ', LT_results.loading_CG_location[counter] )    
    print('LEMAC                     : ', LT_results.loading_LEMAC_location[counter] )
    print('***************************************')     
    counter += 1   
     
    return counter



def compute_aircraft_trim_data_point(aero_analysis_mission,cruise_segment_tag,LT_results,counter,total_sims,neutral_point,k,l):

    # update analysis settings 
    for segment in aero_analysis_mission.segments:
        segment.analyses.geometry.settings.overwrite_fuel_volume      = False  
        segment.analyses.vehicle.mass_properties.takeoff              = None 
        segment.analyses.weights.settings.update_moment_of_inertia    = False
        segment.analyses.weights.settings.update_center_of_gravity    = False 
        segment.analyses.vehicle.neutral_point                        = neutral_point
        segment.analyses.vehicle.neutral_point                        = neutral_point
        segment.analyses.stability.settings.update_center_of_gravity  = False             

    results  = aero_analysis_mission.evaluate()

    # store results
    segment = results.segments[cruise_segment_tag] 
    vehicle = segment.analyses.vehicle

    # store results 
    LT_results.aerodynamic_lift_coefficient[k,l]    = segment.state.conditions.aerodynamics.coefficients.lift.total[0][0]  
    LT_results.aerodynamic_drag_coefficient[k,l]    = segment.state.conditions.aerodynamics.coefficients.drag.total[0][0]  
    LT_results.aerodynamic_moment_coefficient[k,l]  = segment.state.conditions.static_stability.coefficients.M[0][0]         
    LT_results.aerodynamic_moment[k,l]              = segment.state.conditions.frames.inertial.total_moment_vector[0][1]
    LT_results.aerodynamic_neutral_point[k,l]       = segment.state.conditions.static_stability.neutral_point[0][0]  
    LT_results.aerodynamic_static_margin[k,l]       = segment.state.conditions.static_stability.static_margin[0][0]    
    LT_results.aerodynamic_mass[k,l]                = aero_analysis_mission.segments[0].analyses.vehicle.mass_properties.takeoff 
    LT_results.aerodynamic_LEMAC_location[k,l]      = 100 * (vehicle.mass_properties.center_of_gravity[0][0] - aero_analysis_mission.segments[0].analyses.vehicle.LEMAC) / aero_analysis_mission.segments[0].analyses.vehicle.reference_chord

    print('***************************************')
    print('Trim Diagram Data Point  : ' + str(counter+1) + ' of ' +  str(total_sims))
    print('Center of Gravity        : ',vehicle.mass_properties.center_of_gravity[0][0])
    print('Neutral Point            : ',LT_results.aerodynamic_neutral_point[k,l])
    print('Static Margin            : ',LT_results.aerodynamic_static_margin[k,l]) 
    print('Mass                     : ',LT_results.aerodynamic_mass[k,l] ) 
    print('***************************************')  

    counter += 1    
    return counter 
