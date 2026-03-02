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
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_vehicle_center_of_gravity 
from RCAIDE.Library.Mission.Common.Pre_Process import  geometry, mass_properties
import pandas as pd
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
     
    Notes
    -----
                
    # total number of sumulations =  2   * discretization ^ N
    # where 2: is for the loading and unloading cases
    # and N is the number of object types i.e cabin,cargo and fuel 
  
    """  
            
    #------------------------------------------------------------------------  
    # Remove Takeoff mass
    #------------------------------------------------------------------------   
    for segment in mission.segments: 
        segment.analyses.vehicle.mass_properties.takeoff                   = None  
        segment.analyses.geometry.settings.compute_fuel_volume             = True 
        segment.analyses.weights.print_weight_analysis_report              = True 
        segment.analyses.weights.settings.run_center_of_gravity_analysis   = True
        segment.analyses.weights.settings.run_moments_of_inertia_analysis  = True  
        segment.analyses.stability.print_stability_analysis_report         = True
        segment.analyses.stability.settings.compute_neutral_point          = True
        segment.analyses.weights.settings.overwrite_center_of_gravity      = True
    
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
    results      = mission.evaluate() 
    
    # compute mass properties of aircraft to get weight distribution
    vehicle_0         = mission.segments[cruise_segment_tag].analyses.vehicle
    x_cg_0            = mission.segments[cruise_segment_tag].analyses.vehicle.mass_properties.center_of_gravity
    weight_breakdown  = mission.segments[cruise_segment_tag].analyses.vehicle.mass_properties.weight_breakdown 
    neutral_point_0   = mission.segments[cruise_segment_tag].analyses.vehicle.neutral_point 
      
    W_PAX         =  weight_breakdown.payload.passengers  
    PAX           =  vehicle_0.number_of_passengers
    W_PAX_per_pax =  W_PAX / PAX
    MTOW          =  vehicle_0.mass_properties.max_takeoff
    MZFW          =  vehicle_0.mass_properties.max_zero_fuel
    FUEL          =  vehicle_0.mass_properties.max_fuel
    PLD           =  vehicle_0.mass_properties.max_payload
    vehicle_0.mass_properties.payload = PLD  
    PLD_per_pax   =  (weight_breakdown.payload.passengers  + weight_breakdown.payload.baggage) / PAX
    OEW           =  vehicle_0.mass_properties.operating_empty
    if mission.segments[cruise_segment_tag].analyses.weights.propulsion_architecture == 'Hydrogen': 
        MLW = MTOW
    else:
        MLW =  estimate_maximum_landing_weight(MTOW)
    W_CARGO = 0
    m_c  = []
    for cargo_bay in vehicle_0.cargo_bays:  
        W_CARGO += cargo_bay.mass_properties.mass
        m_c.append(cargo_bay.mass_properties.mass) 
    cargo_bay_ratio = np.array(m_c) / sum(np.array(m_c)) 
    W_CARGO += PLD-W_PAX
     

    m_f = []     
    for network in vehicle_0.networks: 
        for source in  network.sources: 
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                m_f.append(source.fuel.mass_properties.mass)
    fuel_tank_ratio = np.array(m_f) / sum(np.array(m_f)) 

    #------------------------------------------------------------------------  
    # Compute Loading Points 
    #------------------------------------------------------------------------ 
    # determine number is weight simulations    

    # total number of sumulations =  2   * discretization ^ N
    # where 2: is for the loading and unloading cases
    # and N is the number of object types i.e cabin,cargo and fuel  
    total_sims =   2 *  discretization **3   
    
    # define discretization 
    percent_fuel         =  np.linspace(0.0, 1,discretization)
    percent_pax          =  np.linspace(0.0, 1,discretization)
    percent_cargo        =  np.linspace(0.0, 1,discretization) 
    percent_weight       =  np.linspace(0.0, 1,discretization)  
    percent_cg_shift     =  np.linspace(0.8,1.2,discretization)  
    filling_order        =  ['ascending','descending']
    
    # create empty data structures
    RES                                             =  Data()
    RES.loading_results                             =  Data()
    RES.trim_results                                =  Data()
    RES.discretization                              = discretization
    RES.loading_results.mass                        = np.zeros((2,discretization,discretization,discretization))
    RES.loading_results.CG_location                 = np.zeros((2,discretization,discretization,discretization))
    RES.loading_results.LEMAC_location              = np.zeros((2,discretization,discretization,discretization))
    RES.loading_results.CG_percent_of_LEMAC_location      = np.zeros((2,discretization,discretization,discretization))
    RES.percent_cargo                               = np.zeros((2,discretization,discretization,discretization))
    RES.percent_pax                                 = np.zeros((2,discretization,discretization,discretization))
    RES.percent_cargo                               = np.zeros((2,discretization,discretization,discretization))
    RES.trim_results.neutral_point                  = np.zeros((discretization,discretization))
    RES.trim_results.static_margin                  = np.zeros((discretization,discretization)) 
    RES.trim_results.mass                           = np.zeros((discretization,discretization)) 
    RES.trim_results.LEMAC_location                 = np.zeros((discretization,discretization)) 
    RES.trim_results.CG_percent_of_LEMAC_location         = np.zeros((discretization,discretization)) 
    RES.MTOW                                        = MTOW     
    RES.OEW                                         = OEW
    RES.MLW                                         = MLW    

    #============================================================================================  
    # Load Diagram Data
    #============================================================================================  
    counter = 0 
    
    for f_o in range(len(filling_order)): 
        for p_i in range(len(percent_pax)):   
            for c_i in range(len(percent_cargo)): 
                for f_i in range(len(percent_fuel)):          
                    #------------------------------------------------------------------------                
                    # Reset all weights to 0
                    #------------------------------------------------------------------------     
                    vehicle                               =  deepcopy(vehicle_0)
                    vehicle.mass_properties.max_zero_fuel = MZFW  
                    vehicle.mass_properties.takeoff       = None  
                    vehicle.mass_properties.payload       = 0
                    vehicle.mass_properties.cargo         = 0   
                    vehicle.number_of_passengers          = 1
                    vehicle.mass_properties.fuel          = 0
                     
                    for network in vehicle.networks:
                        for source in  network.sources: 
                            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):  
                                source.fuel.mass_properties.mass = 0
                                    
                    for cargo_bay in vehicle.cargo_bays:  
                        cargo_bay.mass_properties.mass   =  0
                        
                    for fuselage in  vehicle.fuselages: 
                        for cabin in fuselage.cabins:
                            cabin.number_of_passengers = 0
                            
                    for wing in  vehicle.wings: 
                        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                            for cabin in wing.cabins:    
                                cabin.number_of_passengers = 0 
                      
                    #------------------------------------------------------------------------  
                    # Update Passengers 
                    #------------------------------------------------------------------------  
                    # run weights analysis and store results
                    vehicle.number_of_passengers  =  pax =  1 if p_i == 0 else int(percent_pax[p_i] * PAX ) 
                    for fuselage in  vehicle.fuselages: 
                        for cabin in fuselage.cabins: 
                            cabin.filled_seats_arrangement  = filling_order[f_o]  
                            pax =  1 if p_i == 0 else int(percent_pax[p_i] *  vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].number_of_passengers)
                            cabin_mass = 1E-6 if p_i == 0 else  pax * W_PAX_per_pax
                            cabin.number_of_passengers = pax
                            cabin.mass_properties.mass = cabin_mass
                    
        
                    for wing in vehicle.wings: 
                        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                            for cabin in wing.cabins: 
                                cabin.filled_seats_arrangement  =  filling_order[f_o]     
                                pax =  1 if p_i == 0 else int(percent_pax[p_i] *  vehicle_0.wings[wing.tag].cabins[cabin.tag].number_of_passengers)
                                cabin_mass = 1E-6 if p_i == 0 else  pax * W_PAX_per_pax
                                cabin.number_of_passengers = pax 
                                cabin.mass_properties.mass = cabin_mass
                
                    #------------------------------------------------------------------------  
                    # Update Cargo
                    #------------------------------------------------------------------------
                    for cargo_bay_i ,  cargo_bay in enumerate(vehicle.cargo_bays): 
                        cargo_bay.mass_properties.mass   = percent_cargo[c_i] * W_CARGO *  cargo_bay_ratio[cargo_bay_i]

                    vehicle.mass_properties.cargo    =  percent_cargo[c_i] * W_CARGO
                    vehicle.mass_properties.payload  = (W_PAX_per_pax) *vehicle.number_of_passengers +  vehicle.mass_properties.cargo         
                
                    # loop through fuel tanks 
                    for network in vehicle.networks: 
                        for fuel_tank_i , source in  enumerate(network.sources): 
                            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                                source.fuel.mass_properties.mass = percent_fuel[f_i] * FUEL * fuel_tank_ratio[fuel_tank_i]
            
                    # run weights analysis and store results
                    vehicle.mass_properties.fuel = percent_fuel[f_i] * FUEL 
                               
                    #------------------------------------------------------------------------  
                    # run weights analysis and store results
                    #------------------------------------------------------------------------ 
                    counter =  compute_aircraft_load_data_point(vehicle,cruise_segment_tag,RES,counter,
                                                                total_sims, neutral_point_0,percent_pax[p_i], percent_cargo[c_i], percent_fuel[f_i],
                                                                f_o,p_i, c_i, f_i)

  
          
    #============================================================================================  
    # Trim Diagram Data
    #============================================================================================  

    total_sims = discretization ** 2
    counter    = 0        
    for w_i in range(discretization):    
        for c_g_i in range(discretization): 
            vehicle = deepcopy(vehicle_0)
        
            # Aircraft-Level Properties  
            vehicle.mass_properties.payload                 = PLD_per_pax   if w_i == 0 else percent_weight[w_i] * PLD
            vehicle.mass_properties.cargo                   = percent_weight[w_i] * W_CARGO
            vehicle.mass_properties.center_of_gravity[0][0] = x_cg_0[0][0] * percent_cg_shift[c_g_i]   
            vehicle.number_of_passengers                    = 1 if w_i == 0 else int(vehicle_0.number_of_passengers *  percent_weight[w_i]) 
            vehicle.mass_properties.fuel                    = percent_weight[w_i] * FUEL
             
            for network in vehicle.networks: 
                for source in  network.sources: 
                    if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                        source.fuel.mass_properties.mass = 0
                            
            for cargo_bay in vehicle.cargo_bays:  
                cargo_bay.mass_properties.mass   =  0
                
            for fuselage in  vehicle.fuselages: 
                for cabin in fuselage.cabins:
                    cabin.number_of_passengers = 0
                    
            for wing in  vehicle.wings: 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    for cabin in wing.cabins:    
                        cabin.number_of_passengers = 0 
              
            #------------------------------------------------------------------------  
            # Update Passengers 
            #------------------------------------------------------------------------  
            # run weights analysis and store results
            vehicle.number_of_passengers  =  1 if w_i == 0 else int(percent_weight[w_i] * PAX ) 
            for fuselage in  vehicle.fuselages: 
                for cabin in fuselage.cabins:  
                    pax =  1 if w_i == 0 else int(percent_weight[w_i] *  vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].number_of_passengers)  
                    cabin.number_of_passengers = pax
            

            for wing in vehicle.wings: 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    for cabin in wing.cabins:  
                        pax =  1 if w_i == 0 else int(percent_weight[w_i] *  vehicle_0.wings[wing.tag].cabins[cabin.tag].number_of_passengers)  
                        cabin.number_of_passengers = pax 
        
            #------------------------------------------------------------------------  
            # Update Cargo
            #------------------------------------------------------------------------
            for cargo_bay in vehicle.cargo_bays: 
                cargo_bay.mass_properties.mass   = percent_cargo[w_i]  * W_CARGO *  cargo_bay_ratio[cargo_bay_i]
                

            vehicle.mass_properties.cargo    =  percent_cargo[w_i] * W_CARGO
            vehicle.mass_properties.payload  = (W_PAX_per_pax) *vehicle.number_of_passengers +  vehicle.mass_properties.cargo         
        
            # loop through fuel tanks 
            for network in vehicle.networks:
                for source in  network.sources: 
                    if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                        source.fuel.mass_properties.mass = percent_fuel[w_i]  * FUEL * fuel_tank_ratio[fuel_tank_i]
    
            # run weights analysis and store results
            vehicle.mass_properties.fuel = percent_fuel[w_i] * FUEL
            
            cargo_weight =  percent_cargo[w_i]  * W_CARGO 
            pax_weight   =  percent_fuel[w_i] * W_PAX
            fuel_weight  =  percent_fuel[w_i] * FUEL
            vehicle.mass_properties.takeoff   =    OEW  + pax_weight + cargo_weight +  fuel_weight
            
            # Run aerodynamic analysis    
            counter =  compute_aircraft_trim_data_point(vehicle,cruise_segment_tag,RES,counter,total_sims,neutral_point_0,w_i,c_g_i)
   
    return RES


def compute_aircraft_load_data_point(vehicle,cruise_segment_tag,RES,counter,
                                     total_sims,neutral_point,percent_pax, percent_cargo, percent_fuel,
                                     f_o,p_i,c_i,f_i):
     
        
    # run geometry and mass properties analyes 
    center_of_gravity, mass, moment, _  = compute_vehicle_center_of_gravity(vehicle,
                                            centre_of_gravity_df = pd.DataFrame(columns=[ "Component", "Mass (kg)", "CG x (m)", "CG y (m)","CG z (m)"]),
                                            overwrite_center_of_gravity = True,
                                            segment=None,
                                            verbose=False) 
    
    # store results 
    RES.loading_results.CG_location[f_o,p_i,c_i,f_i]              = center_of_gravity[0][0]  
    RES.loading_results.mass[f_o,p_i,c_i,f_i]                     = mass[0]  
    RES.loading_results.LEMAC_location[f_o,p_i,c_i,f_i]           = vehicle.LEMAC  
    RES.loading_results.CG_percent_of_LEMAC_location[f_o,p_i,c_i,f_i]   =  (RES.loading_results.CG_location[f_o,p_i,c_i,f_i]  - vehicle.LEMAC) / vehicle.reference_chord
     
    print('***************************************')
    print('Loading Diagram Data Point: ' + str(counter+1) + ' of ' +  str(total_sims))
    print('Mass                      : ', RES.loading_results.mass[f_o,p_i,c_i,f_i])
    print('OEW                       : ', vehicle.mass_properties.operating_empty)
    print('Percent Fuel              : ', percent_fuel*100 )
    print('Percent Cargo             : ', percent_cargo*100 )
    print('Percent Pax               : ', percent_pax*100 )
    print('Ref. Chord                : ', vehicle.reference_chord)
    print('LEMAC                     : ', vehicle.LEMAC )
    print('Center of Gravity Location: ', RES.loading_results.CG_location[f_o,p_i,c_i,f_i] )   
    print('Neutral Point Location    : ', neutral_point ) 
    print('LEMAC                     : ', RES.loading_results.LEMAC_location[f_o,p_i,c_i,f_i] ) 
    print('CG as % LEMAC             : ', RES.loading_results.CG_percent_of_LEMAC_location[f_o,p_i,c_i,f_i] )
    print('***************************************')     
    counter += 1   
     
    return counter 

def compute_aircraft_trim_data_point(vehicle,cruise_segment_tag,RES,
                                     counter,total_sims,neutral_point,w_i,c_g_i):
 

    # store results  
    RES.trim_results.neutral_point[w_i,c_g_i]            = neutral_point
    RES.trim_results.static_margin[w_i,c_g_i]            = (RES.trim_results.neutral_point[w_i,c_g_i]  - vehicle.mass_properties.center_of_gravity[0][0]) /vehicle.reference_chord
    RES.trim_results.mass[w_i,c_g_i]                     =  vehicle.mass_properties.takeoff 
    RES.trim_results.LEMAC_location[w_i,c_g_i]           =  vehicle.LEMAC 
    RES.trim_results.CG_percent_of_LEMAC_location[w_i,c_g_i]   = (vehicle.mass_properties.center_of_gravity[0][0] - vehicle.LEMAC) / vehicle.reference_chord

    print('***************************************')
    print('Trim Diagram Data Point  : ' + str(counter+1) + ' of ' +  str(total_sims))
    print('Center of Gravity        : ',vehicle.mass_properties.center_of_gravity[0][0])
    print('OEW                      : ', vehicle.mass_properties.operating_empty)
    print('% LEMAC Location         : ', RES.trim_results.LEMAC_location[w_i,c_g_i] )
    print('Neutral Point            : ',RES.trim_results.neutral_point[w_i,c_g_i])
    print('Static Margin            : ',RES.trim_results.static_margin[w_i,c_g_i]) 
    print('Mass                     : ',RES.trim_results.mass[w_i,c_g_i] ) 
    print('CG as % LEMAC            : ', RES.trim_results.CG_percent_of_LEMAC_location[w_i,c_g_i] )
    print('***************************************')  

    counter += 1    
    return counter 
