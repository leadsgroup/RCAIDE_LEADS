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

#------------------------------------------------------------------------------
# compute_load_and_trim_diagram
#------------------------------------------------------------------------------  
def compute_load_and_trim_diagram(mission, cruise_segment_tag = "cruise",number_of_points = 5):
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
    
    for segment in  mission.segments:
    
        if segment.analyses.aerodynamics == None:
            raise AttributeError('Aerodynamic analysis not set') 
        
        if segment.analyses.weights  == None:
            raise AttributeError('Weights analysis not set')
        
        if segment.analyses.stability  == None:
            raise AttributeError('Stability analysis not set')
        
        vehicle = segment.analyses.vehicle 
        vehicle.mass_properties.takeoff  = None 
    
        # check that cabins are defined with at least one class
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
  
    results  = mission.evaluate() 
         
    #------------------------------------------------------------------------  
    # Compute Loading Points 
    #------------------------------------------------------------------------
    percent_cargo        =  np.hstack((np.zeros(number_of_points), np.linspace(0, 1, number_of_points)))
    percent_pax          =  np.hstack((np.linspace(0, 1, number_of_points), np.ones(number_of_points)))
    percent_cargo        =  np.tile(percent_cargo, 2)
    percent_pax          =  np.tile(percent_pax, 2)
    fill_order           =  sorted( [ 'ascending', 'descending']*int(len(percent_pax)/2))
    percent_fuel         =  np.linspace(0, 1, number_of_points) 
    
    # create empty data structures 
    loading_mass                     = np.zeros((len(percent_cargo),len(percent_fuel)))
    loading_CG_location              = np.zeros((len(percent_cargo),len(percent_fuel)))
    loading_LEMAC_location           = np.zeros((len(percent_cargo),len(percent_fuel)))
    
    # compute mass properties of aircraft to get weight distribution
    vehicle_0         = segment.analyses.vehicle
    x_cg_0            = segment.analyses.vehicle.mass_properties.center_of_gravity
    weight_breakdown  = segment.analyses.vehicle.mass_properties.weight_breakdown 
    neutral_point_0   = segment.analyses.vehicle.neutral_point
                        
     
    CARGO =  weight_breakdown.payload.cargo 
    BAG   =  weight_breakdown.payload.baggage 
    PAX   =  weight_breakdown.payload.passengers 
    MTOW  =  vehicle_0.mass_properties.max_takeoff
    OEW   =  weight_breakdown.empty.total
    
    if vehicle_0.mass_properties.max_landing ==0: 
        MLW  =  estimate_maximum_landing_weight(MTOW)
    else:
        MLW  =  vehicle_0.mass_properties.max_landing

    # -------------------------------------------------------------------------
    # Load Diagram Data 
    # -------------------------------------------------------------------------     
    total_sims = len(percent_cargo) * len(percent_fuel)
    counter    = 0
    for i in range(len(percent_cargo)):
        for j in range(len(percent_fuel)):
 
            # Aircraft-Level Properties  
            vehicle.mass_properties.takeoff  = None # this ensures that the takeoff weight is computed
            vehicle.mass_properties.payload  = (BAG + PAX) * percent_pax[i] +  percent_cargo[i] *CARGO 
            vehicle.mass_properties.cargo    = percent_cargo[i] * CARGO 
            vehicle.number_of_passengers     = np.maximum(1,int(vehicle_0.number_of_passengers * percent_pax[i]))
 
            # Update Passengers           
            for fuselage in  vehicle.fuselages: 
                for cabin in fuselage.cabins:
                    cabin.filled_seats_arrangement  = fill_order[i] 
            for wing in vehicle.wings: 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    for cabin in wing.cabins:    
                        cabin.filled_seats_arrangement  = fill_order[i] 
             
            # Update Fuel            
            for network in  vehicle.networks:
                for source  in  network.sources:
                    if isinstance(source,RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                        source.fuel.mass_properties.mass = percent_fuel[j] * vehicle_0.networks[network.tag].sources[source.tag].fuel.mass_properties.mass
        
            #  run mission   
            mission = update_analyses(mission, overwrite_fuel_volume = False)
            
            geometry(mission)
            mass_properties(mission) 
            
            # store results 
            segment  = mission.segments[cruise_segment_tag]
            loading_CG_location[i,j]         = segment.analyses.vehicle.mass_properties.center_of_gravity[0][0] 
            loading_mass[i,j]                = segment.analyses.vehicle.mass_properties.takeoff 
            loading_LEMAC_location[i,j]      = 100 * (loading_CG_location[i,j] - segment.analyses.vehicle.LEMAC) / segment.analyses.vehicle.reference_chord
            
            print('***************************************')
            print('Loading Diagram Data: ' + str(counter+1) + ' of ' +  str(total_sims))
            print('Mass                : ', loading_mass[i,j])
            print('Percent Fuel        : ', percent_fuel[j]*100 )
            print('Percent Cargo       : ', percent_cargo[i]*100 )
            print('Percent Pax         : ', percent_pax[i]*100 )
            print('LEMAC               : ', loading_LEMAC_location[i,j] )
            print('***************************************')
             

            counter += 1
          
    # -------------------------------------------------------------------------
    # Trim Diagram Data
    # ------------------------------------------------------------------------- 
    percent_mass                     = np.linspace(0,1, number_of_points)
    percent_cg_shift                 = np.linspace(0.8,1.2, number_of_points)
    aerodynamic_lift_coefficient     = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_drag_coefficient     = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_moment_coefficient   = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_neutral_point        = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_static_margin        = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_moment               = np.zeros((len(percent_mass),len(percent_cg_shift))) 
    aerodynamic_mass                 = np.zeros((len(percent_mass),len(percent_cg_shift))) 
    aerodynamic_LEMAC_location       = np.zeros((len(percent_mass),len(percent_cg_shift))) 

    total_sims = len(percent_mass) * len(percent_cg_shift)
    counter    = 0        
    #for k in range(len(percent_mass)):
        #for l in range(len(percent_cg_shift)):
    
            ## Aircraft-Level Properties  
            #vehicle.mass_properties.takeoff                 = None # this ensures that the takeoff weight is computed 
            #vehicle.mass_properties.payload                 = (BAG + PAX) * percent_mass[k] +  percent_mass[k] *CARGO 
            #vehicle.mass_properties.cargo                   = percent_mass[k] * CARGO
            #vehicle.mass_properties.center_of_gravity[0][0] = x_cg_0[0][0] * percent_cg_shift[l]  
            #vehicle.number_of_passengers                    = np.maximum(1,int(vehicle_0.number_of_passengers *  percent_mass[k]))
    
            ## Update Passengers           
            #for fuselage in  vehicle.fuselages: 
                #for cabin in fuselage.cabins:
                    #cabin.filled_seats_arrangement  = fill_order[i] 
            #for wing in vehicle.wings: 
                #if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    #for cabin in wing.cabins:    
                        #cabin.filled_seats_arrangement  = fill_order[i]  
            ## Update Fuel            
            #for network in  vehicle.networks: 
                #for source  in  network.sources:
                    #if isinstance(source,RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                        #source.fuel.mass_properties.mass = percent_mass[k] * vehicle_0.networks[network.tag].sources[source.tag].fuel.mass_properties.mass
            
            ##  run mission  
            #mission  = update_analyses(mission,overwrite_fuel_volume = False, update_center_of_gravity = False,neutral_point= neutral_point_0)  
            #results  = mission.evaluate() 
            #segment  = results.segments[cruise_segment_tag]
            
            ## store results 
            #aerodynamic_lift_coefficient[k,l]    = segment.state.conditions.aerodynamics.coefficients.lift.total[0][0]  
            #aerodynamic_drag_coefficient[k,l]    = segment.state.conditions.aerodynamics.coefficients.drag.total[0][0]  
            #aerodynamic_moment_coefficient[k,l]  = segment.state.conditions.static_stability.coefficients.M[0][0]         
            #aerodynamic_moment[k,l]              = segment.state.conditions.frames.inertial.total_moment_vector[0][1]
            #aerodynamic_neutral_point[k,l]       = segment.state.conditions.static_stability.neutral_point[0][0]  
            #aerodynamic_static_margin[k,l]       = segment.state.conditions.static_stability.static_margin[0][0]    
            #aerodynamic_mass[k,l]                = segment.analyses.vehicle.mass_properties.takeoff 
            #aerodynamic_LEMAC_location[k,l]      = 100 * (vehicle.mass_properties.center_of_gravity[0][0] - segment.analyses.vehicle.LEMAC) / segment.analyses.vehicle.reference_chord
            
            #counter += 1
            #print('***************************************')
            #print('Trim Diagram Data: ' + str(counter) + ' of ' +  str(total_sims))
            #print('Center of Gravity : ',vehicle.mass_properties.center_of_gravity[0][0])
            #print('Neutral Point     : ',aerodynamic_neutral_point[k,l])
            #print('Static Margin     : ',aerodynamic_static_margin[k,l])
            #print('Percent Mass      : ',percent_mass[k]*100 ) 
            #print('Mass              : ', aerodynamic_mass[k,l] ) 
            #print('***************************************')
 
  
    RES = Data(
               number_of_points                = number_of_points,
               loading_CG_location             = loading_CG_location, 
               loading_mass                    = loading_mass,
               loading_LEMAC_location          = loading_LEMAC_location, 
               aerodynamic_lift_coefficient    = aerodynamic_lift_coefficient,
               aerodynamic_drag_coefficient    = aerodynamic_drag_coefficient,  
               aerodynamic_moment_coefficient  = aerodynamic_moment_coefficient, 
               aerodynamic_moment              = aerodynamic_moment,            
               aerodynamic_neutral_point       = aerodynamic_neutral_point,     
               aerodynamic_static_margin       = aerodynamic_static_margin,
               aerodynamic_mass                = aerodynamic_mass,           
               aerodynamic_LEMAC_location      = aerodynamic_LEMAC_location,   
               MTOW                            = MTOW,          
               OEW                             = OEW, 
               MLW                             = MLW,  
               )
    
    return RES  
  
def update_analyses(mission,overwrite_fuel_volume=True, update_center_of_gravity=True, neutral_point = None):
    
    for segment in  mission.segments:
        segment.analyses.vehicle.neutral_point                        = neutral_point 
        segment.analyses.geometry.settings.overwrite_fuel_volume      = overwrite_fuel_volume 
        segment.analyses.weights.settings.update_moment_of_inertia    = update_center_of_gravity 
        segment.analyses.weights.settings.update_center_of_gravity    = update_center_of_gravity
        segment.analyses.weights.print_weight_analysis_report         = False  
        #segment.analyses.stability.settings.update_center_of_gravity  = update_center_of_gravity
    return mission
 