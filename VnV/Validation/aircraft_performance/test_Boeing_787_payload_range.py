# RESEARCH/Aircraft/Boeing_787.py
# 
# 
# Created:  May 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import sys, os
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib as mpl
from matplotlib.gridspec import GridSpec

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Performance.compute_payload_range_diagram     import compute_payload_range_diagram

sys.path.append(os.path.abspath(os.path.join(os.path.join(sys.path[0]), "../../Vehicles")))
import Boeing_787 as Boeing_787


def main():
    ti                   = time.time()
    
    vehicle  = Boeing_787.vehicle_setup()   
    configs  = Boeing_787.configs_setup(vehicle) 
    analyses = analyses_setup(configs) 
    mission  = payload_range_mission_setup(analyses)
    missions = missions_setup(mission)
     
    # run payload range analysis 
    payload_range_results =  compute_payload_range_diagram(mission = missions.base_mission, fuel_reserve_percentage = 0.1, delete_training_data = True)

    apm = {
        "range":            np.array([0., 5500., 9500., 10000.]) * Units.nmi,
        "payload":          np.array([44000., 44000., 9071.8474, 0.]),
        "oew_plus_payload": np.array([161025., 161025., 127005.864, 117934.016]),
    }
    plot_payload_range(payload_range_results, apm)

    # #### DO NOT CHANGE THESE VALUES WITHOUT CONSULTING THE AIRPORT PLANNING MANUAL FIRST ###############
    #  "Airport Planning Manual": {
    #     "range": [0, 5500, 9500, 10000]  nmi,
    #     "payload": (([44000, 44000, 9071.8474, 0]) lbs
    #     "payload + oew": (([161025, 161025, 127005.864, 117934.016]) lbs
    
    # #####################################################################################################
    # ########################################### WARNING #################################################
    # #### DO NOT CHANGE THESE VALUES WITHOUT CONSULTING THE AIRPORT PLANNING MANUAL FIRST ################
    # ########################################### WARNING #################################################
    # #####################################################################################################
        
    truth_values = {
        "range":            np.array([       0.        , 10090424.13008407, 17488065.51305655, 18153977.03778774]),
        "payload":          np.array([44000.        , 44000.        , 10317.36918343,     0.        ]),
        "oew_plus_payload": np.array([160289.63081657, 160289.63081657, 126607.        , 116289.63081657]),
        "fuel":             np.array([     0.        ,  67640.36918343, 101323.        , 101323.        ]),
        "takeoff_weight":   np.array([     0.        , 227930.        , 227930.        , 217612.63081657]),
    }
    # ########################################### WARNING #################################################
    ###### DO NOT CHANGE THESE VALUES WITHOUT CONSULTING THE AIRPORT PLANNING MANUAL FIRST ################
    ###### NO MATTER HOW SMALL THE DIFFERENCE IS, THE SMALL CHANGES ADD UP OVER MULTIPLE PRs ##############
    #######################################################################################################
    ############################################# WARNING #################################################
    #######################################################################################################
            
    # Tolerance checks
    for key in truth_values:
        denom = np.atleast_1d(truth_values[key])
        numer = np.abs(np.atleast_1d(payload_range_results[key]) - denom)

        # Avoid division by zero
        with np.errstate(divide='ignore', invalid='ignore'):
            rel_error = np.where(denom != 0, numer / denom, 0.0)
            error = np.max(rel_error)

        assert error < 5e-3, f"{key} error too large: {error}"
    tf                   = time.time()
    elapsed_time         = round((tf-ti),2)
    print('Payload Range simulation Time: ' + str(elapsed_time) + ' seconds') 
            
    return 

def plot_payload_range(payload_range_results, apm):
    """
    Plot payload-range and OEW+payload-range against Airport Planning Manual reference.

    Parameters
    ----------
    payload_range_results : dict
        Keys: "range" [m], "payload" [lb], "oew_plus_payload" [lb]
    apm : dict
        Keys: "range" [m], "payload" [lb], "oew_plus_payload" [lb]
    """
    plt.style.use('bmh')
    mpl.rcParams["font.family"] = "Times New Roman"

    def nmi_to_km(x): return x * Units.nmi / Units.km
    def km_to_nmi(x): return x * Units.km / Units.nmi
    def lb_to_kg(y):  return y * Units.lbs
    def kg_to_lb(y):  return y / Units.lbs

    cmap   = plt.get_cmap("viridis")
    series = {
        "Boeing 787-8":          {"data": payload_range_results, "color": cmap(0.35), "ls": "-"},
        "Airport Planning Manual":{"data": apm,                  "color": "black",    "ls": "--"},
    }

    fig = plt.figure(figsize=(16, 8))
    gs  = GridSpec(1, 2, width_ratios=[1, 1], wspace=0.6)
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])

    for label, s in series.items():
        d     = s["data"]
        x_nmi = d["range"] / Units.nmi
        kw    = dict(label=label, linewidth=2.0, marker="o", markersize=5,
                     color=s["color"], linestyle=s["ls"])
        ax1.plot(x_nmi, d["payload"],          **kw)
        ax2.plot(x_nmi, d["oew_plus_payload"],  **kw)

    ax1.set_ylabel("Payload (lb)",        fontsize=22, fontweight="bold")
    ax1.set_xlabel("Range (nmi)",          fontsize=22, fontweight="bold")
    ax2.set_ylabel("Payload + OEW (lb)",  fontsize=22, fontweight="bold")
    ax2.set_xlabel("Range (nmi)",          fontsize=22, fontweight="bold")

    secax1 = ax1.secondary_xaxis("top", functions=(nmi_to_km, km_to_nmi))
    secax1.set_xlabel("Range (km)", fontsize=22, fontweight="bold")
    secax2 = ax2.secondary_xaxis("top", functions=(nmi_to_km, km_to_nmi))
    secax2.set_xlabel("Range (km)", fontsize=22, fontweight="bold")

    secay1 = ax1.secondary_yaxis("right", functions=(lb_to_kg, kg_to_lb))
    secay1.set_ylabel("Payload (kg)",        fontsize=22, fontweight="bold")
    secay2 = ax2.secondary_yaxis("right", functions=(lb_to_kg, kg_to_lb))
    secay2.set_ylabel("Payload + OEW (kg)", fontsize=22, fontweight="bold")

    for ax in [ax1, ax2, secax1, secax2]:
        ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x:,.0f}"))
    for ax in [ax1, ax2, secay1, secay2]:
        ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda y, _: f"{y:,.0f}"))
    for ax in [ax1, ax2]:
        ax.tick_params(axis="both", which="major", labelsize=18)
        ax.grid(True, linestyle=":", linewidth=0.8, alpha=0.6)
    for ax in [secax1, secax2, secay1, secay2]:
        ax.tick_params(axis="both", which="major", labelsize=18)

    ax1.axvline(x=5500, color="gray", linestyle="--", linewidth=1.2)
    ax1.set_xlim(-200, 12000)
    ax1.set_ylim(0, 60000)
    ax1.xaxis.set_major_locator(ticker.MultipleLocator(2000))
    ax1.yaxis.set_major_locator(ticker.MultipleLocator(5000))
    ax2.set_xlim(-200, 12000)
    ax2.set_ylim(110000, 175000)
    ax2.xaxis.set_major_locator(ticker.MultipleLocator(2000))

    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(
        handles, labels,
        loc="lower center", ncol=2, frameon=True, fontsize=14,
        bbox_to_anchor=(0.5, 0.03), framealpha=0.95, edgecolor="black",
    )
    fig.subplots_adjust(bottom=0.2)
    plt.tight_layout()
    plt.show()


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def payload_range_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type = 'root_finder'
    

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start           = 30.* Units.knots
    segment.velocity_end             = 167.0 * Units['knots']
    segment.friction_coefficient     = 0.03
    segment.altitude                 = 0.0   
    segment.throttle                 = 1.0

    segment.assigned_control_variables.ground_velocity.active  = True  
    segment.assigned_control_variables.ground_velocity.bounds  = [[-2, 120]]

    mission.append_segment(segment)
      
    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start                                           = 0.0 * Units['knots'] 
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 167.0 * Units['knots']
    segment.air_speed_start                                          = 175.0 * Units['knots']
    segment.climb_rate                                               = 250 * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.air_speed_start                                            = 0.0 * Units['knots'] 
    segment.altitude_end                                             = 1000  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback ) 
    segment.air_speed_start                                          = 200.0 * Units['knots'] 
    segment.altitude_end                                             = 8000   * Units['ft']
    segment.air_speed_end                                            = 300 * Units['knots']
    segment.climb_rate                                               = 1700   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 16000   * Units['ft']
    segment.air_speed                                                = 350 * Units['knots']
    segment.climb_rate                                               = 1300   * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)


    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 35000   * Units['ft']
    segment.air_speed                                                = 450 * Units['knots']
    segment.climb_rate                                               = 1000   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment) 

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 35000 * Units['ft']  
    segment.air_speed                                                = 450 * Units['knots']
    segment.distance                                                 = 7370 * Units.km   
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_end                                             = 10000   * Units.ft
    segment.air_speed                                                = 380 * Units['knots']
    segment.descent_rate                                             = 1850   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "approach" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 2000 * Units.ft
    segment.air_speed                                                = 225.0 * Units['knots']
    segment.descent_rate                                             = 650  * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = .0   * Units.ft
    segment.air_speed                                                = 175.0 * Units['knots']
    segment.descent_rate                                             = 600.0   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment) 


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.reverse_thrust ) 
    segment.velocity_start                                                = 160.0 * Units['knots']
    segment.velocity_end                                                  = 30 * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active                = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)     

 
    return mission



# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle =  vehicle

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
    weights.settings.FLOPS.fidelity                                          = 'Complex'      
    weights.settings.advanced_composites                                     = True
    weights.settings.weight_correction_additions.empty.structural.paint      = 450 
    weights.settings.weight_correction_additions.operational_items.ETOPS     = 7.7 * vehicle.number_of_passengers
    weights.settings.weight_correction_additions.empty.propulsion.battery    = 56 
    weights.settings.weight_correction_factors.empty.structural.landing_gear = 1.05    
    weights.settings.weight_correction_factors.empty.systems.electrical      = 2.67 
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
    
  

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    return analyses    

def missions_setup(mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)

    return missions
if __name__ == '__main__': 
    main()    