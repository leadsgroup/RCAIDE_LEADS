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
    payload_range_results =  compute_payload_range_diagram(mission = missions.base_mission, fuel_reserve_percentage = 0.10)

    apm = {
        "range":            np.array([0., 5500., 9500., 10000.]) * Units.nmi,
        "payload":          np.array([44000., 44000., 9071.8474, 0.]),
        "oew_plus_payload": np.array([161025., 161025., 127005.864, 117934.016]),
    }

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
        
    # FLOPS OEW is 2.6% below the APM (114,913 vs 117,934 kg), so the max-payload range is 4% above the APM
    truth_values = {
        "range":            np.array([       0.        , 10601618.27332284, 17759990.44359219, 18631784.57780205]),
        "payload":          np.array([44000.        , 44000.        , 11693.83       ,     0.        ]),
        "oew_plus_payload": np.array([158913.17      , 158913.17      , 126607.        , 114913.17      ]),
        "fuel":             np.array([     0.        ,  69016.83      , 101323.        , 101323.        ]),
        "takeoff_weight":   np.array([     0.        , 227930.        , 227930.        , 216236.17      ]),
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

        computed = np.squeeze(np.atleast_1d(payload_range_results[key]))
        sign     = "+" if np.squeeze(computed - denom).flat[np.argmax(rel_error)] >= 0 else "-"
        print(f"  {key}:")
        print(f"    truth    = {np.squeeze(denom)}")
        print(f"    computed = {computed}")
        print(f"    error    = {sign}{error * 100:.4f}%")
        assert error < 5e-3, f"{key} error too large: {error}"
    tf                   = time.time()
    elapsed_time         = round((tf-ti),2)
    print('Payload Range simulation Time: ' + str(elapsed_time) + ' seconds')

    plot_payload_range(payload_range_results, apm)    
            
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
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start           = 10.* Units.knots
    segment.velocity_end             = 92.6 * Units['m/s']
    segment.friction_coefficient     = 0.04
    segment.altitude                 = 0.0
    # full power on the ground run is the engine's takeoff rating
    segment.throttle = analyses.takeoff.vehicle.networks.fuel.propulsors['propulsor_1'].rated_takeoff_throttle
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate - to screen height  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_CAS_Constant_Rate(base_segment)
    segment.tag = "first_segment" 
    segment.analyses.extend( analyses.transition ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 35.0   * Units.ft
    segment.calibrated_air_speed      = 180.0 * Units.kts
    segment.climb_rate     = 10.0   * Units['m/s']  
     
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.pitch_angle.active             = True                 
    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant CAS Constant Rate - to 400ft 
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_CAS_Constant_Rate(base_segment)
    segment.tag = "second_segment" 
    segment.analyses.extend( analyses.initial_climb ) 
    segment.altitude_end   = 400   * Units.ft
    segment.calibrated_air_speed      = 180.0 * Units.kts
    segment.climb_rate     = 9.0   * Units['m/s']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.pitch_angle.active             = True                  
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Segment: Constant Acceleration Constant Altitude (from V2 to climb speed)
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "third_segment" 
    segment.analyses.extend( analyses.accel ) 
    segment.air_speed_start      = 180.0 * Units.kts
    segment.air_speed_end      = 251.4 * Units.kts
    segment.acceleration = 1.25
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.pitch_angle.active             = True
    segment.assigned_control_variables.angle_of_attack.active             = True                  
    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Fourth Climb Segment: Constant Throttle Constant TAS (250kts @ 950ft) 
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "fourth_segment" 
    segment.analyses.extend( analyses.cutback )
    segment.air_speed = 253.361 * Units.kts
    segment.altitude_end   = 1500   * Units.ft
    segment.throttle = 0.94
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls            
    segment.assigned_control_variables.angle_of_attack.active                 = True
    segment.assigned_control_variables.angle_of_attack.initial_guess          = True
    segment.assigned_control_variables.angle_of_attack.initial_guess_values   = [[ 1.0 * Units.deg]]
    segment.assigned_control_variables.pitch_angle.active                 = True
    segment.assigned_control_variables.pitch_angle.initial_guess          = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values   = [[ 5.0 * Units.deg]]
    
    mission.append_segment(segment)
     
    # ------------------------------------------------------------------
    #   Climb Segment to 2500ft: Constant Throttle Constant TAS (250kts @ 2000ft)
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "2500_segment" 
    segment.analyses.extend( analyses.cruise )
    segment.air_speed = 257.147 * Units.kts
    segment.altitude_end   = 2500   * Units.ft
    segment.throttle = 0.94
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.angle_of_attack.active                 = True
    segment.assigned_control_variables.angle_of_attack.initial_guess          = True
    segment.assigned_control_variables.angle_of_attack.initial_guess_values   = [[ 1.0 * Units.deg]]
    segment.assigned_control_variables.pitch_angle.active                 = True
    segment.assigned_control_variables.pitch_angle.initial_guess          = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values   = [[ 5.0 * Units.deg]]
    
    mission.append_segment(segment)   
    
    # ------------------------------------------------------------------
    #   Climb Segment to 7500ft: Constant Throttle Constant TAS (250kts CAS @ 5000ft)
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "7500_segment" 
    segment.analyses.extend( analyses.cruise )
    segment.air_speed = 268.401 * Units.kts
    segment.altitude_end   = 7500   * Units.ft
    segment.throttle = 0.9
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.angle_of_attack.active                 = True
    segment.assigned_control_variables.angle_of_attack.initial_guess          = True
    segment.assigned_control_variables.angle_of_attack.initial_guess_values   = [[ 1.0 * Units.deg]]
    segment.assigned_control_variables.pitch_angle.active                 = True
    segment.assigned_control_variables.pitch_angle.initial_guess          = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values   = [[ 5.0 * Units.deg]]
    
    mission.append_segment(segment)  
    
    # ------------------------------------------------------------------
    #   Climb Segment to 12500ft: Constant Throttle Constant TAS (250kts CAS @ 10000ft)
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "12500_segment" 
    segment.analyses.extend( analyses.cruise )
    segment.air_speed = 288.705 * Units.kts
    segment.altitude_end   = 12500   * Units.ft
    segment.throttle = 0.9  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.angle_of_attack.active                 = True
    segment.assigned_control_variables.angle_of_attack.initial_guess          = True
    segment.assigned_control_variables.angle_of_attack.initial_guess_values   = [[ 1.0 * Units.deg]]
    segment.assigned_control_variables.pitch_angle.active                 = True
    segment.assigned_control_variables.pitch_angle.initial_guess          = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values   = [[ 5.0 * Units.deg]]
    
    mission.append_segment(segment)  
    
    # ------------------------------------------------------------------
    #   Climb Segment to 17500ft: Constant Throttle Constant TAS (250ktsCAS @ 15000ft)
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "17500_segment" 
    segment.analyses.extend( analyses.cruise )
    segment.air_speed = 311.139 * Units.kts
    segment.altitude_end   = 17500   * Units.ft
    segment.throttle = 0.9
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls          
    segment.assigned_control_variables.angle_of_attack.active                 = True
    segment.assigned_control_variables.angle_of_attack.initial_guess          = True
    segment.assigned_control_variables.angle_of_attack.initial_guess_values   = [[ 1.0 * Units.deg]]
    segment.assigned_control_variables.pitch_angle.active                 = True
    segment.assigned_control_variables.pitch_angle.initial_guess          = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values   = [[ 5.0 * Units.deg]]                
    
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #   Climb Segment to 22500ft: Constant Throttle Constant TAS (250ktsCAS @ 15000ft)
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "22500_segment" 
    segment.analyses.extend( analyses.cruise )
    segment.air_speed = 335.948 * Units.kts
    segment.altitude_end   = 22500   * Units.ft
    segment.throttle = 0.9
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls        
    segment.assigned_control_variables.angle_of_attack.active                 = True
    segment.assigned_control_variables.angle_of_attack.initial_guess          = True
    segment.assigned_control_variables.angle_of_attack.initial_guess_values   = [[ 1.0 * Units.deg]]
    segment.assigned_control_variables.pitch_angle.active                 = True
    segment.assigned_control_variables.pitch_angle.initial_guess          = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values   = [[ 5.0 * Units.deg]]             
    
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #   Climb Segment to 27500ft: Constant Throttle Constant TAS (250ktsCAS @ 25000ft)
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "27500_segment" 
    segment.analyses.extend( analyses.cruise )
    segment.air_speed = 363.391 * Units.kts
    segment.altitude_end   = 27500   * Units.ft
    segment.throttle = 0.9
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.angle_of_attack.active                 = True
    segment.assigned_control_variables.angle_of_attack.initial_guess          = True
    segment.assigned_control_variables.angle_of_attack.initial_guess_values   = [[ 1.0 * Units.deg]]
    segment.assigned_control_variables.pitch_angle.active                 = True
    segment.assigned_control_variables.pitch_angle.initial_guess          = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values   = [[ 5.0 * Units.deg]]               
    
    mission.append_segment(segment)       
    
    # ------------------------------------------------------------------
    #   Final Climb Segment to 35000ft: Linear Mach Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "ICA_segment" 
    segment.analyses.extend( analyses.cruise )
    segment.mach_number_start = 0.61
    segment.mach_number_end = 0.8
    segment.altitude_end   = 35000   * Units.ft
    segment.climb_rate = 1.78   * Units['m/s'] 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.pitch_angle.active             = True                  
    
    mission.append_segment(segment) 
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Mach Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                      = 35000.0 * Units.ft  
    segment.mach_number                                   = 0.8
    segment.distance                                      = 1000.0 * Units.nmi   
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.pitch_angle.active             = True   
    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Descent Segment: cruise Mach to the 300 KCAS crossover altitude
    # ------------------------------------------------------------------

    segment = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_start                                = 35000.0  * Units.ft
    segment.altitude_end                                  = 29700.0  * Units.ft
    segment.mach_number_start                             = 0.8
    segment.mach_number_end                               = 0.8
    segment.descent_rate                                  = 2000.0 * Units['ft/min']
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.pitch_angle.active             = True     
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Descent Segment: 300 KCAS to 10000 ft
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_CAS_Constant_Rate(base_segment)
    segment.tag = "descent_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 10000.0  * Units.ft
    segment.calibrated_air_speed                          = 300.0 * Units.kts
    segment.descent_rate                                  = 2000.0 * Units['ft/min']
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.pitch_angle.active             = True     
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: 250 KCAS to 5000 ft
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_CAS_Constant_Rate(base_segment)
    segment.tag = "descent_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 5000.0   * Units.ft
    segment.calibrated_air_speed                          = 250.0 * Units.kts
    segment.descent_rate                                  = 1500.0 * Units['ft/min']
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.pitch_angle.active             = True     
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Approach Segment: Constant CAS Constant Rate to the runway
    # ------------------------------------------------------------------
    segment = Segments.Descent.Constant_CAS_Constant_Rate(base_segment)
    segment.tag = "approach"
    segment.analyses.extend( analyses.approach )
    segment.altitude_start                                = 5000.0   * Units.ft
    segment.altitude_end                                  = 0.0   * Units.ft
    segment.calibrated_air_speed                          = 180.0 * Units.kts
    segment.descent_rate                                  = 2.0   * Units['m/s']

    # define flight dynamics to model
    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    # define flight controls
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']]
    segment.assigned_control_variables.pitch_angle.active             = True
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Landing Roll
    # ------------------------------------------------------------------
    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"
    segment.analyses.extend( analyses.landing )
    segment.velocity_end                                  = 20 * Units.knots
    segment.friction_coefficient                          = 0.4
    segment.altitude                                      = 0.0
    segment.assigned_control_variables.elapsed_time.active           = True
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
    weights.settings.weight_correction_additions.empty.propulsion.battery    = 150 
    weights.settings.weight_correction_factors.empty.structural.landing_gear = 1.1
    weights.settings.weight_correction_factors.empty.systems.electrical      = 2.7
    weights.settings.weight_correction_factors.empty.propulsion.engines      = 1.031
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