# ATR_72.py

# Created: 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from   RCAIDE.Framework.Core import Units, Data
from   RCAIDE.Library.Plots  import *   
from RCAIDE.Library.Plots.Common import set_axes, plot_style

# python imports 
import numpy as np   
import matplotlib.pyplot as plt 
import matplotlib.cm as cm
import os 
import sys

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles')) 
from ATR_72                          import vehicle_setup as conventional_vehicle_setup
from ATR_72                          import configs_setup as conventional_configs_setup
from all_electric_ATR_72             import vehicle_setup as all_electric_vehicle_setup
from all_electric_ATR_72             import configs_setup as all_electric_configs_setup
from series_hybrid_electric_ATR_72   import configs_setup as series_hybrid_configs_setup
from series_hybrid_electric_ATR_72   import vehicle_setup as series_hybrid_vehicle_setup
from parallel_hybrid_electric_ATR_72 import configs_setup as parallel_hybrid_configs_setup
from parallel_hybrid_electric_ATR_72 import vehicle_setup as parallel_hybrid_vehicle_setup 

import time 

def main():
    
    solver_type      = "optimize"
    solver_objective = None  
    conventional     = True
    all_electric     = True
    series_hybrid    = True
    parallel_hybrid  = True
    
    convetional_cruise_CL_truth      = 0.6881157129805212
    electric_cruise_CL_truth         = 0.6940128366994059
    series_hybrid_cruise_CL_truth    = 0.6940130981166039
    parallel_hybrid_cruise_CL_truth  = 0.6939609375214035

    error = Data()
    
    plot_data =  []
    powertrain_labels = []
    t0=time.time()
    if conventional:
        print("\n Conventional Powertrain Test") 
        vehicle  = conventional_vehicle_setup() 
        vehicle.networks.fuel.identical_propulsors = False         
        configs  = conventional_configs_setup(vehicle) 
        analyses = analyses_setup(configs) 
        missions = missions_setup(analyses,solver_type,solver_objective)  
        conventional_results  = missions.base_mission.evaluate()

        cruise_CL        = conventional_results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0]   
        print("Conventional ATR 72 Cruise CL: " + str(cruise_CL)) 
        error.conventional_cruise_CL = np.max(np.abs( convetional_cruise_CL_truth  - cruise_CL  )/ convetional_cruise_CL_truth )  
        
        plot_data.append(conventional_results)
        powertrain_labels.append("Conventional")
    if all_electric:        
        print("\n All-Electric Powertrain Test")  
        vehicle  = all_electric_vehicle_setup()  
        vehicle.networks.electric.identical_propulsors = False      
        configs  = all_electric_configs_setup(vehicle) 
        analyses = analyses_setup(configs) 
        missions = missions_setup(analyses,solver_type,solver_objective)  
        electric_results  = missions.base_mission.evaluate()

        cruise_CL        = electric_results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0]   
        print("Electric ATR 72 Cruise CL: " + str(cruise_CL))   
        error.electric_cruise_CL = np.max(np.abs( electric_cruise_CL_truth  - cruise_CL  )/ electric_cruise_CL_truth )  
        
        plot_data.append(electric_results)
        powertrain_labels.append("All-Electric")
    if series_hybrid:        
        print("\n Series Hybrid Powertrain Test") 
        vehicle  = series_hybrid_vehicle_setup() 
        configs  = series_hybrid_configs_setup(vehicle) 
        analyses = analyses_setup(configs) 
        missions = missions_setup(analyses,solver_type,solver_objective)  
        series_hybrid_results  = missions.base_mission.evaluate()
    
        cruise_CL        = series_hybrid_results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0]
        print("Series Hybrid ATR 72 Cruise CL: " + str(cruise_CL))
        error.series_hybrid_cruise_CL= np.max(np.abs( series_hybrid_cruise_CL_truth  - cruise_CL  )/ series_hybrid_cruise_CL_truth )  
        
        plot_data.append(series_hybrid_results)
        powertrain_labels.append("Series Hybrid")
    if parallel_hybrid:        
        print("\n Parallel Hybrid Powertrain Test") 
        vehicle  = parallel_hybrid_vehicle_setup() 
        configs  = parallel_hybrid_configs_setup(vehicle) 
        analyses = analyses_setup(configs) 
        missions = missions_setup(analyses,solver_type,solver_objective)  
        parallel_hybrid_results  = missions.base_mission.evaluate() 
    
        cruise_CL        = parallel_hybrid_results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0]
        print("Parallel Hybrid ATR 72 Cruise CL: " + str(cruise_CL))
        error.parallel_hybrid_cruise_CL = np.max(np.abs( parallel_hybrid_cruise_CL_truth  - cruise_CL  )/ parallel_hybrid_cruise_CL_truth )
        
        plot_data.append(parallel_hybrid_results)
        powertrain_labels.append("Parallel Hybrid")
         

    # add remaining networks MATTEO          
    print("Elapsed Time", (time.time()-t0)/60)         

    print('Errors:')
    print(error) 
    for k,v in list(error.items()): 
        assert(np.abs(v)<1e-6)
        
    plot_battery_pack_conditions(plot_data,powertrain_labels,save_figure = False, show_legend = False,)
    
    return




# ----------------------------------------------------------------------
#   Define the Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

# ----------------------------------------------------------------------
#   Define the Base Analysis
# ----------------------------------------------------------------------

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()  

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------     
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle                            = vehicle 
    analyses.append(aerodynamics) 

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------     
    energy= RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle = vehicle 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    # ------------------------------------------------------------------     
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    # ------------------------------------------------------------------     
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses 

# ----------------------------------------------------------------------
#   Define the Missions
# ----------------------------------------------------------------------
def missions_setup(analyses,solver_type,solver_objective): 

    # create base mission 
    mission          = mission_setup(analyses,solver_type,solver_objective)
    
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,solver_type,solver_objective):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type       = solver_type
    base_segment.state.numerics.solver.objective  = solver_objective  
        
    # ------------------------------------------------------------------
    #   Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "Cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                      = 25000  * Units.feet 
    segment.air_speed                                     = 270    * Units.kts
    segment.distance                                      = 100.   * Units.nautical_mile    
    segment.hybrid_power_split_ratio                      = 0.5 
    segment.battery_fuel_cell_power_split_ratio           = 1.0
    segment.initial_battery_state_of_charge               = 1.0 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.7]]
    segment.assigned_control_variables.body_angle.active             = True     
    mission.append_segment(segment)    
 
      
    return mission

 
def plot_battery_pack_conditions(plot_data,
                                 powertrain_labels, 
                                  save_figure = True,
                                  show_legend = True,
                                  save_filename = "Battery_Pack_Conditions_",
                                  file_type = ".png",
                                  width = 11, height = 7):
    
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
    

    line_colors   = cm.tab10(np.linspace(0, 1, 10)) 
     

    fig_1 = plt.figure()
    fig_1.set_size_inches(width,height)    
    axis_1_1 = fig_1.add_subplot(3,2,1)
    axis_1_2 = fig_1.add_subplot(3,2,2) 
    axis_1_3 = fig_1.add_subplot(3,2,3) 
    axis_1_4 = fig_1.add_subplot(3,2,4)
    axis_1_5 = fig_1.add_subplot(3,2,5) 
    axis_1_6 = fig_1.add_subplot(3,2,6)
    

    fig_2 = plt.figure()
    fig_2.set_size_inches(width,height)    
    axis_2_1 = fig_2.add_subplot(2,2,1)
    axis_2_2 = fig_2.add_subplot(2,2,2) 
    axis_2_3 = fig_2.add_subplot(2,2,3) 
    axis_2_4 = fig_2.add_subplot(2,2,4)
    

    fig_3 = plt.figure()
    fig_3.set_size_inches(width,height)    
    axis_3_1 = fig_3.add_subplot(2,2,1)
    axis_3_2 = fig_3.add_subplot(2,2,2) 
    axis_3_3 = fig_3.add_subplot(2,2,3) 
    axis_3_4 = fig_3.add_subplot(2,2,4)      
     
    for res_i ,  results in enumerate(plot_data):

        for i in range(len(results.segments)):
            # ---------------------------------------------------------------------------
            # Unpack properties to plot             
            # ---------------------------------------------------------------------------

            time         = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min 
            Weight       = results.segments[i].conditions.weights.total_mass[:, 0] * 9.81   
            mdot         = results.segments[i].conditions.weights.vehicle_mass_rate[:, 0]
            thrust       = results.segments[i].conditions.frames.body.thrust_force_vector[:, 0]
            sfc          = (mdot / Units.lb) / (thrust / Units.lbf) * Units.hr    
            cl           = results.segments[i].conditions.aerodynamics.coefficients.lift.total[:,0,None]
            cd           = results.segments[i].conditions.aerodynamics.coefficients.drag.total[:,0,None]
            aoa          = results.segments[i].conditions.aerodynamics.angles.alpha[:,0] / Units.deg
            l_d          = cl/cd    
            segment_tag  =  results.segments[i].tag
            segment_name = segment_tag.replace('_', ' ')
            

            # ---------------------------------------------------------------------------
            # Plot battery pack results if any            
            # ---------------------------------------------------------------------------            
            
            for network in results.segments[0].analyses.energy.vehicle.networks: 
                busses  = network.busses
                
                for  b_i , bus in  enumerate(busses):  
                    no_modules         = len(bus.battery_modules)   
                    bus_config         = bus.battery_module_electric_configuration 
                    battery_module_tag = list(bus.battery_modules.keys())[0]
                    
                    battery_conditions  = results.segments[i].conditions.energy[bus.tag].battery_modules[battery_module_tag] 
                 
                    if bus_config == 'Series':
                        pack_current        = battery_conditions.current[:,0] 
                        pack_volts          = battery_conditions.voltage_under_load[:,0]   * no_modules                          
                    elif bus_config  == 'Parallel': 
                        pack_current        = battery_conditions.current[:,0] * no_modules
                        pack_volts          = battery_conditions.voltage_under_load[:,0]   
                        
                    pack_power          = battery_conditions.power[:,0] * no_modules
                    pack_energy         = battery_conditions.energy[:,0] * no_modules
                    pack_SOC            = battery_conditions.cell.state_of_charge[:,0]   
                    pack_temperature    = battery_conditions.temperature[:,0]   
                
                    if i ==0:                             
                        axis_1_1.plot(time, pack_SOC, color = line_colors[res_i], marker = ps.markers[b_i], linewidth = ps.line_width, label = powertrain_labels[res_i])
                    else:
                        axis_1_1.plot(time, pack_SOC, color = line_colors[res_i], marker = ps.markers[b_i], linewidth = ps.line_width)
                    axis_1_1.set_ylabel(r'SOC')
                    axis_1_1.set_ylim([0,1.1])
                    set_axes(axis_1_1)     
                
                    axis_1_2.plot(time, pack_energy/1000/Units.Wh, color = line_colors[res_i], marker = ps.markers[b_i], linewidth = ps.line_width)
                    axis_1_2.set_ylabel(r'Energy (kW-hr)')
                    set_axes(axis_1_2) 
                
                    axis_1_3.plot(time, pack_current/1000, color = line_colors[res_i], marker = ps.markers[b_i], linewidth = ps.line_width)
                    axis_1_3.set_ylabel(r'Current (kA)')
                    set_axes(axis_1_3)  
                
                    axis_1_4.plot(time, pack_power/1000, color = line_colors[res_i], marker = ps.markers[b_i], linewidth = ps.line_width)
                    axis_1_4.set_ylabel(r'Power (kW)')
                    set_axes(axis_1_4)     
                
                    axis_1_5.plot(time, pack_volts/1000, color = line_colors[res_i], marker = ps.markers[b_i], linewidth = ps.line_width) 
                    axis_1_5.set_ylabel(r'Voltage (kV)')
                    set_axes(axis_1_5) 
                
                    axis_1_6.plot(time, pack_temperature, color = line_colors[res_i], marker = ps.markers[b_i], linewidth = ps.line_width)
                    axis_1_6.set_ylabel(r'Temperature, $\degree$C')
                    set_axes(axis_1_6)  
                 
        
            # ---------------------------------------------------------------------------
            # Fuel consumtion results, throttle and SFC         
            # ---------------------------------------------------------------------------  
            
            axis_2_1.set_ylabel(r'Throttle')
            set_axes(axis_2_1)               
            for network in results.segments[i].analyses.energy.vehicle.networks:   
                propulsor_tag = list(network.propulsors.keys())[0] 
                eta = results.segments[i].conditions.energy.propulsors[propulsor_tag].throttle[:,0] 
                if i ==0:
                    axis_2_1.plot(time, eta, color =  line_colors[res_i], marker = ps.markers[0], linewidth = ps.line_width, label = powertrain_labels[res_i])
                else:
                    axis_2_1.plot(time, eta, color =  line_colors[res_i], marker = ps.markers[0], linewidth = ps.line_width) 
            
            axis_2_2.plot(time, Weight/1000 , color =  line_colors[res_i], marker = ps.markers[0], linewidth = ps.line_width) 
            axis_2_2.set_ylabel(r'Weight (kN)')  
            set_axes(axis_2_2) 
    
            axis_2_3.plot(time, sfc, color =  line_colors[res_i], marker = ps.markers[0], linewidth = ps.line_width)
            axis_2_3.set_xlabel('Time (mins)')
            axis_2_3.set_ylabel(r'SFC (lb/lbf-hr)')
            set_axes(axis_2_3) 
    
            axis_2_4.plot(time, mdot, color =  line_colors[res_i], marker = ps.markers[0], linewidth = ps.line_width)
            axis_2_4.set_xlabel('Time (mins)')
            axis_2_4.set_ylabel(r'Fuel Rate (kg/s)')
            set_axes(axis_2_4)

            # ---------------------------------------------------------------------------
            # Plot Aerodynamic performance     
            # ---------------------------------------------------------------------------  

            axis_3_1 = plt.subplot(2,2,1)
            if i ==0:
                axis_3_1.plot(time, aoa, color =  line_colors[res_i], marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width, label = powertrain_labels[res_i])
            else:
                axis_3_1.plot(time, aoa, color =  line_colors[res_i], marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width)
            axis_3_1.set_ylabel(r'AoA (deg)') 
            axis_3_1.set_xlabel('Time (mins)')        
            axis_3_1.set_ylim([-5,15])
            set_axes(axis_3_1)    
    
            axis_3_2 = plt.subplot(2,2,2)        
            axis_3_2.plot(time, l_d, color =  line_colors[res_i], marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width) 
            axis_3_2.set_ylabel(r'L/D')
            axis_3_2.set_xlabel('Time (mins)')
            set_axes(axis_3_2) 
    
            axis_3_3 = plt.subplot(2,2,3) 
            axis_3_3.plot(time, cl, color =  line_colors[res_i], marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width)
            axis_3_3.set_xlabel('Time (mins)')
            axis_3_3.set_ylabel(r'$C_L$')
            set_axes(axis_3_3) 
    
            axis_3_4 = plt.subplot(2,2,4)        
            axis_3_4.plot(time, cd, color =  line_colors[res_i], marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width)
            axis_3_4.set_xlabel('Time (mins)')
            axis_3_4.set_ylabel(r'$C_D$')
            axis_3_4.set_ylim([0,0.1])
            set_axes(axis_3_4)
            
                        
                                      
    if show_legend:      
        leg_1 =  fig_1.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)  
        leg_2 =  fig_2.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)  
        leg_3 =  fig_3.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)  
    
    # Adjusting the sub-plots for legend 
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_1.subplots_adjust(top=0.8) 
    fig_2.subplots_adjust(top=0.8) 
    fig_3.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text   = 'Battery Pack Conditions'       
    fig_1.suptitle(title_text)        
    fig_2.suptitle(title_text)        
    fig_3.suptitle(title_text) 

    # Save figures if required
    if save_figure:
        fig_1.savefig(f"{save_filename}1{file_type}", dpi=300)
        fig_2.savefig(f"{save_filename}2{file_type}", dpi=300)
        fig_3.savefig(f"{save_filename}3{file_type}", dpi=300)
        
    return 


if __name__ == '__main__': 
    main()    
    plt.show()