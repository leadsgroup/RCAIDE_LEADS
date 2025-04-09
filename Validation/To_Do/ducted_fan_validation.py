# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units ,Data  
from RCAIDE.Library.Plots  import * 
from RCAIDE.Library.Methods.Propulsors.Converters.Ducted_Fan.design_ducted_fan import design_ducted_fan
from RCAIDE.Library.Methods.Propulsors.Converters.Ducted_Fan.compute_ducted_fan_performance import compute_ducted_fan_performance
from RCAIDE.Framework.Mission.Common    import  Conditions , Results 
from RCAIDE.Library.Methods.Propulsors.Converters.DC_Motor          import design_motor 
from RCAIDE.Framework.Mission.Segments.Segment   import Segment  

# python imports  
import matplotlib.pyplot as plt                                      
import matplotlib.cm     as cm
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.axes3d import get_test_data
from scipy import interpolate 
import pickle    
import os
import pandas as pd 
import numpy as  np
import os                                                   
import time  
 
def main(): 
    
    ti                  = time.time()      

    tip_mach            = np.array([0.2, 0.3, 0.4, 0.5, 0.6 ])     
    mach_number         = np.array([0.01,0.1, 0.2, 0.3 , 0.4, 0.5]) 
    altitude            = np.array([0, 5000, 10000, 15000,  20000]) *Units.feet

    thrust              = np.zeros((len(altitude),len(mach_number),len(tip_mach)))
    torque              = np.zeros((len(altitude),len(mach_number),len(tip_mach)))
    power               = np.zeros((len(altitude),len(mach_number),len(tip_mach)))

    bus                                                = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.voltage                                        = 1000                         
    electric_ducted_fan                                = RCAIDE.Library.Components.Propulsors.Electric_Ducted_Fan()
    DF =  define_ducted_fan()
    electric_ducted_fan.ducted_fan                     = DF
    electric_ducted_fan.motor                          = define_ducted_fan_motor(bus,electric_ducted_fan.ducted_fan)
    electric_ducted_fan.electronic_speed_controller    = define_electronic_speed_controller()
    bus.propulsors.append(electric_ducted_fan)    
    
    for i in range(len(altitude)): 
        for j in range(len(mach_number)):
            for k in  range(len(tip_mach)):
                
                atmosphere                                        = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
                atmo_data                                         = atmosphere.compute_values(altitude[i])
                                                                   
                ctrl_pts =  1
                AoA =  0
                true_course      = 0
                fligth_path_angle =  0
                 
                
                segment                                                = RCAIDE.Framework.Mission.Segments.Segment() 
                
                conditions                                             = Results()  
                conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1)) * AoA
                conditions.freestream.density                          = atmo_data.density  
                conditions.freestream.dynamic_viscosity                = atmo_data.dynamic_viscosity
                conditions.freestream.speed_of_sound                   = atmo_data.speed_of_sound
                conditions.freestream.temperature                      = atmo_data.temperature
                conditions.freestream.altitude                         = np.ones((ctrl_pts,1)) *altitude[i]
                conditions.frames.inertial.velocity_vector             = np.array([[mach_number[j] *atmo_data.speed_of_sound[0,0] , 0. ,0.]]) 
                conditions.frames.planet.true_course                   = np.zeros((ctrl_pts,3,3)) 
                conditions.frames.planet.true_course[:,0,0]            = np.cos(true_course),
                conditions.frames.planet.true_course[:,0,1]            = - np.sin(true_course)
                conditions.frames.planet.true_course[:,1,0]            = np.sin(true_course)
                conditions.frames.planet.true_course[:,1,1]            = np.cos(true_course) 
                conditions.frames.planet.true_course[:,2,2]            = 1 
                conditions.frames.wind.transform_to_inertial           = np.zeros((ctrl_pts,3,3))   
                conditions.frames.wind.transform_to_inertial[:,0,0]    = np.cos(fligth_path_angle) 
                conditions.frames.wind.transform_to_inertial[:,0,2]    = np.sin(fligth_path_angle) 
                conditions.frames.wind.transform_to_inertial[:,1,1]    = 1 
                conditions.frames.wind.transform_to_inertial[:,2,0]    = -np.sin(fligth_path_angle) 
                conditions.frames.wind.transform_to_inertial[:,2,2]    = np.cos(fligth_path_angle)  
                conditions.frames.body.transform_to_inertial           = np.zeros((ctrl_pts,3,3))
                conditions.frames.body.transform_to_inertial[:,0,0]    = np.cos(AoA)
                conditions.frames.body.transform_to_inertial[:,0,2]    = np.sin(AoA)
                conditions.frames.body.transform_to_inertial[:,1,1]    = 1
                conditions.frames.body.transform_to_inertial[:,2,0]    = -np.sin(AoA)
                conditions.frames.body.transform_to_inertial[:,2,2]    = np.cos(AoA)     
                segment.state.conditions                               = conditions  
                segment.state.conditions.energy[bus.tag]               = Conditions() 
                
                electric_ducted_fan.append_operating_conditions(segment,bus)     
                for tag, item in  electric_ducted_fan.items(): 
                    if issubclass(type(item), RCAIDE.Library.Components.Component):
                        item.append_operating_conditions(segment,bus,electric_ducted_fan) 
                
                # set throttle
                segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag].throttle[:,0] = 1   
            
                # Run BEMT
                segment.state.conditions.expand_rows(ctrl_pts)
                ducted_fan_conditions             = segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag][electric_ducted_fan.ducted_fan.tag]     
                ducted_fan_conditions.omega[:,0]  = ((tip_mach[k]*atmo_data.speed_of_sound[0,0]) /DF.tip_radius) 
                compute_ducted_fan_performance(electric_ducted_fan,segment.state,bus)
                
                thrust[i, j, k] = segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag][electric_ducted_fan.ducted_fan.tag].thrust[0, 0] 
                torque[i, j, k] = segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag][electric_ducted_fan.ducted_fan.tag].torque[0, 0]
                power[i, j, k]  = segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag][electric_ducted_fan.ducted_fan.tag].power[0, 0]
        
    plot_results(altitude,mach_number,tip_mach,thrust,torque,power)
    
    tf                      = time.time()                                           # [s]       Define the final simulation time
    elapsed_time            = round((tf-ti),2)                                      # [s]       Compute the total simulation time

    print('Simulation Time: ' + str(elapsed_time) + ' seconds per timestep')        # [-]       Print the value of total simulation time    
    
    return