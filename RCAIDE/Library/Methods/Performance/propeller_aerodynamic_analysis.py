#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                              import Units 
from RCAIDE.Framework.Mission.Common                    import Results  
from RCAIDE.Framework.Mission.Segments.Segment          import Segment
 
import numpy as np

# ------------------------------------------------------------------------------
#   Propeller Analysis
# ------------------------------------------------------------------------------ 
def propeller_aerodynamic_analysis(propeller,
                           velocity_range,
                           angular_velocity = 2500*Units.rpm,
                           angle_of_attack = 0, 
                           altitude = 0,
                           delta_isa =0 ): 
    # design aircract 
    electric_rotor                 = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    electric_rotor.rotor           = propeller 
    
    # operarting states  
    ctrl_pts          =  len(velocity_range) 
    
    # Find the operating conditions
    atmosphere                                             = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere_conditions                                  = atmosphere.compute_values(propeller.cruise.design_altitude)  
 
    segment                                                = Segment()  
    conditions                                             = Results()  
    conditions.aerodynamics.angle_of_attack                = np.array([[angle_of_attack]])
    conditions.freestream.density                          = atmosphere_conditions.density 
    conditions.freestream.dynamic_viscosity                = atmosphere_conditions.dynamic_viscosity 
    conditions.freestream.speed_of_sound                   = atmosphere_conditions.speed_of_sound 
    conditions.freestream.temperature                      = atmosphere_conditions.temperature 
    conditions.frames.planet.true_course                   = np.zeros((ctrl_pts,3,3)) 
    conditions.frames.planet.true_course[:,0,0]            = np.cos(0.0),
    conditions.frames.planet.true_course[:,0,1]            = - np.sin(0.0)
    conditions.frames.planet.true_course[:,1,0]            = np.sin(0.0)
    conditions.frames.planet.true_course[:,1,1]            = np.cos(0.0) 
    conditions.frames.planet.true_course[:,2,2]            = 1 
    conditions.frames.wind.transform_to_inertial           = np.zeros((ctrl_pts,3,3))   
    conditions.frames.wind.transform_to_inertial[:,0,0]    = np.cos(0.0) 
    conditions.frames.wind.transform_to_inertial[:,0,2]    = np.sin(0.0) 
    conditions.frames.wind.transform_to_inertial[:,1,1]    = 1 
    conditions.frames.wind.transform_to_inertial[:,2,0]    = -np.sin(0.0) 
    conditions.frames.wind.transform_to_inertial[:,2,2]    = np.cos(0.0)  
    conditions.frames.body.transform_to_inertial           = np.zeros((ctrl_pts,3,3))
    conditions.frames.body.transform_to_inertial[:,0,0]    = np.cos(angle_of_attack)
    conditions.frames.body.transform_to_inertial[:,0,2]    = np.sin(angle_of_attack)
    conditions.frames.body.transform_to_inertial[:,1,1]    = 1
    conditions.frames.body.transform_to_inertial[:,2,0]    = -np.sin(angle_of_attack)
    conditions.frames.body.transform_to_inertial[:,2,2]    = np.cos(angle_of_attack)  
    segment.state.conditions                               = conditions 
 

    electric_rotor.append_operating_conditions(segment) 
    for tag, item in  electric_rotor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,electric_rotor)
            
    # Run BEMT
    segment.state.conditions.expand_rows(ctrl_pts)

    conditions.frames.inertial.velocity_vector             = velocity_range
    conditions.freestream.mach_number                      = conditions.frames.inertial.velocity_vector / conditions.freestream.speed_of_sound
    
    rotor_conditions             =  segment.state.conditions.energy[electric_rotor.tag][propeller.tag]        
    rotor_conditions.omega       = angular_velocity
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance(electric_rotor,segment.state)
     
    results =  segment.state.conditions.energy[electric_rotor.tag][propeller.tag] 
    return  results