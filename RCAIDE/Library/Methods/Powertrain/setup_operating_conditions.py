# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/
# 
# Created:  Jan 2025, M. Clarke  


# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE   
from RCAIDE.Framework.Mission.Common import Results, Residuals
from RCAIDE.Library.Mission.Common.Update.orientations import orientations

# Python package imports
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Operating Test Conditions Set-up
# ---------------------------------------------------------------------------------------------------------------------- 
def setup_operating_conditions(compoment, altitude = 0,velocity_range = np.array([10]), angle_of_attack = 0):
    '''
    Sets up operating conditions for single component analysis 
    '''
    
        
    planet                                            = RCAIDE.Library.Attributes.Planets.Earth()
    working_fluid                                     = RCAIDE.Library.Attributes.Gases.Air() 
    
    # append working fluid properties 
    compoment.working_fluid                           = working_fluid     
    
    atmosphere_sls                                    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                         = atmosphere_sls.compute_values(altitude,0.0) 
    p                                                 = atmo_data.pressure          
    T                                                 = atmo_data.temperature       
    rho                                               = atmo_data.density          
    a                                                 = atmo_data.speed_of_sound    
    mu                                                = atmo_data.dynamic_viscosity 
                                                      
    conditions                                        = Results() 
    conditions.freestream.altitude                    = np.atleast_2d(altitude)
    conditions.freestream.mach_number                 = np.atleast_2d(velocity_range/a)
    conditions.freestream.pressure                    = np.atleast_2d(p)
    conditions.freestream.temperature                 = np.atleast_2d(T)
    conditions.freestream.density                     = np.atleast_2d(rho)
    conditions.freestream.dynamic_viscosity           = np.atleast_2d(mu)
    conditions.freestream.gravity                     = np.atleast_2d(planet.sea_level_gravity)
    conditions.freestream.isentropic_expansion_factor = np.atleast_2d(working_fluid.compute_gamma(T,p))
    conditions.freestream.Cp                          = np.atleast_2d(working_fluid.compute_cp(T,p))
    conditions.freestream.R                           = np.atleast_2d(working_fluid.gas_specific_constant)
    conditions.freestream.speed_of_sound              = np.atleast_2d(a)

    num_ctrl_pts      = len(velocity_range)    
    conditions._size  = num_ctrl_pts
    conditions.expand_rows(num_ctrl_pts)
     
    conditions.freestream.velocity                    = np.atleast_2d(velocity_range) 
    conditions.frames.body.inertial_rotations[:, 1]   = angle_of_attack
    conditions.frames.inertial.velocity_vector[:, 0]  = np.atleast_2d(velocity_range)

    # setup conditions   
    segment                                          = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions                         = conditions    
    orientations(segment) 
    segment.state.residuals.network                  = Residuals()
    
    # append component-specific operating conditions 
    compoment.append_operating_conditions(segment,segment.state.conditions.energy,segment.state.conditions.noise)    
    segment.state.conditions.expand_rows(num_ctrl_pts)              
    return segment.state
 
    
    