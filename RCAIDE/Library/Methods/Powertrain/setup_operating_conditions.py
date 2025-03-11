# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/
# 
# Created:  Jan 2025, M. Clarke  


# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE   
from RCAIDE.Framework.Mission.Common import  Conditions, Results, Residuals
from RCAIDE.Library.Mission.Common.Update.orientations import orientations

# Python package imports
import numpy as np
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  Operating Test Conditions Set-up
# ---------------------------------------------------------------------------------------------------------------------- 
def setup_operating_conditions(compoment, altitude = 0,velocity_range = np.array([10]), angle_of_attack = 0):
    '''
    Set up operating conditions 
    
    '''
    
    ctrl_pts  = len(velocity_range)
        
    planet                                            = RCAIDE.Library.Attributes.Planets.Earth()
    working_fluid                                     = RCAIDE.Library.Attributes.Gases.Air()
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
    conditions._size = ctrl_pts
    conditions.expand_rows(ctrl_pts)
     
    conditions.freestream.velocity                    = np.atleast_2d(velocity_range) 
    conditions.frames.body.inertial_rotations[:, 1]   = angle_of_attack
    conditions.frames.inertial.velocity_vector[:, 0]  = np.atleast_2d(velocity_range)

    # setup conditions   
    segment                                          = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions                         = conditions    
    orientations(segment) 
    segment.state.residuals.network                  = Residuals()  
   
    if type(compoment) == RCAIDE.Library.Components.Powertrain.Converters.DC_Motor:                 
        propulsor         = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
        distributor       = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
        propulsor.motor   = compoment
        
        propulsor.append_operating_conditions(segment)   
        segment.state.conditions.energy[distributor.tag] = Conditions() 
        segment.state.conditions.noise[distributor.tag]  = Conditions()    
        propulsor.append_propulsor_unknowns_and_residuals(segment)
        
        
                
    elif type(compoment) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor: 
        propulsor         = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor() 
        distributor       = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
        propulsor.motor   = compoment
        
        propulsor.append_operating_conditions(segment)   
        segment.state.conditions.energy[distributor.tag] = Conditions() 
        segment.state.conditions.noise[distributor.tag]  = Conditions()    
        propulsor.append_propulsor_unknowns_and_residuals(segment)
        
                
    elif isinstance(compoment,RCAIDE.Library.Components.Powertrain.Converters.Rotor):  
        propulsor         = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
        distributor       = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
        propulsor.rotor   = compoment
        
        propulsor.append_operating_conditions(segment)   
        segment.state.conditions.energy[distributor.tag] = Conditions() 
        segment.state.conditions.noise[distributor.tag]  = Conditions()    
        propulsor.append_propulsor_unknowns_and_residuals(segment)
        
    elif isinstance(compoment,RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator):
        compoment.append_operating_conditions(segment)
        return segment.state , compoment.tag
                          
    elif isinstance(compoment,RCAIDE.Library.Components.Powertrain.Converters.Turboshaft): 
        compoment.append_operating_conditions(segment) 
        return segment.state , compoment.tag
                        
    elif isinstance(compoment,RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor): 
        propulsor = deepcopy(compoment)
        propulsor.working_fluid =  working_fluid
        
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
            
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    
            if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine:
                distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()            
        
        propulsor.append_operating_conditions(segment)   
        segment.state.conditions.energy[distributor.tag] = Conditions() 
        segment.state.conditions.noise[distributor.tag]  = Conditions()    
        propulsor.append_propulsor_unknowns_and_residuals(segment)

    segment.state.conditions.expand_rows(ctrl_pts)              
    return segment.state , propulsor 
 
    
    