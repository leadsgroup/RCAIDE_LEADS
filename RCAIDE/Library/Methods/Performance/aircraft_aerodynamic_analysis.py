# RCAIDE/Methods/Performance/aircraft_aerodynamic_analysis.py
# 
# 
# Created:  Dec 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import  Data 
 
# Pacakge imports 
import numpy as np  

#------------------------------------------------------------------------------
# aircraft_aerodynamic_analysis
#------------------------------------------------------------------------------  
def aircraft_aerodynamic_analysis(vehicle,
                                  angle_of_attack_range,
                                  Mach_number_range,
                                  control_surface_deflection_range = np.array([[0]]),
                                  altitude = 0,
                                  delta_ISA=0,
                                  use_surrogate = True,
                                  model_fuselage = True):  

    #------------------------------------------------------------------------
    # setup flight conditions
    #------------------------------------------------------------------------   
    atmosphere     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(altitude,delta_ISA)
    P              = atmo_data.pressure 
    T              = atmo_data.temperature 
    rho            = atmo_data.density 
    a              = atmo_data.speed_of_sound 
    mu             = atmo_data.dynamic_viscosity
       
    # -----------------------------------------------------------------
    # Evaluate Without Surrogate
    # ----------------------------------------------------------------- 
    ctrl_pts = len(angle_of_attack_range[:, 0] )
    state                                         = RCAIDE.Framework.Mission.Common.State()
    state.conditions                              = RCAIDE.Framework.Mission.Common.Results() 
    state.conditions.freestream.density           = rho * np.ones_like(angle_of_attack_range)
    state.conditions.freestream.dynamic_viscosity = mu  * np.ones_like(angle_of_attack_range)
    state.conditions.freestream.temperature       = T   * np.ones_like(angle_of_attack_range)
    state.conditions.freestream.pressure          = P   * np.ones_like(angle_of_attack_range)
    state.conditions.aerodynamics.angles.alpha    = angle_of_attack_range  
    state.conditions.aerodynamics.angles.beta     = angle_of_attack_range *0  
    state.conditions.freestream.u                 = angle_of_attack_range *0       
    state.conditions.freestream.v                 = angle_of_attack_range *0       
    state.conditions.freestream.w                 = angle_of_attack_range *0       
    state.conditions.static_stability.roll_rate   = angle_of_attack_range *0       
    state.conditions.static_stability.pitch_rate  = angle_of_attack_range *0 
    state.conditions.static_stability.yaw_rate    = angle_of_attack_range *0  
    state.conditions.expand_rows(ctrl_pts)
 
    CL_vals    = np.zeros((len(angle_of_attack_range),len(Mach_number_range)))  
    CD_vals    = np.zeros((len(angle_of_attack_range),len(Mach_number_range)))
    
 
    state.analyses                                  =  Data()
    aerodynamics                                    = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.use_surrogate             = use_surrogate 
    aerodynamics.vehicle                            = vehicle
    aerodynamics.settings.model_fuselage            = model_fuselage   
    aerodynamics.initialize()
    state.analyses.aerodynamics = aerodynamics 
    
    for i in range (len(Mach_number_range)):  
        state.conditions.freestream.mach_number                 = Mach_number_range[i, 0] * np.ones_like(angle_of_attack_range)
        state.conditions.freestream.velocity                    = Mach_number_range[i, 0] * a   * np.ones_like(angle_of_attack_range)   
        state.conditions.freestream.reynolds_number             = state.conditions.freestream.density * state.conditions.freestream.velocity / state.conditions.freestream.dynamic_viscosity 
        state.conditions.frames.inertial.velocity_vector[:,0]   = Mach_number_range[i, 0] * a[0, 0]   *  angle_of_attack_range[:, 0] 
        
     
        # ---------------------------------------------------------------------------------------
        # Evaluate With Surrogate
        # ---------------------------------------------------------------------------------------  
        _                 = state.analyses.aerodynamics.evaluate(state)        
        CL_vals[:,i]      = state.conditions.aerodynamics.coefficients.lift.total[:, 0]
        CD_vals[:,i]      = state.conditions.aerodynamics.coefficients.drag.total[:, 0] 

  
    results = Data(
        Mach              = Mach_number_range, 
        alpha             = angle_of_attack_range, 
        lift_coefficient  = CL_vals, 
        drag_coefficient  = CD_vals, 
    )  
         
    # FUTURE WORK          
    #control_surfaces_aerodynamics = Data()
    #num_defl =  len(control_surface_deflection_range)
    #for wing in vehicle.wings:
        #for control_surface in  wing.control_surfaces: 
            #CL_cs = np.zeros((num_defl,1))
            #CD_cs = np.zeros((num_defl,1))
            #for cs_i in  range(num_defl):
                #control_surface.deflection = control_surface_deflection_range[i]
                
                #_        = state.analyses.aerodynamics.evaluate(state)     
                #CL_cs[:,cs_i]    = state.conditions.aerodynamics.coefficients.lift.total
                #CD_cs[:,cs_i]    = state.conditions.aerodynamics.coefficients.drag.total 
                
            #control_surfaces_aerodynamics[control_surface.tag].lift_coefficient =  CL_cs
            #control_surfaces_aerodynamics[control_surface.tag].drag_coefficient =  CD_cs
            
            #control_surface.deflection =  0
    #results.control_surfaces_aerodynamics    = control_surfaces_aerodynamics
    #results.control_surface_deflection_range = control_surface_deflection_range 
    return results  
