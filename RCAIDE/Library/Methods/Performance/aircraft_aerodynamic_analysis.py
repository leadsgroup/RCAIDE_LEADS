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
from RCAIDE.Library.Mission.Common.Update.orientations import orientations
 
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
                                  use_surrogate  = False,
                                  model_fuselage = True):  
    
    
    if  len(angle_of_attack_range[:, 0] ) != len(Mach_number_range[:, 0]) :
        assert print('Mach and AoA range must be the same dimension')
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
    state.conditions.aerodynamics.angles.beta     = np.zeros_like(angle_of_attack_range)  
    state.conditions.freestream.u                 = np.zeros_like(angle_of_attack_range)       
    state.conditions.freestream.v                 = np.zeros_like(angle_of_attack_range)       
    state.conditions.freestream.w                 = np.zeros_like(angle_of_attack_range)      
    state.conditions.static_stability.roll_rate   = np.zeros_like(angle_of_attack_range)
    state.conditions.static_stability.pitch_rate  = np.zeros_like(angle_of_attack_range)
    state.conditions.static_stability.yaw_rate    = np.zeros_like(angle_of_attack_range)
    state.conditions.expand_rows(ctrl_pts)
  
    state.analyses                                     = Data()
    aerodynamics                                       = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.use_surrogate                = use_surrogate
    aerodynamics.vehicle                               = vehicle
    aerodynamics.settings.model_fuselage               = model_fuselage 
    aerodynamics.settings.number_of_spanwise_vortices  = 50
    aerodynamics.settings.number_of_chordwise_vortices = 10
    aerodynamics.initialize()
    state.analyses.aerodynamics = aerodynamics 
      
    state.conditions.freestream.mach_number                 = Mach_number_range
    state.conditions.freestream.velocity                    = Mach_number_range  * a   
    state.conditions.freestream.reynolds_number             = state.conditions.freestream.density * state.conditions.freestream.velocity / state.conditions.freestream.dynamic_viscosity 
    state.conditions.frames.inertial.velocity_vector[:,0]   = Mach_number_range[:,0] * a[:,0]    
    state.conditions.frames.body.inertial_rotations[:,1]    = angle_of_attack_range[:,0]
    

    # setup conditions   
    segment        = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state  = state
    orientations(segment) 
 
    # ---------------------------------------------------------------------------------------
    # Evaluate With Surrogate
    # ---------------------------------------------------------------------------------------  
    _                 = state.analyses.aerodynamics.evaluate(state)    
          
    return  state.conditions.aerodynamics
