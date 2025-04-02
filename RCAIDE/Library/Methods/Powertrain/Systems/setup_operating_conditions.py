# RCAIDE/Library/Methods/Powertrain/Systems/setup_operating_conditions.py
# 
# Created: M Clarke.

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
def setup_operating_conditions(component, altitude=0, velocity_vector=np.array([[10, 0, 0]])):
    """
    Sets up operating conditions for powertrain component testing.
    
    Parameters
    ----------
    component : Component
        The powertrain component to be tested. Can be one of:
            - Converter (DC_Motor, PMSM_Motor, Rotor)
            - Propulsor (Turbofan, Turbojet, Turboprop, Turboshaft, ICE_Propeller)
    altitude : float, optional
        Altitude for the test conditions [m]. Default is 0 (sea level).
    velocity_vector : numpy.ndarray, optional
        Velocity vector in body-frame coordinates [m/s].
        Default is [[10, 0, 0]] (10 m/s in the x-direction).
    
    Returns
    -------
    state : State
        State object containing atmospheric and flight conditions
    propulsor_tag : str
        Tag identifier for the propulsor component
    
    Notes
    -----
    This function creates a standardized test environment for powertrain components
    by setting up atmospheric conditions, freestream properties, and component-specific
    operating conditions.
    
    The function performs the following steps:
        1. Identifies the component type and creates appropriate propulsor and distributor
        2. Sets up Earth atmospheric conditions at the specified altitude
        3. Calculates freestream properties (pressure, temperature, density, etc.)
        4. Creates a segment with the calculated conditions
        5. Appends component-specific operating conditions
        6. Sets up residuals for network calculations
    
    The function handles both converter components (which are wrapped in appropriate
    propulsors) and propulsor components directly.
    
    **Major Assumptions**
        * US Standard Atmosphere 1976 model is used
        * Earth gravity model is used
        * Air is the working fluid
    
    See Also
    --------
    RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976
    RCAIDE.Library.Mission.Common.Update.orientations
    """
    
    working_fluid = RCAIDE.Library.Attributes.Gases.Air()
    
    if isinstance(component, RCAIDE.Library.Components.Powertrain.Converters.Converter):
        # assign generatic propulsor 
        if type(component) == RCAIDE.Library.Components.Powertrain.Converters.DC_Motor:  
            propulsor         = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
            distributor       = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
            propulsor.motor   = component  
        if type(component) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor: 
            propulsor         = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor() 
            distributor       = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
            propulsor.motor   = component   
        if isinstance(component,RCAIDE.Library.Components.Powertrain.Converters.Rotor):  
            propulsor         = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
            distributor       = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
            propulsor.rotor   = component   
            
    elif isinstance(component,RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor): 
        propulsor = deepcopy(component)
        propulsor.working_fluid =  working_fluid
        
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turboshaft:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    
        if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.ICE_Propeller:
            distributor = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
            
        
    planet                                            = RCAIDE.Library.Attributes.Planets.Earth()
    working_fluid = RCAIDE.Library.Attributes.Gases.Air()
    atmosphere_sls                                    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                         = atmosphere_sls.compute_values(0.0,0.0)
                                                      
    p                                                 = atmo_data.pressure          
    T                                                 = atmo_data.temperature       
    rho                                               = atmo_data.density          
    a                                                 = atmo_data.speed_of_sound    
    mu                                                = atmo_data.dynamic_viscosity
     
    speed                                             =  np.linalg.norm(velocity_vector, axis =1) 
                                                      
    conditions                                        = Results() 
    conditions.freestream.altitude                    = np.atleast_2d(altitude)
    conditions.freestream.mach_number                 = np.atleast_2d(speed/a)
    conditions.freestream.pressure                    = np.atleast_2d(p)
    conditions.freestream.temperature                 = np.atleast_2d(T)
    conditions.freestream.density                     = np.atleast_2d(rho)
    conditions.freestream.dynamic_viscosity           = np.atleast_2d(mu)
    conditions.freestream.gravity                     = np.atleast_2d(planet.sea_level_gravity)
    conditions.freestream.isentropic_expansion_factor = np.atleast_2d(working_fluid.compute_gamma(T,p))
    conditions.freestream.Cp                          = np.atleast_2d(working_fluid.compute_cp(T,p))
    conditions.freestream.R                           = np.atleast_2d(working_fluid.gas_specific_constant)
    conditions.freestream.speed_of_sound              = np.atleast_2d(a)
    conditions.freestream.velocity                    = velocity_vector 
    
    AoA =  np.arctan(velocity_vector[0, 2] / velocity_vector[0, 0])

    conditions.frames.body.inertial_rotations        =  np.array([[0, AoA, 0]]) 
    conditions.static_stability.roll_rate            =  np.array([[0]])  
    conditions.static_stability.pitch_rate           =  np.array([[0]])
    conditions.static_stability.yaw_rate             =  np.array([[0]]) 

    # setup conditions   
    segment                                           = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions                          = conditions    
    orientations(segment)
    
    segment.state.residuals.network = Residuals()
    propulsor.append_operating_conditions(segment) 
    for tag, item in  propulsor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,propulsor) 
      
    segment.state.conditions.energy[distributor.tag] = Conditions() 
    segment.state.conditions.noise[distributor.tag]  = Conditions()    
    propulsor.append_propulsor_unknowns_and_residuals(segment)
         
    return segment.state , propulsor.tag