# RCAIDE/Library/Methods/Energy/Converters/Turboelectric_Generator/design_turboshaft.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

# RCAIDE Imports     
import RCAIDE 
from RCAIDE.Framework.Mission.Common                                 import Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft         import design_turboshaft
from RCAIDE.Library.Methods.Powertrain.Converters.Generator          import design_optimal_generator 

# Python package imports   
import numpy   as np

# ----------------------------------------------------------------------------------------------------------------------  
#  Design Turboshaft
# ----------------------------------------------------------------------------------------------------------------------   
def design_turboelectric_generator(turboelectric_generator):  
    """
    MATTEO HEADER


    """

    #check if mach number and temperature are passed
    #call the atmospheric model to get the conditions at the specified altitude
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(turboelectric_generator.turboshaft.design_altitude,turboelectric_generator.turboshaft.design_isa_deviation)
    planet     = RCAIDE.Library.Attributes.Planets.Earth()
    
    p   = atmo_data.pressure          
    T   = atmo_data.temperature       
    rho = atmo_data.density          
    a   = atmo_data.speed_of_sound    
    mu  = atmo_data.dynamic_viscosity   

    # setup conditions
    conditions = RCAIDE.Framework.Mission.Common.Results()

    # freestream conditions    
    conditions.freestream.altitude                    = np.atleast_1d(turboelectric_generator.turboshaft.design_altitude)
    conditions.freestream.mach_number                 = np.atleast_1d(turboelectric_generator.turboshaft.design_mach_number)
    conditions.freestream.pressure                    = np.atleast_1d(p)
    conditions.freestream.temperature                 = np.atleast_1d(T)
    conditions.freestream.density                     = np.atleast_1d(rho)
    conditions.freestream.dynamic_viscosity           = np.atleast_1d(mu)
    conditions.freestream.gravity                     = np.atleast_1d(planet.compute_gravity(turboelectric_generator.turboshaft.design_altitude))
    conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turboelectric_generator.turboshaft.working_fluid.compute_gamma(T,p))
    conditions.freestream.Cp                          = np.atleast_1d(turboelectric_generator.turboshaft.working_fluid.compute_cp(T,p))
    conditions.freestream.R                           = np.atleast_1d(turboelectric_generator.turboshaft.working_fluid.gas_specific_constant)
    conditions.freestream.speed_of_sound              = np.atleast_1d(a)
    conditions.freestream.velocity                    = np.atleast_1d(a*turboelectric_generator.turboshaft.design_mach_number)
         
         
    fuel_line                = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    segment                  = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state            = Conditions()  
    segment.state.conditions = conditions 
    segment.state.conditions.energy.converters[turboelectric_generator.tag] = Conditions() 
    
    turboelectric_generator.append_operating_conditions(segment)   

    design_turboshaft(turboelectric_generator,segment,fuel_line) 

    design_optimal_generator(turboelectric_generator.generator)    

    return      
  