# RCAIDE/Library/Methods/Propulsor/Ducted_Fan_Propulsor/translate_conditions_to_dfdc_cases.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import  RCAIDE
from RCAIDE.Framework.Core import Data ,Units

# ---------------------------------------------------------------------------------------------------------------------- 
# Translate Conditions to DFDC Cases 
# ----------------------------------------------------------------------------------------------------------------------    
def translate_conditions_to_dfdc_cases(dfdc_analysis):
    """ Translate fligth conditions to DFDC Cases 

    Assumptions:
        N.A.

    Source:
        N.A.
    
    Inputs:
        dfdc_analysis (dict): DFDC analysis data structure  

    Outputs:
        None
    """
    # set up aerodynamic Conditions object
    template   = dfdc_analysis.settings.filenames.results_template 
    mach       = dfdc_analysis.training.mach     
    altitude   = dfdc_analysis.training.altitude        
    tip_mach   = dfdc_analysis.training.tip_mach
    ducted_fan = dfdc_analysis.geometry
    
    # first case is the design case 
    case            = Data() 
    atmosphere      = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data       = atmosphere.compute_values(ducted_fan.cruise.design_altitude)  
    rpm             = ducted_fan.cruise.design_angular_velocity/Units.rpm 
    case.tag        = template.format(ducted_fan.cruise.design_freestream_velocity,rpm,ducted_fan.cruise.design_altitude)   
    case.velocity   = ducted_fan.cruise.design_freestream_mach * atmo_data.speed_of_sound[0,0]
    case.RPM        = rpm
    case.altitude   = ducted_fan.cruise.design_altitude / 1000 # DFDC takes altitude in kilometers 
    dfdc_analysis.append_case(case)
    
    for i in range(len(mach)): 
        for j in range(len(tip_mach)):   
            for k in range(len(altitude)):     
                case            = Data() 
                atmosphere      = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
                atmo_data       = atmosphere.compute_values(altitude[k])  
                velocity        = mach[i] * atmo_data.speed_of_sound[0,0]
                tip_speed       = tip_mach[j]*atmo_data.speed_of_sound[0,0]
                omega           = tip_speed /ducted_fan.tip_radius
                rpm             = omega / Units.rpm
                case.tag        = template.format(velocity,rpm,altitude[k])  
                case.velocity   = velocity
                case.RPM        = rpm
                case.altitude   = altitude[k] / 1000
                dfdc_analysis.append_case(case) 
    return 