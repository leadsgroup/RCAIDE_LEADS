# RCAIDE/Library/Methods/Propulsor/Ducted_Fan_Propulsor/read_results.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data, Units  
import numpy as np
import  os 
# ---------------------------------------------------------------------------------------------------------------------- 
# Read Results 
# ----------------------------------------------------------------------------------------------------------------------  
def read_results(dfdc_analysis):
    """ This functions reads the results from the results text file created 
    at the end of an DFDC function call

    Assumptions:
        None
        
    Source: 
        None
        
    Inputs:
        None

    Outputs:
        results     

    Properties Used:
        N/A
    """     
    ducted_fan        = dfdc_analysis.geometry
    Nr                = ducted_fan.number_of_radial_stations
    results_template  = dfdc_analysis.settings.filenames.results_template
    run_folder        = dfdc_analysis.settings.filenames.run_folder
    mach              = dfdc_analysis.training.mach   
    tip_machs         = dfdc_analysis.training.tip_mach  
    altitudes         = dfdc_analysis.training.altitude
    len_m             = len(mach)
    len_tm            = len(tip_machs) 
    len_a             = len(altitudes)
    
    results                                             = Data()
    results.geometry                                    = Data() 
    results.geometry.rotor_twist_distribution           = np.zeros(Nr)
    results.geometry.rotor_chord_distribution           = np.zeros(Nr)
    results.geometry.rotor_radius_distribution          = np.zeros(Nr) 
    results.geometry.rotor_non_dim_radius_distribution  = np.zeros(Nr) 
    results.geometry.rotor_solidity_distribution        = np.zeros(Nr)
    results.geometry.stator_twist_distribution          = np.zeros(Nr)
    results.geometry.stator_chord_distribution          = np.zeros(Nr)
    results.geometry.stator_radius_distribution         = np.zeros(Nr) 
    results.geometry.stator_non_dim_radius_distribution = np.zeros(Nr) 
    results.geometry.stator_solidity_distribution       = np.zeros(Nr)  
    results.performance                                 = Data() 
    results.performance.thrust                          = np.zeros((len_m,len_tm,len_a))  
    results.performance.power                           = np.zeros((len_m,len_tm,len_a))  
    results.performance.efficiency                      = np.zeros((len_m,len_tm,len_a)) 
    results.performance.torque                          = np.zeros((len_m,len_tm,len_a)) 
    results.performance.thrust_coefficient              = np.zeros((len_m,len_tm,len_a))  
    results.performance.power_coefficient               = np.zeros((len_m,len_tm,len_a)) 
    results.performance.advance_ratio                   = np.zeros((len_m,len_tm,len_a))
    results.performance.figure_of_merit                 = np.zeros((len_m,len_tm,len_a))  
    results.performance.converged_solution              = np.zeros((len_m,len_tm,len_a))    
   
    # Read geometry 
    geometry_filename = os.path.join(run_folder, 'ducted_fan_geometry.txt')
    with open(geometry_filename,'r') as geometry_file: 
            geometry_lines                       = geometry_file.readlines() 
            results.geometry.hub_radius          = float(geometry_lines[27][12:23].strip())
            results.geometry.tip_radius          = float(geometry_lines[27][35:44].strip())  
            for n_r in range(Nr):
                results.geometry.rotor_radius_distribution[n_r]         = float(geometry_lines[36 + Nr + n_r][0:12].strip())
                results.geometry.rotor_non_dim_radius_distribution[n_r] = float(geometry_lines[36 + Nr + n_r][12:22].strip())
                results.geometry.rotor_chord_distribution[n_r]          = float(geometry_lines[36 + Nr + n_r][22:35].strip())
                results.geometry.rotor_twist_distribution[n_r]          = float(geometry_lines[36 + Nr + n_r][35:44].strip()) * Units.degrees
                results.geometry.rotor_solidity_distribution[n_r]       = float(geometry_lines[36 + Nr + n_r][44:56].strip())
    
    # Read desing point data
 
    atmosphere       = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data        = atmosphere.compute_values(ducted_fan.cruise.design_altitude) 
    a                = atmo_data.speed_of_sound[0,0] 
    design_RPM       = ducted_fan.cruise.design_angular_velocity/Units.rpm 
    design_velocity  = ducted_fan.cruise.design_freestream_velocity 
    design_altitude  = ducted_fan.cruise.design_altitude  
    string           = results_template.format(design_velocity,design_RPM,design_altitude)     
    results_filename = os.path.join(run_folder, string + '.txt')
    with open(results_filename,'r') as case_results_file: 
        case_lines                       = case_results_file.readlines() 
        results.performance.design_thrust              = float(case_lines[8][13:26].strip())
        results.performance.design_power               = float(case_lines[8][39:52].strip())
        results.performance.design_efficiency          = float(case_lines[8][65:76].strip()) 
        results.performance.design_torque              = float(case_lines[21][13:26].strip())        
        results.performance.design_thrust_coefficient  = float(case_lines[13][7:20].strip())        
        results.performance.design_power_coefficient   = float(case_lines[13][27:39].strip())       
        
    # Read evaluation point data 
    for i in range(len_m): 
        for j in range(len_tm):
            for k in range(len_a):
                try: 
                    atmosphere      = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
                    atmo_data       = atmosphere.compute_values(altitudes[k]) 
                    a               = atmo_data.speed_of_sound[0,0] 
                    omega           = ((tip_machs[j]*a) /dfdc_analysis.geometry.tip_radius)
                    rpm             = omega / Units.rpm
                    velocity        =  mach[i] * a 
                    string          = results_template.format(velocity,rpm,altitudes[k])   
                    results_filename   = os.path.join(run_folder, string + '.txt')
                    with open(results_filename,'r') as case_results_file: 
                        case_lines                       = case_results_file.readlines() 
                        results.performance.thrust[i,j,k]              = float(case_lines[8][13:26].strip())
                        results.performance.power[i,j,k]               = float(case_lines[8][39:52].strip())
                        results.performance.efficiency[i,j,k]          = float(case_lines[8][65:76].strip()) 
                        results.performance.torque[i,j,k]              = float(case_lines[10][39:52].strip())        
                        results.performance.thrust_coefficient[i,j,k]  = float(case_lines[13][7:20].strip())        
                        results.performance.power_coefficient[i,j,k]   = float(case_lines[13][27:39].strip())   
                        results.performance.advance_ratio[i,j,k]       = float(case_lines[13][45:57].strip())
    
                    results.performance.converged_solution[i,j,k]  =  True                        
                except:
                    results.performance.converged_solution[i,j,k]  = False
                    results.performance.thrust[i,j,k]              = np.nan
                    results.performance.power[i,j,k]               = np.nan
                    results.performance.efficiency[i,j,k]          = np.nan
                    results.performance.torque[i,j,k]              = np.nan       
                    results.performance.thrust_coefficient[i,j,k]  = np.nan      
                    results.performance.power_coefficient[i,j,k]   = np.nan  
                    results.performance.advance_ratio[i,j,k]       = np.nan

    return results