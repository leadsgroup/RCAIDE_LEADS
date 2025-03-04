# RCAIDE/Library/Methods/Powertrain/Converters/Ducted_Fan.py
#  
# Created:  Jul 2023, M. Clarke
# Modified: Jan 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import  RCAIDE
from RCAIDE.Framework.Core import Data ,redirect  
from RCAIDE.Framework.Analyses.Propulsion.Ducted_Fan_Design_Code import Ducted_Fan_Design_Code
from RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.compute_ducted_fan_performance import compute_ducted_fan_efficiency
 
# python imports   
from shutil import rmtree
from .write_geometry                     import  write_geometry
from .write_input_deck                   import  write_input_deck
from .run_dfdc_analysis                  import  run_dfdc_analysis
from .translate_conditions_to_dfdc_cases import  translate_conditions_to_dfdc_cases
from .read_results                       import  read_results 
from scipy import interpolate 
import os
import numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  design_ducted_fan
# ---------------------------------------------------------------------------------------------------------------------- 
def design_ducted_fan(ducted_fan, dfdc_bin_name = 'dfdc', new_regression_results = False, keep_files = True): 
    """
    Designs and optimizes a ducted fan propulsor using either Rankine-Froude Momentum Theory or 
    Blade Element Momentum Theory (BEMT) with DFDC integration.

    Parameters
    ----------
    ducted_fan : Ducted_Fan
        Ducted fan component to be designed
    dfdc_bin_name : str, optional
        Name of DFDC executable, defaults to 'dfdc'
    new_regression_results : bool, optional
        Flag to generate new regression results, defaults to False
    keep_files : bool, optional
        Flag to keep DFDC input/output files, defaults to True

    Returns
    -------
    None
        Updates ducted_fan object with:
            - Blade geometry (twist, chord, radius distributions)
            - Performance characteristics at design point
            - Performance surrogate models (BEMT only)

    Notes
    -----
    Two fidelity levels are available:

    1. Rankine-Froude Momentum Theory:
        - Simple momentum theory calculations
        - Uses polynomial fits for performance coefficients
        - Quick preliminary design estimates

    2. Blade Element Momentum Theory (BEMT):
        - Detailed blade design using DFDC
        - Generates performance surrogate models
        - Accounts for 3D effects and losses
        - Requires external DFDC executable

    The BEMT method creates surrogate models for:
        - Thrust
        - Power
        - Efficiency
        - Torque
        - Thrust coefficient
        - Power coefficient

    **Major Assumptions**
        * Steady state operation
        * Incompressible flow for Rankine-Froude
        * No blade-wake interaction
        * Linear interpolation for surrogate models
        * US Standard Atmosphere 1976

    References
    ----------
    [1] Drela, M., "DFDC - Ducted Fan Design Code", MIT Aero & Astro, 2005
    [2] Esotec Developments, "DFDC Documentation", http://www.esotec.org/sw/DFDC.html

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.compute_ducted_fan_performance
    """
    

    if ducted_fan.fidelity == 'Rankine_Froude_Momentum_Theory':
        
        omega     = ducted_fan.cruise.design_angular_velocity
        V         = ducted_fan.cruise.design_freestream_velocity 
        atmo      = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        rho       = atmo.compute_values(ducted_fan.cruise.design_altitude,0.).density
        
        n, D, J, Cp, Ct, eta_p  = compute_ducted_fan_efficiency(ducted_fan, V, omega)

        T      = Ct * rho * (n**2)*(D**4) 
        P      = Cp * rho * (n**3)*(D**5)  
        Q      = P / omega
                
        ducted_fan.cruise.design_power              = P[0, 0]
        ducted_fan.cruise.design_efficiency         = eta_p 
        ducted_fan.cruise.design_thrust             = T[0, 0] 
        ducted_fan.cruise.design_torque             = Q[0, 0]
        ducted_fan.cruise.design_thrust_coefficient = Ct  
        ducted_fan.cruise.design_power_coefficient  = Cp 

    elif ducted_fan.fidelity == 'Blade_Element_Momentum_Theory': 
        if ducted_fan.cruise.design_altitude == None:
            raise AttributeError('design altitude not set')
        atmosphere      = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmo_data       = atmosphere.compute_values(ducted_fan.cruise.design_altitude)      
        
        if ducted_fan.cruise.design_freestream_mach == None: 
            ducted_fan.cruise.design_freestream_mach = ducted_fan.cruise.design_freestream_velocity / atmo_data.speed_of_sound[0,0]  
    
        if ducted_fan.cruise.design_freestream_velocity == None: 
            ducted_fan.cruise.design_freestream_velocity = ducted_fan.cruise.design_freestream_mach * atmo_data.speed_of_sound[0,0]       
        
        if (ducted_fan.cruise.design_angular_velocity)  == None:
            raise AttributeError('design tip mach and/or angular velocity not set')
            
        dfdc_analysis                                   = Ducted_Fan_Design_Code() 
        dfdc_analysis.geometry                          = ducted_fan
        dfdc_analysis.settings.filenames.dfdc_bin_name  = dfdc_bin_name
        dfdc_analysis.settings.new_regression_results   = new_regression_results
        dfdc_analysis.settings.keep_files               = keep_files
        
        # Get the root directory (one level above RCAIDE)
        current_dir = os.path.dirname(__file__)
        # Go up to the root directory containing RCAIDE
        root_dir = os.path.abspath(os.path.join(current_dir, *['..'] * 6))
    
        # Set the run folder path
        dfdc_analysis.settings.filenames.run_folder = os.path.join(root_dir, 'Regressions', 'Tests', 'network_electric', 'dfdc_files')
        run_folder = dfdc_analysis.settings.filenames.run_folder
    
        # Ensure the directory exists
        os.makedirs(run_folder, exist_ok=True)
        run_script_path                                 = run_folder.rstrip('dfdc_files').rstrip('/')    
        deck_template                                   = dfdc_analysis.settings.filenames.deck_template 
        print_output                                    = dfdc_analysis.settings.print_output  
        dfdc_analysis.current_status.deck_file          = deck_template.format(1) 
     
        # translate conditions  
        translate_conditions_to_dfdc_cases(dfdc_analysis)  
        dfdc_analysis.settings.filenames.case  =  dfdc_analysis.geometry.tag +  '.case'
        # write the input files
        with redirect.folder(run_folder,force=False):
            write_geometry(dfdc_analysis,run_script_path)    
            write_input_deck(dfdc_analysis)   
    
            # RUN DFDC!
            _ = run_dfdc_analysis(dfdc_analysis,print_output)
    
        # translate results
        results = read_results(dfdc_analysis)
    
        if not dfdc_analysis.settings.keep_files:
            rmtree(run_folder) 
             
        # save blade geometry 
        ducted_fan.rotor.twist_distribution           = results.geometry.rotor_twist_distribution            
        ducted_fan.rotor.chord_distribution           = results.geometry.rotor_chord_distribution            
        ducted_fan.rotor.radius_distribution          = results.geometry.rotor_radius_distribution           
        ducted_fan.rotor.non_dim_radius_distribution  = results.geometry.rotor_non_dim_radius_distribution   
        ducted_fan.rotor.solidity_distribution        = results.geometry.rotor_solidity_distribution         
        ducted_fan.stator.twist_distribution          = results.geometry.stator_twist_distribution           
        ducted_fan.stator.chord_distribution          = results.geometry.stator_chord_distribution           
        ducted_fan.stator.radius_distribution         = results.geometry.stator_radius_distribution          
        ducted_fan.stator.non_dim_radius_distribution = results.geometry.stator_non_dim_radius_distribution  
        ducted_fan.stator.solidity_distribution       = results.geometry.stator_solidity_distribution   
    
        mach              = dfdc_analysis.training.mach       
        tip_mach          = dfdc_analysis.training.tip_mach         
        altitude          = dfdc_analysis.training.altitude/1000
    
        # create performance surrogates 
        raw_data           = results.performance
        thrust             = clean_data(raw_data.thrust,mach,tip_mach,altitude,raw_data.converged_solution)               
        power              = clean_data(raw_data.power,mach,tip_mach,altitude,raw_data.converged_solution)                
        efficiency         = clean_data(raw_data.efficiency,mach,tip_mach,altitude,raw_data.converged_solution)           
        torque             = clean_data(raw_data.torque,mach,tip_mach,altitude,raw_data.converged_solution)               
        thrust_coefficient = clean_data(raw_data.thrust_coefficient,mach,tip_mach,altitude,raw_data.converged_solution)   
        power_coefficient  = clean_data(raw_data.power_coefficient,mach,tip_mach,altitude,raw_data.converged_solution)     
        
        surrogates =  Data()
        surrogates.thrust              = interpolate.RegularGridInterpolator((mach,tip_mach,altitude),thrust               ,method = 'linear',   bounds_error=False, fill_value=None)      
        surrogates.power               = interpolate.RegularGridInterpolator((mach,tip_mach,altitude),power                ,method = 'linear',   bounds_error=False, fill_value=None)
        surrogates.efficiency          = interpolate.RegularGridInterpolator((mach,tip_mach,altitude),efficiency           ,method = 'linear',   bounds_error=False, fill_value=None)      
        surrogates.torque              = interpolate.RegularGridInterpolator((mach,tip_mach,altitude),torque               ,method = 'linear',   bounds_error=False, fill_value=None)
        surrogates.thrust_coefficient  = interpolate.RegularGridInterpolator((mach,tip_mach,altitude),thrust_coefficient   ,method = 'linear',   bounds_error=False, fill_value=None)      
        surrogates.power_coefficient   = interpolate.RegularGridInterpolator((mach,tip_mach,altitude),power_coefficient    ,method = 'linear',   bounds_error=False, fill_value=None) 
        
        ducted_fan.performance_surrogates =  surrogates 
        
        ducted_fan.cruise.design_thrust             = results.performance.design_thrust            
        ducted_fan.cruise.design_power              = results.performance.design_power             
        ducted_fan.cruise.design_efficiency         = results.performance.design_efficiency        
        ducted_fan.cruise.design_torque             = results.performance.design_power /  ducted_fan.cruise.design_angular_velocity   
        ducted_fan.cruise.design_thrust_coefficient = results.performance.design_thrust_coefficient
        ducted_fan.cruise.design_power_coefficient  = results.performance.design_power_coefficient  
        
    return

def clean_data(raw_data,mach,tip_mach,altitude,convergence_matrix):  
    cleaned_data =  np.zeros((len(mach),len(tip_mach),len(altitude))) 
    for i in range(len(mach)): 
        x = tip_mach
        y = altitude
        
        # mask invalid values
        array = np.ma.masked_invalid(raw_data[i])
        if np.all(array.mask ==False):
            cleaned_data[i, :, :] =  array.data
        else: 
            yy,xx   = np.meshgrid(y, x)
            x1      = xx[~array.mask]
            y1      = yy[~array.mask]
            newarr  = array[~array.mask] 
            points  = np.vstack((x1, y1)).T
            values1 = newarr.data
            cleaned_data[i, :, :]  = interpolate.griddata(points, values1, (xx, yy), method='cubic',  fill_value = -1E5)
             
    return cleaned_data

