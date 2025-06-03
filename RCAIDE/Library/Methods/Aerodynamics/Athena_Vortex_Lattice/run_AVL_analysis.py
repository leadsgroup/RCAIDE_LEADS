# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/run_AVL_analysis.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core                                                        import redirect
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.read_results       import read_results
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.purge_files        import purge_files
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.write_geometry     import write_geometry
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.write_mass_file    import write_mass_file
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.write_run_cases    import write_run_cases
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.write_input_deck   import write_input_deck 
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.translate_data     import translate_conditions_to_cases, translate_results_to_conditions 
from RCAIDE.Library.Methods.Geometry.Planform.populate_control_sections           import populate_control_sections   
from RCAIDE.Library.Components.Wings.Control_Surfaces                             import Aileron , Elevator , Slat , Flap , Rudder 

# package imports 
import sys
import time
import subprocess
import os
from shutil import rmtree   

# ----------------------------------------------------------------------------------------------------------------------
# run_analysis
# ---------------------------------------------------------------------------------------------------------------------- 
def run_AVL_analysis(aerodynamics,run_conditions):
    """
    Executes complete AVL vortex lattice analysis including geometry setup, case generation, and result processing.

    Parameters
    ----------
    aerodynamics : Data
        AVL aerodynamics analysis object containing configuration and settings
            - vehicle : Vehicle
                Aircraft configuration with wings, fuselage, and control surfaces
            - settings : Data
                Analysis configuration parameters
                    - trim_aircraft : bool
                        Flag to enable aircraft trimming for equilibrium flight
                    - filenames : Data
                        File path templates and analysis directories
                    - print_output : bool
                        Flag to control console output during analysis
                    - keep_files : bool
                        Flag to retain AVL files after analysis completion
    run_conditions : Data
        Flight conditions for analysis
            - aerodynamics : Data
                Flight state data including angles, velocities, and atmospheric properties

    Returns
    -------
    None
        Results are stored directly in run_conditions

    Notes
    -----
    This function orchestrates the complete AVL analysis workflow including:
        1. Aircraft geometry translation to AVL format
        2. Mass properties file generation for trimmed flight
        3. Analysis case setup from flight conditions
        4. AVL input deck creation with run commands
        5. AVL solver execution and monitoring
        6. Result file parsing and data extraction
        7. Translation of results back to RCAIDE format

    Control surfaces are automatically detected and configured for analysis.
    The function handles both single-point analysis and parametric sweeps
    across multiple flight conditions.

    File management is automated with temporary directories created and
    optionally cleaned up based on keep_files setting. Analysis progress
    is tracked through batch indexing for multiple case execution.

    **Extra modules required**

    AVL (Athena Vortex Lattice) executable must be installed and accessible
    in the system PATH or specified in analysis settings.

    **Definitions**

    'Vortex Lattice Method'
        Numerical technique solving potential flow around discretized lifting surfaces

    'Trimmed Flight'
        Equilibrium flight condition with zero net forces and moments

    'Influence Coefficient'
        Mathematical relationship between vortex element and induced velocity

    References
    ----------
    [1] Drela, M. and Youngren, H., "AVL 3.36 User Primer", MIT, 2017
    """
    
    # unpack
    trim_aircraft                    = aerodynamics.settings.trim_aircraft
    run_folder                       = os.path.abspath(aerodynamics.settings.filenames.run_folder)
    run_script_path                  = run_folder.rstrip('avl_files').rstrip('/')   
    aero_results_template_1          = aerodynamics.settings.filenames.aero_output_template_1       # 'stability_axis_derivatives_{}.dat' 
    aero_results_template_2          = aerodynamics.settings.filenames.aero_output_template_2       # 'surface_forces_{}.dat'
    aero_results_template_3          = aerodynamics.settings.filenames.aero_output_template_3       # 'strip_forces_{}.dat'      
    aero_results_template_4          = aerodynamics.settings.filenames.aero_output_template_4       # 'body_axis_derivatives_{}.dat' 
    dynamic_results_template_1       = aerodynamics.settings.filenames.dynamic_output_template_1    # 'eigen_mode_{}.dat'
    dynamic_results_template_2       = aerodynamics.settings.filenames.dynamic_output_template_2    # 'system_matrix_{}.dat'
    batch_template                   = aerodynamics.settings.filenames.batch_template
    deck_template                    = aerodynamics.settings.filenames.deck_template 
    print_output                     = aerodynamics.settings.print_output 

    # rename defaul avl aircraft tag
    aerodynamics.tag                         = 'avl_analysis_of_{}'.format(aerodynamics.vehicle.tag) 
    aerodynamics.settings.filenames.features = aerodynamics.vehicle.tag + '.avl'
    aerodynamics.settings.filenames.mass_file= aerodynamics.vehicle.tag + '.mass'
    
    # update current status
    aerodynamics.current_status.batch_index += 1
    batch_index                      = aerodynamics.current_status.batch_index
    aerodynamics.current_status.batch_file   = batch_template.format(batch_index)
    aerodynamics.current_status.deck_file    = deck_template.format(batch_index)
           
    # control surfaces
    num_cs           = 0
    cs_names         = []
    cs_functions     = [] 
    control_surfaces = False
    
    for wing in aerodynamics.vehicle.wings: # this parses through the wings to determine how many control surfaces does the vehicle have 
        if wing.control_surfaces:
            control_surfaces = True 
            wing = populate_control_sections(wing)     
            num_cs_on_wing = len(wing.control_surfaces)
            num_cs +=  num_cs_on_wing
            for cs in wing.control_surfaces:
                ctrl_surf = cs    
                cs_names.append(ctrl_surf.tag)  
                if (type(ctrl_surf) ==  Slat):
                    ctrl_surf_function  = 'slat'
                elif (type(ctrl_surf) ==  Flap):
                    ctrl_surf_function  = 'flap' 
                elif (type(ctrl_surf) ==  Aileron):
                    ctrl_surf_function  = 'aileron'                          
                elif (type(ctrl_surf) ==  Elevator):
                    ctrl_surf_function  = 'elevator' 
                elif (type(ctrl_surf) ==  Rudder):
                    ctrl_surf_function = 'rudder'                      
                cs_functions.append(ctrl_surf_function)  

    # translate conditions
    cases = translate_conditions_to_cases(aerodynamics,run_conditions)    
    for case in cases:
        case.stability_and_control.number_of_control_surfaces = num_cs
        case.stability_and_control.control_surface_names      = cs_names
    aerodynamics.current_status.cases                         = cases  
     
    for case in cases:  
        case.aero_result_filename_1     = aero_results_template_1.format(case.tag)        # 'stability_axis_derivatives_{}.dat'  
        case.aero_result_filename_2     = aero_results_template_2.format(case.tag)        # 'surface_forces_{}.dat'
        case.aero_result_filename_3     = aero_results_template_3.format(case.tag)        # 'strip_forces_{}.dat'          
        case.aero_result_filename_4     = aero_results_template_4.format(case.tag)        # 'body_axis_derivatives_{}.dat'            
        case.eigen_result_filename_1    = dynamic_results_template_1.format(case.tag)     # 'eigen_mode_{}.dat'
        case.eigen_result_filename_2    = dynamic_results_template_2.format(case.tag)     # 'system_matrix_{}.dat'
    
    # write the input files
    with redirect.folder(run_folder,force=False):
        write_geometry(aerodynamics,run_script_path)
        write_mass_file(aerodynamics,run_conditions)
        write_run_cases(aerodynamics,trim_aircraft)
        write_input_deck(aerodynamics, trim_aircraft,control_surfaces)

        # RUN AVL! 
        exit_status = call_avl(aerodynamics,print_output)
        results_avl = read_results(aerodynamics)
        
    # translate results
    translate_results_to_conditions(cases,run_conditions,results_avl) 

    if not aerodynamics.settings.keep_files:
        rmtree( run_folder )
        
    return 
 
def call_avl(avl_object,print_output):
    """
    Executes AVL solver with specified input deck and monitors process completion.

    Parameters
    ----------
    avl_object : Data
        AVL analysis object containing file paths and execution settings
            - settings : Data
                Analysis configuration
                    - new_regression_results : bool
                        Flag to generate new analysis results
                    - filenames : Data
                        File path specifications
                            - avl_bin_name : str
                                AVL executable name or path
                            - features : str
                                Aircraft geometry file name
                            - log_filename : str
                                Analysis log file path
                            - err_filename : str
                                Error log file path
            - current_status.deck_file : str
                Input command deck file path
    print_output : bool
        Flag to control console output display during execution

    Returns
    -------
    exit_status : int
        AVL process exit code (0 for successful completion)

    Notes
    -----
    This function manages the AVL solver subprocess execution including:
        - Command deck file processing
        - Standard input/output redirection
        - Process monitoring and error handling
        - Exit status reporting

    The function supports both interactive and batch execution modes.
    Console output can be suppressed for automated analysis workflows
    while maintaining log file generation for debugging purposes.

    For regression testing mode, the function bypasses actual AVL execution
    and returns successful status to enable testing without AVL installation.

    **Major Assumptions**
        * Input geometry and command files are valid
        * System has sufficient resources for analysis completion
        * File permissions allow creation of output files

    **Definitions**

    'Exit Status'
        Numeric code returned by process indicating success (0) or error (non-zero)

    'Subprocess'
        Independent process launched from parent application
    """
    print_output =  True 
    new_regression_results = avl_object.settings.new_regression_results
    if new_regression_results:
        log_file = avl_object.settings.filenames.log_filename
        err_file = avl_object.settings.filenames.err_filename
        if isinstance(log_file,str):
            purge_files(log_file)
        if isinstance(err_file,str):
            purge_files(err_file)
        avl_call = avl_object.settings.filenames.avl_bin_name
        geometry = avl_object.settings.filenames.features
        in_deck  = avl_object.current_status.deck_file  
    
        with redirect.output(log_file,err_file):
    
            ctime = time.ctime() # Current date and time stamp
    
            with open(in_deck,'r') as commands: 
                
                # Initialize suppression of console window output
                if print_output == False:
                    devnull = open(os.devnull,'w')
                    sys.stdout = devnull       
                    
                # Run AVL
                avl_run = subprocess.Popen([avl_call,geometry],stdout=sys.stdout,stderr=sys.stderr,stdin=subprocess.PIPE)
                for line in commands:
                    avl_run.stdin.write(line.encode('utf-8'))
                    avl_run.stdin.flush()
                  
                # Terminate suppression of console window output  
                if print_output == False:
                    sys.stdout = sys.__stdout__                    
                    
            avl_run.wait()
    
            exit_status = avl_run.returncode
            ctime = time.ctime()
    else: 
        exit_status = 0        
    return exit_status

