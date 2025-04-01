# RCAIDE/Library/Methods/Propulsor/Ducted_Fan_Propulsor/run_dfdc_analysis.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------

from RCAIDE.Framework.Core  import redirect 
import sys 
import subprocess
import os 
from .purge_files  import purge_files 

# ---------------------------------------------------------------------------------------------------------------------- 
# Run DFDC Analysis
# ----------------------------------------------------------------------------------------------------------------------   
def run_dfdc_analysis(dfdc_object, print_output):
    """
    Executes the DFDC (Ducted Fan Design Code) executable with the specified input deck.
    
    Parameters
    ----------
    dfdc_object : DFDCAnalysis
        Analysis object containing the following attributes:
            - settings : Data
                Configuration settings
                    - new_regression_results : bool
                        Flag to use existing results instead of running analysis
                    - filenames : Data
                        File path information
                            - log_filename : str
                                Path to log file for stdout
                            - err_filename : str
                                Path to error file for stderr
                            - dfdc_bin_name : str
                                Path to DFDC executable
                            - case : str
                                Case name for DFDC
            - current_status : Data
                Current analysis state
                    - deck_file : str
                        Path to input deck file with DFDC commands
    print_output : bool
        Flag to control whether DFDC console output is displayed
    
    Returns
    -------
    exit_status : int
        Return code from the DFDC process
        0 indicates successful execution
        Non-zero values indicate errors
    
    Notes
    -----
    This function handles the execution of the DFDC executable by:
        1. Purging any existing log and error files
        2. Opening the input deck file and feeding commands to DFDC
        3. Capturing and redirecting stdout/stderr to specified files
        4. Returning the process exit code
    
    This function requires the third party DFDC executable to be installed. More
    information on the DFDC executable from Drela and Youngrencan be found at: https://web.mit.edu/drela/Public/web/dfdc/ 
    
    If new_regression_results is True, the function will skip execution
    and return a success code (0) without running DFDC.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.purge_files
    RCAIDE.Framework.Core.redirect
    """      
    new_regression_results = dfdc_object.settings.new_regression_results
    if new_regression_results:
        exit_status = 0 
    else:
        log_file = dfdc_object.settings.filenames.log_filename
        err_file = dfdc_object.settings.filenames.err_filename
        if isinstance(log_file,str):
            purge_files(log_file)
        if isinstance(err_file,str):
            purge_files(err_file)
        dfdc_call = dfdc_object.settings.filenames.dfdc_bin_name
        case      = dfdc_object.settings.filenames.case
        in_deck   = dfdc_object.current_status.deck_file  
    
        with redirect.output(log_file,err_file): 
            with open(in_deck,'r') as commands: 
                
                # Initialize suppression of console window output
                if print_output == False:
                    devnull = open(os.devnull,'w')
                    sys.stdout = devnull       
                    
                # Run DFDC
                dfdc_run = subprocess.Popen([dfdc_call,case],stdout=sys.stdout,stderr=sys.stderr,stdin=subprocess.PIPE)
                for line in commands:
                    dfdc_run.stdin.write(line.encode('utf-8'))
                    dfdc_run.stdin.flush()
                    
                  
                # Terminate suppression of console window output  
                if print_output == False:
                    sys.stdout = sys.__stdout__                    
                    
            dfdc_run.wait()
    
            exit_status = dfdc_run.returncode 

    return exit_status

