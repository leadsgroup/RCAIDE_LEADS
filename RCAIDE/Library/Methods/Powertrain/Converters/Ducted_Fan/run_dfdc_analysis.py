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
def run_dfdc_analysis(dfdc_object,print_output):
    """ This calls the DFDC executable and runs an analysis

    Assumptions:
        None
        
    Source:
        None

    Inputs:
        dfdc_object - passed into the  call_dfdc function  
        
    Outputs:
        results

    Properties Used:
        N/A
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

