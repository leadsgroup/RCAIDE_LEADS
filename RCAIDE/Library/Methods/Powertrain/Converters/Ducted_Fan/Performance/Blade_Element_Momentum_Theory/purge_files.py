# RCAIDE/Library/Methods/Propulsor/Ducted_Fan_Propulsor/purge_files.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------

import os
# ---------------------------------------------------------------------------------------------------------------------- 
# Purge Files  
# ----------------------------------------------------------------------------------------------------------------------   
def purge_files(filenames_array, directory=''):
    """
    Removes specified files from a directory to prevent conflicts.
    
    Parameters
    ----------
    filenames_array : list
        List of filenames to be removed from the specified directory.
    directory : str, optional
        Path to the directory containing the files to be removed.
        If not specified, the current directory is used.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function attempts to remove each file in the provided list from the
    specified directory. If a file does not exist, the function silently
    continues without raising an error.
    
    This utility is necessary for cleaning DFDC files. 
    
    References
    ----------
    [1] Drela, M. and Youngren, H., AVL, http://web.mit.edu/drela/Public/web/avl
    """    	
    for f in filenames_array:
        try:
            os.remove(os.path.abspath(os.path.join(directory,f)))
        except OSError:
            pass 