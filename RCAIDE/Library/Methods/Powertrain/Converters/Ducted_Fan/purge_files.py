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
def purge_files(filenames_array,directory=''):
    """ Purges folder folder of conflicting files

    Assumptions:
    None

    Source:
        Drela, M. and Youngren, H., AVL, http://web.mit.edu/drela/Public/web/avl

    Inputs:
        None

    Outputs:
    None

    Properties Used:
        N/A
    """    	
    for f in filenames_array:
        try:
            os.remove(os.path.abspath(os.path.join(directory,f)))
        except OSError:
            pass 