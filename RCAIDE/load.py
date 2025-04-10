# load.py 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------      

import json
import pickle
from RCAIDE.Framework.Core import Data, DataOrdered
import numpy as np
from collections import OrderedDict

# ----------------------------------------------------------------------------------------------------------------------
#  load
# ----------------------------------------------------------------------------------------------------------------------    
def load(filename, pickle_format=False):
    """
    Imports a Pickle or JSON file into a RCAIDE data structure.
    
    Parameters
    ----------
    filename : str
        Path to the file to be loaded, without extension for pickle files
    pickle_format : bool, optional
        Flag indicating whether to load a pickle file (True) or JSON file (False)
        Default is False (JSON format)
        
    Returns
    -------
    data : RCAIDE.Framework.Core.Data
        RCAIDE data structure containing the loaded information
    
    Notes
    -----
    This function supports two file formats:
    
    1. JSON format (default): Loads a JSON file and converts it to a RCAIDE data structure
       using the read_RCAIDE_json_dict function.
    
    2. Pickle format: Loads a binary pickle file directly into a Python object.
       The .pkl extension is automatically added to the filename.
    
    JSON format is human-readable and more portable across different Python versions,
    while pickle format is more efficient for large data structures but less portable.
    
    See Also
    --------
    RCAIDE.save
    """
    
    if pickle_format:
        load_file = filename + '.pkl' 
        with open(load_file, 'rb') as file:
            data = pickle.load(file)  
    else: 
        # Get JSON string
        f = open(filename)
        res_string = f.readline()
        f.close()    
        
        # Convert to dictionary
        res_dict = json.loads(res_string,object_pairs_hook=OrderedDict)    
        
        # Convert to RCAIDE data structure
        data = read_RCAIDE_json_dict(res_dict) 
    
    return data 

def read_RCAIDE_json_dict(res_dict):
    """Builds a RCAIDE data structure based on a dictionary from a JSON file. This is initial case.

    Assumptions:
        Dictionary was created based on a previously saved RCAIDE data structure. 
        
    Source:
        None

    Args: 
        res_dict     : Dictionary based on the RCAIDE data structure [unitless] 
        
    Returns:
        RCAIDE_data  : RCAIDE data structure [unitless]   
    """      
    keys = res_dict.keys() # keys from top level
    RCAIDE_data = Data() # initialize RCAIDE data structure
    
    # Assign all values
    for k in keys:
        k = str(k)
        v = res_dict[k]
        RCAIDE_data[k] = build_data_r(v) # recursive function
    return RCAIDE_data
 
def build_data_r(v):
    """Builds a RCAIDE data structure based on a dictionary from a JSON file. This is recursive step.

    Assumptions:
        Dictionary was created based on a previously saved RCAIDE data structure. 

    Source:
        None

    Args: 
        v     : generic value [unitless]  
        
    Returns:
        ret  :  value converted to needed format [unitless]   
    """          
    tv = type(v) # Get value type
    
    # Transform to RCAIDE data structure with appropriate types
    if tv == OrderedDict:
        keys = v.keys()
        # Recursively assign values
        ret = DataOrdered()
        for k in keys:
            k = str(k)
            ret[k] = build_data_r(v[k])
    elif tv == list:
        ret = np.array(v)
    elif (tv == str): 
        ret = str(v)
    elif (tv == bool):
        ret = v
    elif tv == type(None):
        ret = None
    elif (tv == float) or (tv == int):
        ret = v        
    else:
        raise TypeError('Data type not expected in RCAIDE JSON structure')

    return ret