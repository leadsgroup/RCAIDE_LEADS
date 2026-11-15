# DiffedData.py
#
# Created:  Feb 2015, T. Lukacyzk
# Modified: Feb 2016, T. MacDonald
#           Jun 2016, E. Botero

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from copy import deepcopy
from .Container import Container as ContainerBase
from .Data import Data
from .DataOrdered import DataOrdered
import numpy as np

# ----------------------------------------------------------------------
#  Config
# ----------------------------------------------------------------------

class Diffed_Data(Data):
    """ This is for creating a data new class where a different copy is saved.
        This is useful for creating a new configuration of a vehicle.

        Assumptions:
        N/A

        Source:
        N/A
    """    

    
    def __defaults__(self):
        """ A stub for all classes that come later
            
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            N/A
    
            Outputs:
            N/A
    
            Properties Used:
            N/A    
        """  
        self.tag    = 'config'
        self._base  = Data()
        self._diff  = None
        
    def __init__(self,base=None):
        """ Initializes the new Diffed_Data() class through a deepcopy
    
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            N/A
    
            Outputs:
            N/A
    
            Properties Used:
            N/A    
        """  
        if base is None: base = Data()
        this = deepcopy(base) # deepcopy is needed here to build configs - Feb 2016, T. MacDonald
        Data.__init__(self,this)
        self._base = base
        self._diff = None
        
    def store_diff(self):
        """ Saves the values in which this configuration differs from the vehicle it was made from.

            Outputs:
            self._diff.changed    {path: value of the configuration}
            self._diff.original   {path: value of the vehicle}
            self._diff.added      [path] of the entries the vehicle does not have
        """
        self._diff = configuration_values(self, base_vehicle(self))

# ----------------------------------------------------------------------
#  Config Container
# ----------------------------------------------------------------------

class Container(ContainerBase):
    """ A dict-type container with attribute, item and index style access
        intended to hold a attribute-accessible list of Data(). This is unordered.
        
        Assumptions:
        N/A
        
        Source:
        N/A
        
    """
    def append(self,value):
        """ Appends the value to the containers
        
            Assumptions:
            None
        
            Source:
            N/A
        
            Inputs:
            self
        
            Outputs:
            N/A
            
            Properties Used:
            N/A
        """         
        if isinstance(value, Diffed_Data):
            value.store_diff()
        ContainerBase.append(self,value)

    def store_diff(self):
        """ Finds the differences and saves them
    
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            N/A
    
            Outputs:
            N/A
    
            Properties Used:
            N/A    
        """          
        for config in self:
            if isinstance(config, Diffed_Data):
                config.store_diff()


# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------

Diffed_Data.Container = Container

# ------------------------------------------------------------
#  Configuration values
# ------------------------------------------------------------

def base_vehicle(configuration):
    """The vehicle a configuration was made from, following configurations made from configurations."""
    vehicle = configuration
    while isinstance(vehicle, Diffed_Data):
        vehicle = vehicle._base
    return vehicle


def configuration_values(configuration, vehicle):
    """ The values in which a configuration differs from its vehicle.

        Assumptions:
        A configuration changes values the vehicle already has. It does not add or remove components.

        Inputs:
        configuration    Diffed_Data made from the vehicle
        vehicle          the vehicle the configuration was made from

        Outputs:
        values.changed   {path: value of the configuration}, a path being the tuple of keys from the vehicle
        values.original  {path: value of the vehicle}
        values.added     [path] of the entries the vehicle does not have, or has as another kind of value
    """
    values          = Data()
    values.changed  = {}
    values.original = {}
    values.added    = []
    compare_values(configuration, vehicle, (), values)
    return values


def compare_values(item, base_item, path, values):
    """Walks a configuration and its vehicle together and records the leaf values that differ."""
    for key in item.keys():
        # internal entries, and the tag at the top that names the configuration
        if str(key).startswith('_') or (key == 'tag' and len(path) == 0):
            continue
        key_path = path + (key,)
        if key not in base_item.keys():
            values.added.append(key_path)
            continue
        value      = item[key]
        base_value = base_item[key]
        if isinstance(value, (Data, DataOrdered)) and isinstance(base_value, (Data, DataOrdered)):
            compare_values(value, base_value, key_path, values)
        elif isinstance(value, (Data, DataOrdered)) or isinstance(base_value, (Data, DataOrdered)):
            values.added.append(key_path)
        elif is_plain_value(value) and not values_equal(value, base_value):
            values.changed[key_path]  = value
            values.original[key_path] = base_value
    return


def is_plain_value(value):
    """True for numbers, flags, strings, arrays and lists of them: what a configuration can change."""
    if value is None or isinstance(value, (bool, int, float, str, np.ndarray, np.generic)):
        return True
    if isinstance(value, (list, tuple)):
        return all(is_plain_value(item) for item in value)
    return False


def values_equal(va, vb):
    """True if two leaf values are equal, including lists or tuples of arrays and arrays of different shapes."""
    if isinstance(va, (list, tuple)) and isinstance(vb, (list, tuple)):
        return len(va) == len(vb) and all(values_equal(a, b) for a, b in zip(va, vb))
    try:
        return bool(np.all(va == vb))
    except ValueError:
        return False
