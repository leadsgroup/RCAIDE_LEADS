# Data.py
#
# Created:  Jun 2016, E. Botero
# Modified: Jan 2020, M. Clarke
#           May 2020, E. Botero
#           Jul 2021, E. Botero
#           Oct 2021, E. Botero



# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import numpy as np
from .Arrays import atleast_2d_col, array_type, matrix_type 

from copy import copy

# for enforcing attribute style access names
import string
from warnings import warn
chars = string.punctuation + string.whitespace
t_table = str.maketrans( chars          + string.ascii_uppercase , 
                            '_'*len(chars) + string.ascii_lowercase )

dictgetitem = dict.__getitem__
objgetattrib = object.__getattribute__

# ----------------------------------------------------------------------
#   Data
# ----------------------------------------------------------------------        
class Data(dict):
    """ An extension of the Python dict which allows for both tag and '.' usage.
        This is an unordered dictionary. So indexing it will not produce deterministic results.
        This has less overhead than ordering. If ordering is needed use DataOrdered().
       
        Assumptions:
        N/A
        
        Source:
        N/A
    """
    
    def __getattribute__(self, k):
        """ Retrieves an attribute set by a key k
    
            Assumptions:
            Does a try if it is a dict, but if that fails treats it as an object
    
            Source:
            N/A
    
            Inputs:
            k
    
            Outputs:
            whatever is found by k
    
            Properties Used:
            N/A
            """         
        try:
            return dictgetitem(self,k)
        except:
            return objgetattrib(self,k)

    def __setattr__(self, k, v):
        """ An override of the standard __setattr_ in Python.
            
            Assumptions:
            This one tries to treat k as an object, if that fails it treats it as a key.
    
            Source:
            N/A
    
            Inputs:
            k        [key]
            v        [value]
    
            Outputs:
            N/A
    
            Properties Used:
            N/A    
        """
        try:
            objgetattrib(self, k)
        except:
            self[k] = v
        else:          
            object.__setattr__(self, k, v) 
    
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
        pass      
    
    def __new__(cls,*args,**kwarg):
        """ Creates a new Data() class
    
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
        
        
        # initialize data, no inputs
        self = super(Data,cls).__new__(cls)
        super(Data,self).__init__() 
        
        # get base class list
        klasses = self.get_bases()
                
        # fill in defaults trunk to leaf
        for klass in klasses[::-1]:
            try:
                klass.__defaults__(self)
            except:
                pass
            
        return self
    
    def typestring(self):
        """ This function makes the .key.key structure in string form of Data()
    
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
        
        # build typestring
        typestring = str(type(self)).split("'")[1]
        typestring = typestring.split('.')
        if typestring[-1] == typestring[-2]:
            del typestring[-1]
        typestring = '.'.join(typestring) 
        return typestring    
    
    def dataname(self):
        """ This function is used for printing the class
    
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
        return "<data object '" + self.typestring() + "'>"     
    
    
    def __str__(self,indent=''):
        """ This function is used for printing the class. This starts the first line of printing.
    
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
        
        new_indent = '  '
        args = ''
        
        # trunk data name
        if not indent:
            args += self.dataname()  + '\n'
        else:
            args += ''
            
        args += self.__str2(indent)
        
        return args    
    
    
    def __str2(self,indent=''):
        """ This regresses through and does the rest of printing that __str__ missed
    
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
        
        new_indent = '  '
        args = ''
        
        # trunk data name
        if indent: args += '\n'
        
        # print values   
        for key,value in self.items():
            
            # skip 'hidden' items
            if isinstance(key,str) and key.startswith('_'):
                continue
            
            # recurse into other dict types
            if isinstance(value,dict):
                if not value:
                    val = '\n'
                else:
                    try:
                        val = value.__str2(indent+new_indent)
                    except RuntimeError: # recursion limit
                        val = ''
                    except:
                        val = value.__str__(indent+new_indent)
                                                
            # everything else
            else:
                val = str(value) + '\n'
                
            # this key-value, indented
            args+= indent + str(key) + ' : ' + val
            
        return args        
    
    def __init__(self,*args,**kwarg):
        """ :meta private:"""
        # """ 
        #     Initializes a new Data() class
    
        #     Assumptions:
        #     N/A
    
        #     Source:
        #     N/A
    
        #     Inputs:
        #     N/A
    
        #     Outputs:
        #     N/A
    
        #     Properties Used:
        #     N/A    
        # """           

        # handle input data (ala class factory)
        input_data = Data.__base__(*args,**kwarg)
        
        # update this data with inputs
        self.update(input_data)    

    def __iter__(self):
        """ :meta private:"""
        # """ Returns all the iterable values. Can be used in a for loop.
    
        #     Assumptions:
        #     N/A
    
        #     Source:
        #     N/A
    
        #     Inputs:
        #     N/A
    
        #     Outputs:
        #     N/A
    
        #     Properties Used:
        #     N/A    
        # """           
        return iter(self.values())
       
    
    def values(self):
        """ :meta private:"""
        # """ Returns all values inside the Data() class.
    
        #     Assumptions:
        #     N/A
    
        #     Source:
        #     N/A
    
        #     Inputs:
        #     N/A
    
        #     Outputs:
        #     values
    
        #     Properties Used:
        #     N/A    
        # """          
        return self.__values()          
            
    def __values(self):
        """ :meta private:"""
        #  Iterates over all keys to find all the data values.
    
        #     Assumptions:
        #     N/A
    
        #     Source:
        #     N/A
    
        #     Inputs:
        #     N/A
    
        #     Outputs:
        #     values
    
        #     Properties Used:
        #     N/A    
        #            
        return [self[key] for key in super(Data,self).__iter__()]    
    
    def update(self,other):
        """ :meta private:"""
        # """ Updates the internal values of a dictionary with given data
    
        #     Assumptions:
        #     N/A
    
        #     Source:
        #     N/A
    
        #     Inputs:
        #     other
    
        #     Outputs:
        #     N/A
    
        #     Properties Used:
        #     N/A    
        # """           
        if not isinstance(other,dict):
            raise TypeError('input is not a dictionary type')
        for k,v in other.items():
            # recurse only if self's value is a Dict()
            if k.startswith('_'):
                continue
        
            try:
                self[k].update(v)
            except:
                self[k] = v
        return 
    
    def get_bases(self):
        """ Finds the higher classes that may be built off of data
    
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            N/A
    
            Outputs:
            klasses
    
            Properties Used:
            N/A    
        """          
        # Get the Method Resolution Order, i.e. the ancestor tree
        klasses = list(self.__class__.__mro__)
        
        # Make sure that this is a Data object, otherwise throw an error.
        if Data not in klasses:
            raise TypeError('class %s is not of type Data()' % self.__class__)    
        
        # Remove the last two items, dict and object. Since the line before ensures this is a data objectt this won't break
        klasses = klasses[:-2]

        return klasses    
    
    def append(self,value,key=None):
        """ Adds new values to the classes. Can also change an already appended key
    
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            value
            key
    
            Outputs:
            N/A
    
            Properties Used:
            N/A    
        """          
        if key is None: key = value.tag
        key = key.translate(t_table)
        if key in self: raise KeyError('key "%s" already exists' % key)
        self[key] = value        
    
    def deep_set(self,keys,val):
        """ Regresses through a list of keys the same value in various places in a dictionary.
    
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            keys  - The keys to iterate over
            val   - The value to be set
    
            Outputs:
            N/A
    
            Properties Used:
            N/A    
        """               
        
        if isinstance(keys,str):
            keys = keys.split('.')
        
        data = self
         
        if len(keys) > 1:
            for k in keys[:-1]:
                data = data[k]
        
        if keys[-1][-1] ==']':
            splitkey = keys[-1].split('[')
            thing = data[splitkey[0]]
            for ii in range(1,len(splitkey)-1):
                index    = int(splitkey[ii][:-1])
                thing = thing[index]
            index    = int(splitkey[-1][:-1])
            thing[index] = val
        else:
            data[ keys[-1] ] = val
            
        return data

    def deep_get(self,keys):
        """ Regresses through a list of keys to pull a specific value out
    
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            keys  - The keys to iterate over
            
            Outputs:
            value - The value to be retrieved
    
            Properties Used:
            N/A    
        """          
        
        if isinstance(keys,str):
            keys = keys.split('.')
        
        data = self
         
        if len(keys) > 1:
            for k in keys[:-1]:
                data = data[k]
        
        value = data[ keys[-1] ]
        
        return value
        
    def do_recursive(self,func,other=None):
        """ Recursively combines matching keys of self and other (a second
            Data, or None) via func(self_value, other_value), returning a
            new object of self's own class with the same nested structure.

            Recurses wherever both self and other have a nested Data value at
            the same key (any Data subclass -- Conditions, plain Data, ...
            not just self's own class, since a Data structure like mission
            state.conditions commonly nests different Data subclasses at
            different levels); every other leaf (arrays, scalars, strings,
            ...) goes straight to func, which decides what's combinable (e.g.
            RCAIDE.Framework.Mission.Common.State.append_array vstacks
            matching rank-2 arrays and otherwise just keeps self's value).

            Assumptions:
            None

            Source:
            N/A

            Inputs:
            func    [callable]  func(self_value, other_value) -> combined_value
            other   [Data]      second structure to combine against, or None

            Outputs:
            Data (same class as self)

            Properties Used:
            None
        """
        result = self.__class__()
        for k,v in self.items():
            other_v = other[k] if isinstance(other,dict) and k in other else None
            if isinstance(v,Data) and isinstance(other_v,Data):
                result[k] = v.do_recursive(func,other_v)
            else:
                result[k] = func(v,other_v)
        return result

    def pack_array(self,output='vector'):
        """ maps the data dict to a 1D vector or 2D column array
        
            Assumptions:
                will only pack int, float, np.array and np.matrix (max rank 2)
                if using output = 'matrix', all data values must have 
                same length (if 1D) or number of rows (if 2D), otherwise is skipped
    
            Source:
            N/A
    
            Inputs:
                output - either 'vector' (default), or 'array'
                         chooses whether to output a 1d vector or 
                         2d column array
            
            Outputs:
                array - the packed array
    
            Properties Used:
            N/A  
        
        """
        
        
        # check output type
        if not output in ('vector','array'): raise Exception('output type must be "vector" or "array"')        
        vector = output == 'vector'
        
        # list to pre-dump array elements
        M = []
        
        # valid types for output
        valid_types = ( int, float,
                        array_type,
                        matrix_type )
        
        # initialize array row size (for array output)
        size = [False]
        
        # the packing function
        def do_pack(D):
            for v in D.values(): 
                try:
                    rank = v.ndim
                except:
                    rank = 0
                    
                # type checking
                if isinstance( v, dict ): 
                    do_pack(v) # recursion!
                    continue
                elif not isinstance( v, valid_types ): continue
                elif rank > 2: continue
                # make column vectors
                v = atleast_2d_col(v)
                # handle output type
                if vector:
                    # unravel into 1d vector
                    v = v.ravel(order='F')
                else:
                    # check array size
                    size[0] = size[0] or v.shape[0] # updates size once on first array
                    if v.shape[0] != size[0]: 
                        #warn ('array size mismatch, skipping. all values in data must have same number of rows for array packing',RuntimeWarning)
                        continue
                # dump to list
                M.append(v)
            #: for each value
        
        # do the packing
        do_pack(self)
        
        # pack into final array
        if M:
            M = np.hstack(M)
        else:
            # empty result
            if vector:
                M = np.array([])
            else:
                M = np.array([[]])
        
        # done!
        return M
    
    def unpack_array(self,M):
        """ unpacks an input 1d vector or 2d column array into the data dictionary
            important that the structure of the data dictionary, and the shapes
            of the contained values are the same as the data from which the array was packed
    
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            M - either a 1D vector or 2D column array
            
            Outputs:
            a reference to self, updates self in place
    
            Properties Used:
            N/A    
        """           

        
        # dont require dict to have numpy
        import numpy as np
        from .Arrays import atleast_2d_col, array_type, matrix_type
        
        # check input type
        vector = M.ndim  == 1
        
        # valid types for output
        valid_types = ( int, float,
                        array_type,
                        matrix_type )
        
        # counter for unpacking
        _index = [0]
        
        # the unpacking function
        def do_unpack(D):
            for k,v in D.items():
                try:
                    rank = v.ndim
                except:
                    rank = 0
                # type checking
                if isinstance(v, dict): 
                    do_unpack(v) # recursion!
                    continue
                elif not isinstance(v,valid_types): continue
                
                # get unpack index
                index = _index[0]                
                
                # skip if too big
                if rank > 2: 
                    continue
                
                # scalars
                elif rank == 0:
                    if vector:
                        D[k] = M[index]
                        index += 1
                    else:#array
                        continue
                        #raise RuntimeError , 'array size mismatch, all values in data must have same number of rows for array unpacking'
                    
                # 1d vectors
                elif rank == 1:
                    n = len(v)
                    if vector:
                        D[k][:] = M[index:(index+n)]
                        index += n
                    else:#array
                        D[k][:] = M[:,index]
                        index += 1
                    
                # 2d arrays
                elif rank == 2:
                    n,m = v.shape
                    if vector:
                        D[k][:,:] = np.reshape( M[index:(index+(n*m))] ,[n,m], order='F')
                        index += n*m 
                    else:#array
                        D[k][:,:] = M[:,index:(index+m)]
                        index += m
                
                #: switch rank
                
                _index[0] = index

            #: for each itme
        
        # do the unpack
        do_unpack(self)
         
        # check
        if not M.shape[-1] == _index[0]: warn('did not unpack all values',RuntimeWarning)
         
        # done!
        return self      