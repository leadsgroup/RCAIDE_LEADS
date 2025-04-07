# latin_hypercube_sampling.py
#
# Created:  Jul 2016, R. Fenrich (outside of RCAIDE code)
# Modified: Apr 2017, T. MacDonald


# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import numpy as np

## If needed for mapping to normal distribution:
#from scipy.stats.distributions import norm 


# ----------------------------------------------------------------------
#   Latin Hypercube Sampling
# ----------------------------------------------------------------------
def latin_hypercube_sampling(num_dimensions,num_samples,bounds=None,criterion='random'):
    """Provides an array of chosen dimensionality and number of samples taken according
    to latin hypercube sampling. Bounds can be optionally specified.

    Assumptions:
    None

    Source:
    None

    Inputs:
    num_dimensions       [-]
    num_samples          [-]
    bounds (optional)    [-]      Default is 0 to 1. Input value should be in the form (with numpy arrays)
                                  (array([low_bnd_1,low_bnd_2,..]), array([up_bnd_1,up_bnd_2,..]))
    criterion            <string> Possible values: random and center. Determines if samples are 
                                  taken at the center of a bucket or randomly from within it.
                         
    Outputs:             
    lhd                  [-]      Array of samples

    Properties Used:
    N/A
    """       
    
    n = num_dimensions
    samples = num_samples
    
    segsize = 1./samples
    lhd = np.zeros((samples,n))
    
    if( criterion == "random" ): # sample is randomly chosen from within segment
        segment_starts = np.arange(samples)*segsize
        lhd_base       = np.transpose(np.tile(segment_starts,(n,1)))
        lhd            = lhd_base + np.random.rand(samples,n)*segsize
    elif( criterion == "center" ): # sample is chosen as center of segment
        segment_starts = np.arange(samples)*segsize
        lhd_base       = np.transpose(np.tile(segment_starts,(n,1)))
        lhd            = lhd_base + 0.5*segsize           
    else:
        raise NotImplementedError("Other sampling criterion not implemented")
        
    # Randomly switch values around to create Latin Hypercube
    for jj in range(n):
        np.random.shuffle(lhd[:,jj])
        
    ## Map samples to the standard normal distribution (if needed for future functionality)
    #lhd = norm(loc=0,scale=1).ppf(lhd)
    
    if bounds != None:
        lower_bounds = bounds[0]
        upper_bounds = bounds[1]
        lhd = lhd*(upper_bounds-lower_bounds) + lower_bounds

    return lhd