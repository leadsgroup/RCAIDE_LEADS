# RCAIDE/Framework/Analyses/Energy/Energy.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core     import Data
from RCAIDE.Framework.Analyses import Analysis 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  ANALYSIS
# ----------------------------------------------------------------------------------------------------------------------   

# ----------------------------------------------------------------------------------------------------------------------
#  Energy Analysis
# ----------------------------------------------------------------------------------------------------------------------   
class Energy(Analysis):
    """ This is the base class for energy analyses.
    """
    def __defaults__(self):
        """This sets the default values and methods for the analysis.
            
        Assumptions:
            None

        Source:
            None 
        """        
        self.tag      = 'energy'
        self.vehicle  = Data()
        
    def evaluate(self,unknowns,segment,network):
        """Evaluate the thrust produced by the energy network.
    
        Assumptions:
            None

        Source:
            None

        Args:
            state (dict): flight conditions [-]

        Returns:
            results : results of the thrust evaluation method. 
        """  
        vehicle = segment.analyses.vehicle
        state   = segment.state

        # Pack the unknowns to pass through the network
        if isinstance(unknowns,np.ndarray):
            state.unknowns.network.unpack_array(unknowns)

        network.evaluate(state, vehicle)
        
        if type(segment) != RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge: 
            state.residuals.network[ 'electrical_power'] = state.conditions.energy.net_electrical_power 
            #state.residuals.network[ 'chemical_power']   = state.conditions.energy.net_chemical_power
         

        # Unpack Residuals
        residual_keys = list(state.residuals.network.keys())
        residual_keys.remove('tag')
        network_res = Data()
        full_ures_vals = Data()
        for res in residual_keys:
            network_res[res] = state.residuals.network[res]
            full_ures_vals[res] = network_res[res]

        return  full_ures_vals.pack_array()
    