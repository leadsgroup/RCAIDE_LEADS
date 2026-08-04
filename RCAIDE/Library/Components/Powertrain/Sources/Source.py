# RCAIDE/Library/Components/Powertrain/Sources/Source
#  
# 
# Created:  Oct. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from   RCAIDE.Framework.Core               import Data
from   RCAIDE.Library.Components           import Component 
from   RCAIDE.Framework.Mission.Common     import Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  Source
# ---------------------------------------------------------------------------------------------------------------------- 
class Source(Component):
    """
    ``domain`` is the energy domain this source provides (``'chemical'`` or
    ``'electrical'``), self-declared by each concrete subclass so topology
    analysis (``RCAIDE.Library.Mission.Common.Pre_Process.energy.
    analyze_topology``) can classify sources without a hardcoded isinstance
    list.
    """
    def __defaults__(self):
        """ This sets the default values.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            None
        """          
        self.tag                          = 'source'
        self.active                       = True
        self.assigned_distributors        = None
        self.efficiency                   = 0.0
        self.power_split_ratio            = 1.0
        # See Propulsor.identical_propulsors -- same reuse-eligibility concept,
        # applied to sources (e.g. multiple Battery_Pack instances) in
        # Network.evaluate()/unpack_unknowns()/residuals(). Defaults False,
        # unlike propulsors/converters: sources commonly carry independent
        # per-instance state (state of charge, degradation, temperature
        # history) that a stateless propulsor/converter calculation doesn't,
        # so reuse isn't safe to assume. Not to be confused with
        # Battery_Pack.identical_modules, a separate flag for reuse *within*
        # a single pack's own modules, not between sibling sources.
        self.identical_sources            = False
        self.domain                       = None