# RCAIDE/Library/Components/Powertrain/Distributors/Distributor.py 
# 
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Components                      import Component   

# ---------------------------------------------------------------------------------------------------------------------- 
#  Distributor
# ----------------------------------------------------------------------------------------------------------------------
class Distributor(Component):
    """
    Base class for components that route power between sources/converters and
    the propulsors or systems that consume it (e.g. an electrical bus or fuel line).

    Attributes
    ----------
    tag : str
        Identifier for the distributor (default: 'tag')
    type : str
        Component type identifier (default: 'distributor')
    working_fluid : None or Attribute
        Fluid carried by the distributor, if any (default: None)
    active : bool
        Flag indicating if the distributor is operational (default: True)
    assigned_distributors : None or list
        Distributors this one feeds into, if any (default: None)
    efficiency : float
        Power transfer efficiency (default: 0.0)

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus
    RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line
    """

    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            None 
        """
        # set the deafult values
        self.tag                      = 'tag' 
        self.type                     = 'distributor'
        self.working_fluid            = None
        self.active                   = True
        self.assigned_distributors    = None  
        self.efficiency               = 0.0
        
    def initialize(self,network):
        return

    def compute_performance(self,state,network):
        """Default distributor contribution to the network power balance.

        A generic distributor (e.g. an electrical bus) has no power of its
        own beyond what its assigned components already report through
        compute_distribution_losses() during the propulsor/system/converter/
        source loops -- that power is already counted once there. Returning
        zero here avoids double-counting it a second time when Network.py
        folds this return into net_*_power. Subclasses with genuine
        additional physics not captured elsewhere (e.g. Fuel_Line's no-pump
        fallback) override this with real, non-double-counted values.
        """
        ones_row = state.ones_row

        inputs  = Data()
        outputs = Data()
        inputs.power  = Data()
        outputs.power = Data()

        inputs.power.mechanical  = 0. * ones_row(1)
        inputs.power.electrical  = 0. * ones_row(1)
        inputs.power.chemical    = 0. * ones_row(1)
        inputs.power.hydraulic   = 0. * ones_row(1)
        inputs.power.thermal     = 0. * ones_row(1)

        outputs.power.mechanical = 0. * ones_row(1)
        outputs.power.electrical = 0. * ones_row(1)
        outputs.power.chemical   = 0. * ones_row(1)
        outputs.power.hydraulic  = 0. * ones_row(1)
        outputs.power.thermal    = 0. * ones_row(1)

        return inputs, outputs

    def unpack_unknowns(self,segment):
        return

    def pack_residuals(self,segment):
        return

    def append_unknowns_and_residuals(self,segment):
        return