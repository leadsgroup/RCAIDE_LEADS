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
        distributor_conditions = state.conditions.energy.distributors[self.tag]

        inputs  = Data()
        outputs = Data()
        inputs.power  = Data()
        outputs.power = Data()

        inputs.power.mechanical  = distributor_conditions.inputs.power.mechanical
        inputs.power.electrical  = distributor_conditions.inputs.power.electrical
        inputs.power.chemical    = distributor_conditions.inputs.power.chemical
        inputs.power.hydraulic   = distributor_conditions.inputs.power.hydraulic
        inputs.power.thermal     = distributor_conditions.inputs.power.thermal

        outputs.power.mechanical = distributor_conditions.outputs.power.mechanical
        outputs.power.electrical = distributor_conditions.outputs.power.electrical
        outputs.power.chemical   = distributor_conditions.outputs.power.chemical
        outputs.power.hydraulic  = distributor_conditions.outputs.power.hydraulic
        outputs.power.thermal    = distributor_conditions.outputs.power.thermal

        return inputs, outputs

    def unpack_unknowns(self,segment):
        return

    def pack_residuals(self,segment):
        return

    def append_unknowns_and_residuals(self,segment):
        return