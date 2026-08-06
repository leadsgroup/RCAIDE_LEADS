# RCAIDE/Library/Components/Propulsors/Propulsor.py
#  
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Components           import Component  
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_cylinder_moment_of_inertia 

import numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  Propusor
# ---------------------------------------------------------------------------------------------------------------------- 
class Propulsor(Component):
    """
    Base class for all propulsion system components in RCAIDE.
    
    Attributes
    ----------
    tag : str
        Identifier for the propulsor, defaults to 'propulsor'
        
    active : bool
        Flag indicating if the propulsor is operational, defaults to True
        
    wing_mounted : bool
        Flag indicating if the propulsor is mounted on a wing, defaults to True

    domain : str
        Primary propulsive energy domain this propulsor draws from
        (``'chemical'`` or ``'electrical'``), self-declared by each concrete
        subclass so topology analysis (``RCAIDE.Library.Mission.Common.
        Pre_Process.energy.analyze_topology``) can classify propulsors
        without a hardcoded isinstance list.

    Notes
    -----
    This class serves as the foundation for all propulsion system implementations 
    in RCAIDE. It provides the basic structure and common attributes needed for:
        * Electric propulsion systems (rotors, ducted fans)
        * Internal combustion engine systems (fixed and constant-speed propellers)
        * Gas turbine systems (turbofans, turbojets, turboprops)
    
    The class inherits from Component, providing basic component functionality
    while adding propulsion-specific features. Derived classes must implement
    their own performance calculation methods and system-specific attributes.
    
    **Definitions**

    'Propulsor'
        Any device that generates thrust or lift force through the manipulation 
        of fluid momentum (air in most cases)
    
    'Wing-mounted'
        Configuration where the propulsor is attached to the aircraft's wing 
        rather than the fuselage or other structures
    
    See Also
    --------
    RCAIDE.Library.Components.Component
    RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor
    RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Ducted_Fan
    RCAIDE.Library.Components.Powertrain.Propulsors.ICE_Propeller
    RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_ICE_Propeller
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
        self.tag                          = 'propulsor'
        self.active                       = True
        self.wing_mounted                 = True
        self.domain                       = None
        self.nacelle                      = None 
        self.identical_propulsors         = True
        self.reverse_thrust               = False
        self.sealevel_static_thrust       = 0.0
        self.working_fluid                = RCAIDE.Library.Attributes.Gases.Air()
        self.assigned_converters          = None 
        self.assigned_modulators          = None 
        self.assigned_distributors        = None
        self.nacelle                      = None 
        self.efficiency                   = 1.0
        self.sealevel_static_thrust       = 0.0  
        self.diameter                     = 0.0      
        self.length                       = 0.0
        self.height                       = 0.0    
        self.working_fluid                = RCAIDE.Library.Attributes.Gases.Air()

    def initialize(self, network):
        return

    def append_segment_conditions(self, segment):
        propulsor_conditions = segment.state.conditions.energy.propulsors[self.tag]
        propulsor_conditions.inputs.power.propulsive[:,0]  = 0.0
        propulsor_conditions.inputs.power.mechanical[:,0]  = 0.0
        propulsor_conditions.inputs.power.electrical[:,0]  = 0.0
        propulsor_conditions.inputs.power.chemical[:,0]    = 0.0
        propulsor_conditions.inputs.power.pneumatic[:,0]   = 0.0
        propulsor_conditions.inputs.power.hydraulic[:,0]   = 0.0
        propulsor_conditions.inputs.power.thermal[:,0]     = 0.0
        propulsor_conditions.outputs.power.propulsive[:,0] = 0.0
        propulsor_conditions.outputs.power.mechanical[:,0] = 0.0
        propulsor_conditions.outputs.power.electrical[:,0] = 0.0
        propulsor_conditions.outputs.power.chemical[:,0]   = 0.0
        propulsor_conditions.outputs.power.pneumatic[:,0]  = 0.0
        propulsor_conditions.outputs.power.hydraulic[:,0]  = 0.0
        propulsor_conditions.outputs.power.thermal[:,0]    = 0.0

    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the propulsor.

        Parameters
        ---------- 
        center_of_gravity : list, optional
            Reference point coordinates, defaults to [[0, 0, 0]]
        
        Returns
        -------
        ndarray
            3x3 moment of inertia tensor
        """

        _, _ =  compute_cylinder_moment_of_inertia(self, self.length, self.diameter/2, 0, 0, center_of_gravity=center_of_gravity)  
        return