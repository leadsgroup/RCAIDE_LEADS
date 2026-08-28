# RCAIDE/Library/Components/Powertrain/Distributors/Electrical_Bus.py 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Jan 2025, M. Clarke 
#           Sep 2025, M. Guidotti
#           Apr 2026, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Distributors.Fuel_Line.compute_fuel_line_distribution_losses import compute_fuel_line_distribution_losses  
from .Distributor                                                  import Distributor 
from RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus import * 
from RCAIDE.Library.Attributes.Materials                           import Copper, Polyimide 
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_distributor_moment_of_inertia import *
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_distributor_center_of_gravity import * 
from RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus.size_electrical_cable import size_electrical_cable

# ----------------------------------------------------------------------------------------------------------------------
#  Electrical_Bus
# ---------------------------------------------------------------------------------------------------------------------- 
class Electrical_Bus(Distributor):
    """
    Class for managing power distribution between aircraft electrical components

    Attributes
    ----------
    tag : str
        Identifier for the electrical bus (default: 'electrical_line')

    domain : str
        Power domain this distributor carries (default: 'electrical')

    active : bool
        Flag indicating if the bus is operational (default: True)

    design_power : float
        Design power the bus is sized to carry [W] (default: 0.0), derived
        from its assigned sources/converters/propulsors during `initialize()`

    design_voltage : float
        Design voltage of the bus [V] (default: 0.0), set from an assigned
        battery pack's or fuel cell's voltage during `initialize()`

    frequency : float
        AC frequency, if applicable [Hz] (default: 0.0)

    current_type : str
        Type of current carried, e.g. 'DC' or 'AC' (default: 'DC')

    efficiency : float
        Power distribution efficiency (default: 1)

    length : float
        Cable length [m] (default: 1)

    number_of_parallel_wires : int or None
        Number of parallel conductors (default: None). When None,
        `size_electrical_cable` auto-selects the smallest count that keeps
        each conductor under a practical single-conductor size (~500 kcmil,
        NEC 310.10(H)'s parallel-conductor practice) and writes the result
        back here.

    design_ambient_temperature : float
        Ambient temperature used for cable sizing [K] (default: 273)

    conductor : Component
        Conductor properties (`radius`, `material` -- default `Copper()`,
        `resistance`), sized by `size_electrical_cable`

    insulator : Component
        Insulator properties (`radius`, `material` -- default `Polyimide()`)

    duplicate_wires : int
        Number of duplicate cables carried for redundancy (default: 2)

    Notes
    -----
    The electrical bus manages power distribution between sources and consumers,
    handling voltage regulation, power splitting, and battery management. It supports
    both series and parallel battery configurations.

    **Definitions**

    'C-rate'
        Rate at which a battery is charged/discharged relative to its capacity
        
    'Power Split Ratio'
        Fraction of total power handled by this bus in multi-bus configurations

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules
        Battery module components
    """
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """                
        self.tag                                       = 'electrical_line' 
        self.domain                                    = 'electrical'  
        self.active                                    = True
        self.design_power                              = 0.0
        self.design_voltage                            = 0.0  
        self.frequency                                 = 0.0    
        self.current_type                              = 'DC'   
        self.efficiency                                = 1
        self.length                                    = 1
        self.number_of_parallel_wires                  = None
        self.design_ambient_temperature                = 273  
        self.maximum_insulator_electric_field          = 0  
        self.maximum_operating_temperature             = 0 
        self.maximum_current                           = 0  
        self.maximum_temperature                       = 423
        self.conductor                                 = Component()
        self.conductor.radius                          = None
        self.conductor.material                        = Copper()  
        self.conductor.resistance                      = None
        self.insulator                                 = Component()
        self.insulator.radius                          = None
        self.insulator.material                        = Polyimide()  
        self.duplicate_wires                           = 2 


    def append_operating_conditions(self, segment):
        """
        Append operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing operating conditions
        """
        append_bus_conditions(self, segment)
        return
        
    def append_segment_conditions(self, segment):
        """
        Append segment-specific conditions to the bus

        Parameters
        ----------
        segment : Segment
            Flight segment data
        """
        append_bus_segment_conditions(self,segment)
        return      
    
    
    def compute_distribution_losses(self, component_conditions, state, network):
        compute_electrical_bus_distribution_losses(self, component_conditions, state, network) 
        return

    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]):
        """
        Computes the moment of inertia tensor for the electrical bus.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]] 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.moments_of_inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """ 
        return
 
    def initialize(self,network):
        """
        Computes the bus's design voltage and power from its assigned sources,
        converters, and propulsor-integrated generators, then sizes its cable.

        Batteries and fuel cells (bare Generic_Fuel_Cell_Stack, or one wrapped in a
        Reformer_Fuel_Cell composite) set `design_voltage` directly, since they define
        the bus's electrical potential. Generators only add to `design_power`,
        since their output voltage follows the bus rather than setting it -- so a
        generator-only bus keeps whatever `design_voltage` the vehicle definition
        set explicitly.

        An IDG/IDM's wiring is read from its parent propulsor (matching
        analyze_topology's convention), not its own assigned_distributors, which
        is always None. Multiple IDGs on one bus carry the same shared target in
        their own design_power (not a per-engine portion), so they're combined
        with max(), not summed.

        Parameters
        ----------
        network : Network
            Energy network this bus belongs to, used to find the sources,
            converters, and propulsors assigned to it.
        """
        self.design_power = 0.0

        def assigned_here(component):
            return component.active and component.assigned_distributors is not None and (self.tag in component.assigned_distributors[0])

        def generator_design_power(generator): 
            return getattr(generator, 'design_power', None) or getattr(getattr(generator, 'generator', None), 'design_power', 0.0) or 0.0

        for source in network.sources:
            if assigned_here(source):
                if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                    if source.power_split_ratio != 0.0:
                        self.design_voltage = source.voltage
                        # maximum_power is the pack's physical max-discharge ceiling, sized
                        # off design_capacity for endurance -- it can vastly exceed what the
                        # pack is actually asked to deliver (design_power, the vehicle's own
                        # target for this bus's load). Cable/bus sizing should track the
                        # latter: the former never overwrites design_power the way fuel-cell
                        # sizing does, so it stays a true "never asked to draw more than this"
                        # cap here, not a byproduct of energy-capacity sizing.
                        source_power = source.maximum_power
                        if source.design_power is not None:
                            source_power = min(source_power, source.design_power)
                        self.design_power += source_power

        for converter in network.converters:
            if assigned_here(converter):
                if isinstance(converter, RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack): 
                    self.design_voltage = converter.voltage
                    self.design_power  += converter.maximum_power
                elif isinstance(converter, RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell): 
                    stack = converter.fuel_cell
                    if stack is not None and isinstance(stack, RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack):
                        self.design_voltage = stack.voltage
                        self.design_power  += stack.maximum_power
                elif isinstance(converter, (RCAIDE.Library.Components.Powertrain.Converters.Generator,
                                            RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator)):
                    self.design_power  += generator_design_power(converter)

        # Propulsor-integrated generators aren't in network.converters, so check separately.
        for propulsor in network.propulsors:
            idg = getattr(propulsor, 'integrated_drive_generator', None)
            if idg is not None and assigned_here(propulsor):
                self.design_power = max(self.design_power, generator_design_power(idg))

        size_electrical_cable(self)
        return
    
    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for the distributor. 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the moment of inertia calculation
        """ 
        return 
