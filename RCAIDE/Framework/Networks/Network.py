# RCAIDE/Framework/Networks/Network.py 
#
# Created:  Mar 2025, M.Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports
import  RCAIDE 
from RCAIDE.Framework.Mission.Common     import Residuals, Conditions
from RCAIDE.Library.Mission.Common.Unpack_Unknowns.energy import unknowns 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.compute_motor_performance         import *
from RCAIDE.Library.Methods.Powertrain.Converters.Generator.compute_generator_performance import * 
from RCAIDE.Library.Components import Component

# python imports 
import numpy as np 
import scipy.linalg as sla

# ----------------------------------------------------------------------------------------------------------------------
#  Network
# ---------------------------------------------------------------------------------------------------------------------- 
class Network(Component):
    """
    Generalized Energy Network (powertrain) class capable of creating all derivatives of conventional
    and unconventional powertrains, including hybrid-electric powertrains and all-electric networks.

                                            GENERIC NETWORK
          .........................................:..........................................
          :                        :                                :                        :
    .-------------.         .-------------.                 .-------------.           .-------------.
    | propulsor 1 |         | propulsor 2 |                 | propulsor 2 |           | propulsor 3 |
    '-------------'         '-------------'                 '-------------'           '-------------'
          ||                       ||                              ||                        ||
          ||   .-------------.     ||                              ||  .-------------.       ||
          ||== | converter 1 |====== electric bus / fuel line =========| converter 2 |=======||
               '-------------'                                         '-------------'

    Attributes
    ----------
    tag : str
        Identifier for the network.

    reverse_thrust : bool
        Flag to enable reverse thrust computation. Default is False.

    propulsors : Container
        Collection of propulsor components (turbofans, rotors, etc.).

    converters : Container
        Collection of converter components (turboshafts, motors, generators, etc.).

    nacelles : Container
        Collection of nacelle components.

    modulators : Container
        Collection of modulator components.

    distributors : Container
        Collection of distributor components (electrical buses, fuel lines, etc.).

    sources : Container
        Collection of energy source components (fuel tanks, battery packs, etc.).

    systems : Container
        Collection of system components (avionics, environmental controls, etc.).

    Notes
    -----
    The evaluate function is broken into four sections:

    1. **Propulsors** — computes forces and moments from all active propulsors.
    2. **Systems** — computes power consumption from auxiliary systems (avionics,
       environmental controls, hydraulics, etc.).
    3. **Converters** — computes performance of converters on distribution lines
       (turboshafts, motors, pumps, fuel cell stacks, etc.).
    4. **Sources** — computes energy consumption and state updates for storage
       devices (fuel tanks, batteries).

    Propulsor groups can be set to active or inactive to simulate engine-out
    conditions.

    **Power Split Architecture**

    Hybridization is controlled by two ratios set on the mission segment (not the
    network):

    - **phi** (``hybrid_power_split_ratio``) — fraction of propulsive power from
      electrical sources (0 = all fuel, 1 = all electric).
    - **psi** (``battery_fuel_cell_power_split_ratio``) — fraction of electrical
      power from batteries vs. other electrical providers — fuel cells or
      generators (e.g. Turboelectric_Generator) (1 = all battery, 0 = all
      fuel cell/generator).

    ::

        Total Propulsive Power
            |
            |--- (1 - phi) ---> Fuel (combustion) ---> Turbine shaft work
            |
            |--- (phi) -------> Electrical power
                                    |
                                    |--- (psi) ------> Batteries
                                    |
                                    |--- (1 - psi) --> Fuel Cells / Generators

    These ratios are resolved during pre-processing by
    ``RCAIDE.Library.Mission.Common.Pre_Process.energy``, which analyzes the
    network topology and either uses user-specified values from the segment,
    auto-derives defaults for simple topologies, or registers them as
    optimization variables for ambiguous configurations.

    **Definitions**

    'Propulsor Group'
        Any single or group of components that work together to provide thrust.

    See Also
    --------
    RCAIDE.Framework.Mission.Segments.Evaluate
        Where phi and psi are set per mission segment
    RCAIDE.Library.Mission.Common.Pre_Process.energy
        Topology analysis and phi/psi resolution
    RCAIDE.Library.Framework.Networks.Fuel
        Fuel network class
    RCAIDE.Library.Framework.Networks.Fuel_Cell
        Fuel_Cell network class
    RCAIDE.Library.Framework.Networks.Electric
        All-Electric network class
    """

    def __defaults__(self):
        """ This sets the default values for the network to function.
        """
        self.tag                                 = 'network'
        self.reverse_thrust                      = False
        self.propulsors                          = Container()
        self.converters                          = Container()
        self.nacelles                            = Container()
        self.modulators                          = Container()
        self.distributors                        = Container()
        self.sources                             = Container()
        self.systems                             = Container()
         

    def evaluate(network,state,vehicle):
        """ Computes the performance of the network.
        
            Notes
            -----
            * Power balance uses the sign convention: 
              negative = leaving the distributor, positive = feeding the distributor.
        """

        # unpack
        center_of_gravity = vehicle.mass_properties.center_of_gravity
        conditions        = state.conditions
        propulsors        = network.propulsors
        converters        = network.converters  
        distributors      = network.distributors
        modulators        = network.modulators
        sources           = network.sources
        systems           = network.systems

        total_thrust            = 0. * state.ones_row(3)
        total_moment            = 0. * state.ones_row(3)
        total_mdot              = 0. * state.ones_row(1)
        total_propulsive_power  = 0. * state.ones_row(1)
        net_chemical_power      = 0. * state.ones_row(1) 
        net_electrical_power    = 0. * state.ones_row(1)
        net_thermal_power       = 0. * state.ones_row(1)
        net_hydraulic_power     = 0. * state.ones_row(1)
 
        # ----------------------------------------------------------
        # Propulsors
        # ----------------------------------------------------------
        stored_results_flag  = False
        stored_propulsor_tag = None
        for propulsor in propulsors:
            if propulsor.active:
                # "identical_propulsors" only makes reuse valid within a run of
                # truly identical propulsors -- e.g. a vehicle with both cruise
                # propellers and lift rotors has two distinct groups, and a
                # propulsor must never reuse another group's results just
                # because it inherited the default identical_propulsors=True.
                # assigned_distributors differing is a reliable, always-available
                # signal that the propulsor belongs to a different group.
                same_group = (stored_results_flag == True and
                              propulsor.assigned_distributors == propulsors[stored_propulsor_tag].assigned_distributors)
                if propulsor.identical_propulsors == False or not same_group:
                    inputs, outputs, stored_results_flag, stored_propulsor_tag = propulsor.compute_performance(state,network,center_of_gravity=center_of_gravity)
                else:
                    inputs, outputs = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag, center_of_gravity=center_of_gravity)

                propulsor_thrust = outputs.thrust
                propulsor_moment = outputs.moment
                # network.reverse_thrust is the network-wide (all propulsors)
                # flag vehicle configs commonly set; propulsor.reverse_thrust
                # allows overriding it per propulsor. Either being True reverses
                # this propulsor's contribution.
                if propulsor.reverse_thrust == True or network.reverse_thrust == True:
                    propulsor_thrust = propulsor_thrust * -1
                    propulsor_moment = propulsor_moment * -1

                total_thrust           += propulsor_thrust
                total_moment           += propulsor_moment
                total_mdot             += state.conditions.energy.propulsors[propulsor.tag].fuel_mass_flow_rate  
                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)   
                net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic)
                net_chemical_power     += (outputs.power.chemical - inputs.power.chemical) 
                     
                for input_power_type in inputs.power.keys():
                    state.conditions.energy.inputs.power[input_power_type] += inputs.power[input_power_type] 

                for output_power_type in outputs.power.keys():
                    state.conditions.energy.outputs.power[output_power_type] += outputs.power[output_power_type] 
                # compute losses for assigned distributors
                if propulsor.assigned_distributors != None:
                    for distributor_tag in propulsor.assigned_distributors[0]: 
                        distributor = network.distributors[distributor_tag]  
                        distributor.compute_distribution_losses(state.conditions.energy.propulsors[propulsor.tag],state,network)   
 
                        state.conditions.energy.distributors[distributor_tag].outputs.power[distributor.domain]   += inputs.power[distributor.domain]  
                        state.conditions.energy.distributors[distributor_tag].inputs.power[distributor.domain]    += outputs.power[distributor.domain] 
   
        # ----------------------------------------------------------
        # Systems
        # ----------------------------------------------------------
        stored_results_flag  = False
        for system in systems:
            if system.active: 
                inputs, outputs,_,_= system.compute_performance(state,vehicle)  
                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)
                net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic)
                net_chemical_power     += (outputs.power.chemical - inputs.power.chemical) 

                for input_power_type in inputs.power.keys():
                    state.conditions.energy.inputs.power[input_power_type] += inputs.power[input_power_type] 

                for output_power_type in outputs.power.keys():
                    state.conditions.energy.outputs.power[output_power_type] += outputs.power[output_power_type] 

                # compute losses for assigned distributors
                if system.assigned_distributors != None:
                    for distributor_tag in system.assigned_distributors[0]:
                        distributor = network.distributors[distributor_tag]
                        distributor.compute_distribution_losses(state.conditions.energy.systems[system.tag],state,network)

                        state.conditions.energy.distributors[distributor_tag].outputs.power[distributor.domain]   += inputs.power[distributor.domain]
                        state.conditions.energy.distributors[distributor_tag].inputs.power[distributor.domain]    += outputs.power[distributor.domain]
                
        # ----------------------------------------------------------
        # Converters 
        # ----------------------------------------------------------                
        stored_results_flag   = False
        stored_converter_tag  = None
        for converter in converters:
            if converter.active:
                # See the matching comment on the propulsor loop above: reuse is
                # only valid within a run of truly identical converters sharing
                # the same distributor group, not just "any converter computed
                # so far in this network."
                same_group = (stored_results_flag == True and
                              converter.assigned_distributors == converters[stored_converter_tag].assigned_distributors)
                if converter.identical_converters == False or not same_group:
                    converter.reverse_mode_computation = True
                    inputs, outputs, stored_results_flag, stored_converter_tag = converter.compute_performance(state,network)
                else:
                    inputs, outputs = converter.reuse_stored_data(state,network,stored_conveter_tag=stored_converter_tag)
                total_mdot             += state.conditions.energy.converters[converter.tag].fuel_mass_flow_rate
                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)
                net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic)       
                net_chemical_power     += (outputs.power.chemical - inputs.power.chemical)  

                for input_power_type in inputs.power.keys():
                    state.conditions.energy.inputs.power[input_power_type] += inputs.power[input_power_type] 

                for output_power_type in outputs.power.keys():
                    state.conditions.energy.outputs.power[output_power_type] += outputs.power[output_power_type] 
                
                # compute losses for assigned distributors
                if converter.assigned_distributors != None:
                    for distributor_tag in converter.assigned_distributors[0]:
                        distributor = network.distributors[distributor_tag]
                        distributor.compute_distribution_losses(state.conditions.energy.converters[converter.tag],state,network)

                        state.conditions.energy.distributors[distributor_tag].outputs.power[distributor.domain]   += inputs.power[distributor.domain]
                        state.conditions.energy.distributors[distributor_tag].inputs.power[distributor.domain]    += outputs.power[distributor.domain]

        # ----------------------------------------------------------
        # Sources 
        # ----------------------------------------------------------
        stored_results_flag  = False
        for source in sources: 
            if source.active:    
                inputs, outputs, _, _  = source.compute_performance(state,network)   
                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)
                net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic)  
                net_chemical_power     += (outputs.power.chemical - inputs.power.chemical)

                if source.assigned_distributors != None:
                    for distributor_tag in source.assigned_distributors[0]:
                        distributor = network.distributors[distributor_tag]
                        distributor.compute_distribution_losses(state.conditions.energy.sources[source.tag],state,network)

                        state.conditions.energy.distributors[distributor_tag].outputs.power[distributor.domain]   += inputs.power[distributor.domain]
                        state.conditions.energy.distributors[distributor_tag].inputs.power[distributor.domain]    += outputs.power[distributor.domain]


        # ----------------------------------------------------------
        # Distributors
        # ----------------------------------------------------------
        # Runs after sources so that any per-source condition values a
        # distributor's own performance depends on (e.g. heat delivered to a
        # coolant loop by the battery modules it cools) are already computed.
        for distributor in distributors:
            if distributor.active:
                distributor.compute_performance(state,network)

        # Final aggregation for system level performance
        conditions.energy.total_force_vector       = total_thrust
        conditions.energy.total_moment_vector      = total_moment
        conditions.weights.vehicle.mass_rate       = total_mdot
        conditions.energy.net_electrical_power     = net_electrical_power
        conditions.energy.net_thermal_power        = net_thermal_power
        conditions.energy.net_hydraulic_power      = net_hydraulic_power
        conditions.energy.net_chemical_power       = net_chemical_power

        return
     
       
    def append_segment_conditions(self,segment): 
 
        segment.conditions.energy.inputs.power.propulsive[:,0]     = 0.0
        segment.conditions.energy.inputs.power.mechanical[:,0]     = 0.0
        segment.conditions.energy.inputs.power.electrical[:,0]     = 0.0
        segment.conditions.energy.inputs.power.chemical[:,0]       = 0.0
        segment.conditions.energy.inputs.power.pneumatic[:,0]      = 0.0
        segment.conditions.energy.inputs.power.hydraulic[:,0]      = 0.0
        segment.conditions.energy.inputs.power.thermal[:,0]        = 0.0
        segment.conditions.energy.outputs.power.propulsive[:,0]    = 0.0
        segment.conditions.energy.outputs.power.mechanical[:,0]    = 0.0
        segment.conditions.energy.outputs.power.electrical[:,0]    = 0.0
        segment.conditions.energy.outputs.power.chemical[:,0]      = 0.0
        segment.conditions.energy.outputs.power.pneumatic[:,0]     = 0.0
        segment.conditions.energy.outputs.power.hydraulic[:,0]     = 0.0
        segment.conditions.energy.outputs.power.thermal[:,0]       = 0.0 
              
        return 
        
    
    def unpack_unknowns(self,segment):
        """Unpacks the unknowns set in the mission to be available for the mission.
    
        Assumptions: 
                    # See the matching comment in Network.evaluate(): a propulsor
                    # can only be treated as identical to the most recently
                    # unpacked one if they also share the same distributor group.
        
        Source:
        N/A
        
        Inputs: 
            segment   - data structure of mission segment [-]
        
        Outputs: 
        
        Properties Used:
        N/A
        """            
         
        unknowns(segment)
        for network in segment.analyses.vehicle.networks:
            reference_distributors = None
            for p_i, propulsor in enumerate(network.propulsors):
                if propulsor.active:
                    if propulsor.identical_propulsors == False or reference_distributors is None or propulsor.assigned_distributors != reference_distributors:
                        propulsor.unpack_unknowns(segment)
                        reference_distributors = propulsor.assigned_distributors
            reference_source_distributors = None
            for source in network.sources:
                if source.active:
                    if source.identical_sources == False or reference_source_distributors is None or source.assigned_distributors != reference_source_distributors:
                        source.unpack_unknowns(segment)
                        reference_source_distributors = source.assigned_distributors
            for modulator in network.modulators:
                modulator.unpack_unknowns(segment)
            for distributor in network.distributors:
                distributor.unpack_unknowns(segment) 
            for system in network.systems:
                system.unpack_unknowns(segment) 
        return    
     
    def residuals(self,segment):
        """ This packs the residuals to be sent to the mission solver.
    
           Assumptions:
           None
    
           Source:
           N/A
    
           Inputs:
           state.conditions.energy:
               motor(s).torque                      [N-m]
               rotor(s).torque                      [N-m] 
           residuals soecific to the battery cell   
           
           Outputs:
           residuals specific to battery cell and network
    
           Properties Used: 
           N/A
       """         
        for network in segment.analyses.vehicle.networks:
            reference_distributors = None
            for p_i, propulsor in enumerate(network.propulsors):
                if propulsor.active:
                    if propulsor.identical_propulsors == False or reference_distributors is None or propulsor.assigned_distributors != reference_distributors:
                        propulsor.pack_residuals(segment)
                        reference_distributors = propulsor.assigned_distributors
            reference_source_distributors = None
            for source in network.sources:
                if source.active:
                    if source.identical_sources == False or reference_source_distributors is None or source.assigned_distributors != reference_source_distributors:
                        source.pack_residuals(segment)
                        reference_source_distributors = source.assigned_distributors
            for modulator in network.modulators:
                modulator.pack_residuals(segment)
            for distributor in network.distributors:
                distributor.pack_residuals(segment) 
            for system in network.systems:
                system.pack_residuals(segment) 
        return
    
# ----------------------------------------------------------------------
#  Component Container
# ---------------------------------------------------------------------- 
class Container(Component.Container):
    """ The Network container class 
    """
    def evaluate(self,state,center_of_gravity):
        """ This is used to evaluate the thrust and moments produced by the network.

            Assumptions:  
                If multiple networks are attached their performances will be summed

            Source:
                None 
        """ 

        self.evaluate(state,center_of_gravity)



# ----------------------------------------------------------------------
#  Handle Linking
# ----------------------------------------------------------------------
Network.Container = Container