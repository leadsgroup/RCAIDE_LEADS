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

    hybrid_power_split_ratio : float
        Fraction of total propulsive power supplied by electrical sources, denoted
        as phi. This ratio controls how much electrical power is routed to the
        integrated drive motor on the propulsor shaft and how much power is drawn
        from electrochemical sources (batteries, fuel cells).

        - phi = 0.0 : all power from fuel (conventional turbofan/turbojet)
        - phi = 0.5 : equal split between fuel and electric motor (parallel hybrid)
        - phi = 1.0 : all power from electrical sources (all-electric)

        The turbomachinery (compressor, fan) always computes the full thermodynamic
        work regardless of phi. The mechanical hybridization is handled through
        the external_shaft_work term in the turbine energy balance, which accounts
        for motor-supplied shaft power and motor efficiency losses.

        Default is 0.0.

    battery_fuel_cell_power_split_ratio : float
        Fraction of electrical power supplied by batteries versus fuel cells,
        denoted as psi. This ratio partitions the electrical power demand among
        electrochemical energy sources.

        - psi = 1.0 : all electrical power from batteries
        - psi = 0.5 : equal split between batteries and fuel cells
        - psi = 0.0 : all electrical power from fuel cells

        Default is 1.0.

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

    The two power split ratios (phi and psi) together define the complete energy
    sourcing strategy for the network::

        Total Propulsive Power
            |
            |--- (1 - phi) ---> Fuel (combustion) ---> Turbine shaft work
            |
            |--- (phi) -------> Electrical power
                                    |
                                    |--- (psi) ------> Batteries
                                    |
                                    |--- (1 - psi) --> Fuel Cells

    The actual electrical power (in Watts) is determined by the network solver,
    which finds the power level that satisfies the net electrical power balance
    residual (sum of all electrical sources minus all electrical sinks equals zero).

    **Definitions**

    'Propulsor Group'
        Any single or group of components that work together to provide thrust.

    See Also
    --------
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
        self.hybrid_power_split_ratio            = 0.0
        self.battery_fuel_cell_power_split_ratio = 1.0
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
        for propulsor in propulsors:
            if propulsor.active:
                if propulsor.identical_propulsors == False or stored_results_flag == False: 
                    inputs, outputs, stored_results_flag, stored_propulsor_tag = propulsor.compute_performance(state,network,center_of_gravity=center_of_gravity)
                else:
                    inputs, outputs = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag, center_of_gravity=center_of_gravity)

                if propulsor.reverse_thrust == True:
                    total_thrust = outputs.thrust * -1
                    total_moment = outputs.moment * -1

                total_thrust           += outputs.thrust
                total_moment           += outputs.moment 
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
        stored_results_flag  = False
        for converter in converters:   
            if converter.active: 
                if converter.identical_converters == False or stored_results_flag == False: 
                    converter.reverse_mode_computation = True
                    inputs, outputs, stored_results_flag, stored_conveter_tag = converter.compute_performance(state,network)
                else:
                    inputs, outputs = converter.reuse_stored_data(state,network,stored_conveter_tag=stored_conveter_tag)  
                total_mdot             += state.conditions.energy.propulsors[converter.tag].fuel_mass_flow_rate   
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
        N/A
        
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
            for p_i, propulsor in enumerate(network.propulsors):
                if propulsor.active and (propulsor.identical_propulsors == False or p_i == 0): 
                    propulsor.unpack_unknowns(segment) 
            for s_i, source in enumerate(network.sources):
                if source.active and (source.identical_sources == False or s_i == 0): 
                    source.unpack_unknowns(segment) 
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
            for p_i, propulsor in enumerate(network.propulsors):    
                if propulsor.active and (propulsor.identical_propulsors == False or p_i == 0):
                    propulsor.pack_residuals(segment) 
            for s_i, source in enumerate(network.sources):
                if source.active and (source.identical_sources == False or s_i == 0): 
                    source.pack_residuals(segment) 
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