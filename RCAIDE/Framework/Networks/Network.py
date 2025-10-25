# RCAIDE/Framework/Networks/Network.py 
#
# Created:  Mar 2025, M.Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports
import  RCAIDE 
from RCAIDE.Framework.Mission.Common                      import Residuals 
from RCAIDE.Library.Mission.Common.Unpack_Unknowns.energy import unknowns
from RCAIDE.Library.Methods.Powertrain.Systems.compute_systems_power_draw                 import compute_systems_power_draw
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.compute_motor_performance         import *
from RCAIDE.Library.Methods.Powertrain.Converters.Generator.compute_generator_performance import * 
from RCAIDE.Library.Components import Component

# python imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Network
# ---------------------------------------------------------------------------------------------------------------------- 
class Network(Component):  
    """ 
    
    Generalized Energy Network (powertrain) Class capable of creating all derivatives of conventional
    and unconventional powertrains, including hybrid-electric powertrains and the all-electric network.
    
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
        Identifier for the network   
    
    Notes
    -----
    The evaluate function is broken into three sections: Section 1 computes all the forces and moments
    from propulsors regardless of if they are powered by fuel or an electrochemical energy storage system;
    Section 2 computees the perfomrance of any converters on the distrution lines, for example,
    turboshafts, motors, pumps etc; and Section 3 computes the thermal mangement of the system as
    well as energy consumtion of the powertrain. The state of storage devices such as covnentional fuel tanks,
    batteries are also updates. Propulsor groups can be "active" or "inactive" to simulate
    engine out conditions. Energy consumtion from avionics is also modeled 
    
    **Definitions** 
    'Propulsor Group'
        Any single or group of Components that work together to provide thrust.

    
    
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
        self.tag                          = 'network'
        self.propulsors                   = Container() 
        self.converters                   = Container()
        self.non_propulsive_converters    = []
        self.nacelles                     = Container()
        self.modulators                   = Container()
        self.distributors                 = Container()
        self.sources                      = Container()
        self.systems                      = Container() 

    def evaluate(network,state,center_of_gravity):
        """ Computes the performance of the network.
        
            Notes
            -----
            * Power balance uses the sign convention: 
              negative = leaving the distributor, positive = feeding the distributor.
        """

        # unpack
        conditions              = state.conditions
        propulsors              = network.propulsors
        converters              = network.converters
        distributors            = network.distributors
        modulators              = network.modulators
        sources                 = network.sources
        systems                 = network.systems

        total_thrust            = 0. * state.ones_row(3)
        total_moment            = 0. * state.ones_row(3)
        total_mdot              = 0. * state.ones_row(1)
        total_propulsive_power  = 0. * state.ones_row(1)

        # ----------------------------------------------------------
        # Propulsors
        # ----------------------------------------------------------
        stored_results_flag  = False
        for propulsor in propulsors:
            if propulsor.active:
                if propulsor.identical_propulsors == False or stored_results_flag == False:
                    inputs, outputs, stored_results_flag, stored_propulsor_tag = propulsor.compute_performance(state, network, center_of_gravity=center_of_gravity)
                else:
                    inputs, outputs = propulsor.reuse_stored_data(state, network, stored_propulsor_tag=stored_propulsor_tag, center_of_gravity=center_of_gravity)

                if propulsor.reverse_thrust == True:
                    total_thrust = outputs.thrust * -1
                    total_moment = outputs.moment * -1

                total_thrust           += outputs.thrust
                total_moment           += outputs.moment
                total_propulsive_power += outputs.power.propulsive

                if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or \
                   isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet) or \
                   isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop) or \
                   isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine) or \
                   isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine):
                    total_mdot += inputs.power.chemical / network.sources.fuel_tank.fuel.lower_heating_value

        # ----------------------------------------------------------
        # Systems
        # ----------------------------------------------------------
        for system in systems:
            _, _ = system.compute_performance(state)

        n_rows     = len(distributors)

        distributor_tags = []
        for dist in distributors:
            distributor_tags.append(dist.tag)

        for t_idx in range(len(state.conditions.energy.propulsors[next(iter(propulsors)).tag].outputs.power.electrical)):

            b_vector = np.zeros((n_rows,1))
            unknown_rows  = []
            unknown_signs = []

            # propulsors → known/unknown per distributor, sign: outputs +, inputs −
            for propulsor in propulsors:
                for distributors_tag in propulsor.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        row_index = 0
                        for i in range(n_rows):
                            if distributor_tags[i] == distributor_tag:
                                row_index = i
                        distributor = distributors[distributor_tag]
                        if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan):
                            if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                                val = conditions.energy.propulsors[propulsor.tag].outputs.power.electrical[t_idx,0]
                                if val == 0.0:
                                    unknown_rows.append(row_index)
                                    unknown_signs.append(+1.0)
                                else:
                                    b_vector[row_index,0] += val
                            elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                                val = conditions.energy.propulsors[propulsor.tag].inputs.power.chemical[t_idx,0]
                                if val == 0.0:
                                    unknown_rows.append(row_index)
                                    unknown_signs.append(-1.0)
                                else:
                                    b_vector[row_index,0] -= val

            # converters (non-propulsive) → electrical outputs feed (+), chemical inputs draw (−)
            for converter in network.non_propulsive_converters:
                for distributors_tag in converter.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        row_index = 0
                        for i in range(n_rows):
                            if distributor_tags[i] == distributor_tag:
                                row_index = i
                        distributor = distributors[distributor_tag]
                        if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                            val = conditions.energy.converters[converter.tag].outputs.power.electrical[t_idx,0]
                            if val == 0.0:
                                unknown_rows.append(row_index)
                                unknown_signs.append(+1.0)
                            else:
                                b_vector[row_index,0] += val
                        elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                            val = conditions.energy.converters[converter.tag].inputs.power.chemical[t_idx,0]
                            if val == 0.0:
                                unknown_rows.append(row_index)
                                unknown_signs.append(-1.0)
                            else:
                                b_vector[row_index,0] -= val

            # modulators → TRU rule: AC side uses input (draw −), DC side uses output (feed +)
            for modulator in modulators:
                for distributors_tag in modulator.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        row_index = 0
                        for i in range(n_rows):
                            if distributor_tags[i] == distributor_tag:
                                row_index = i
                        distributor = distributors[distributor_tag]
                        if isinstance(modulator, RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit):
                            if distributor.type == 'AC':
                                val_in = conditions.energy.modulators[modulator.tag].inputs.power.electrical[t_idx,0]
                                if val_in == 0.0:
                                    unknown_rows.append(row_index)
                                    unknown_signs.append(-1.0)
                                else:
                                    b_vector[row_index,0] -= val_in
                            elif distributor.type == 'DC':
                                val_out = conditions.energy.modulators[modulator.tag].outputs.power.electrical[t_idx,0]
                                if val_out == 0.0:
                                    unknown_rows.append(row_index)
                                    unknown_signs.append(+1.0)
                                else:
                                    b_vector[row_index,0] += val_out

            # sources → electrical outputs feed (+), chemical outputs feed (+)
            for source in sources:
                for distributors_tag in source.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        row_index = 0
                        for i in range(n_rows):
                            if distributor_tags[i] == distributor_tag:
                                row_index = i
                        distributor = distributors[distributor_tag]
                        if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                            val = conditions.energy.sources[source.tag].outputs.power.electrical[t_idx,0]
                            if val == 0.0:
                                unknown_rows.append(row_index)
                                unknown_signs.append(+1.0)
                            else:
                                b_vector[row_index,0] += val
                        elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                            val = conditions.energy.sources[source.tag].outputs.power.chemical[t_idx,0]
                            if val == 0.0:
                                unknown_rows.append(row_index)
                                unknown_signs.append(+1.0)
                            else:
                                b_vector[row_index,0] += val

            # systems → electrical inputs draw (−)
            for system in systems:
                for distributors_tag in system.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        row_index = 0
                        for i in range(n_rows):
                            if distributor_tags[i] == distributor_tag:
                                row_index = i
                        val = conditions.energy.systems[system.tag].inputs.power.electrical[t_idx,0]
                        if val == 0.0:
                            unknown_rows.append(row_index)
                            unknown_signs.append(-1.0)
                        else:
                            b_vector[row_index,0] -= val

            # distributor ↔ distributor links → create two unknowns (one per row), keep A as 0/1
            for dist_a in distributors:
                for distributors_tag in dist_a.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        name_a = dist_a.tag
                        name_b = distributor_tag
                        add_link = False
                        if name_a < name_b:
                            add_link = True
                        if add_link:
                            row_a = 0
                            row_b = 0
                            for i in range(n_rows):
                                if distributor_tags[i] == name_a:
                                    row_a = i
                                if distributor_tags[i] == name_b:
                                    row_b = i
                            unknown_rows.append(row_a)
                            unknown_signs.append(-1.0)
                            unknown_rows.append(row_b)
                            unknown_signs.append(+1.0)

            n_unknowns = len(unknown_rows)

            A_matrix = np.zeros((n_rows, n_unknowns))
            for j in range(n_unknowns):
                r = unknown_rows[j]
                A_matrix[r, j] = 1.0

            x_mag, _, _, _ = np.linalg.lstsq(A_matrix, b_vector, rcond=None)

            x_signed = np.zeros((n_unknowns,1))
            for j in range(n_unknowns):
                x_signed[j,0] = unknown_signs[j] * x_mag[j,0]

            # optional debug prints
            for i in range(n_rows):
                print("[A·x=b] row", i, "tag", distributor_tags[i], " b=", float(b_vector[i,0]))
            for j in range(n_unknowns):
                sgn = "+"
                if unknown_signs[j] < 0.0:
                    sgn = "-"
                print("[x]", j, sgn, "row", unknown_rows[j], "=", float(abs(x_signed[j,0])))
                debug = 0
                
        # save the computed unknown powers back to the state.conditions.energy.distributor[distributor.tag].power[domain] 
        # and on each corresponding component inputs/outputs.power 

        # print the power balance for each row distirbutpr (so independent distributprs and connected ones) for debugging purposes  

        # print the value of each unknown power for debugging purposes     
            

        # Final aggregation
        conditions.energy.thrust_force_vector  = total_thrust
        conditions.energy.thrust_moment_vector = total_moment
        conditions.energy.net_power            = total_propulsive_power
        conditions.weights.vehicle_mass_rate   = total_mdot
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
        for network in segment.analyses.energy.vehicle.networks:
            for propulsor in network.propulsors:
                propulsor.unpack_propulsor_unknowns(segment) 
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
        for network in segment.analyses.energy.vehicle.networks:
            for propulsor_i, propulsor in enumerate(network.propulsors):    
                if propulsor.active:
                    propulsor =  network.propulsors[propulsor.tag]
                    propulsor.pack_propulsor_residuals(segment) 
        return      
    
    def add_unknowns_and_residuals_to_segment(self, segment):
        """ This function sets up the information that the mission needs to run a mission segment using this network 
         
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            segment
            eestimated_throttles           [-]
            estimated_propulsor_group_rpms [-]  
            
            Outputs:
            segment
    
            Properties Used:
            N/A
        """                   
        segment.state.residuals.network = Residuals()
        
        for network in segment.analyses.energy.vehicle.networks:
            
            for propulsor in network.propulsors: 
                propulsor.append_operating_conditions(segment, network)  
                propulsor.append_propulsor_unknowns_and_residuals(segment)   
    
            for converter in network.converters: 
                converter.append_operating_conditions(segment)  

            for modulator in network.modulators: 
                modulator.append_operating_conditions(segment)  

            for source in  network.sources: 
                source.append_operating_conditions(segment)  

            for system in network.systems:
                system.append_operating_conditions(segment)             
    
            for distributor_i, distributor in enumerate(network.distributors):
                distributor.append_operating_conditions(segment)              
                
                # # Assign network-specific  residuals, unknowns and results data structures 
                # if distributor.active:
                #     for propulsor_group in  distributor.assigned_propulsors:
                #         propulsor =  network.propulsors[propulsor_group[0]]
                #         propulsor.append_propulsor_unknowns_and_residuals(segment)

                #     for converter_group in  distributor.assigned_converters:
                #         converter =  network.converters[converter_group[0]]
                        
                # if isinstance(distributor,RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line):                                                    
                #     # ------------------------------------------------------------------------------------------------------            
                #     # Create coolant_lines results data structure  
                #     # ------------------------------------------------------------------------------------------------------
                #     segment.state.conditions.energy.distributors[distributor.tag] = RCAIDE.Framework.Mission.Common.Conditions()        
                    
                #     # ------------------------------------------------------------------------------------------------------
                #     # Assign network-specific  residuals, unknowns and results data structures
                #     # ------------------------------------------------------------------------------------------------------       
                #     for battery_module in distributor.assigned_sources: 
                #         for btms in battery_module:
                #             btms.append_operating_conditions(segment,distributor)
                            
                #     for heat_exchanger in distributor.heat_exchangers: 
                #         heat_exchanger.append_operating_conditions(segment, distributor)
                            
                #     for reservoir in distributor.reservoirs: 
                #         reservoir.append_operating_conditions(segment, distributor)                           
    
        # Ensure the mission knows how to pack and unpack the unknowns and residuals
        segment.process.iterate.unknowns.network            = self.unpack_unknowns
        segment.process.iterate.residuals.network           = self.residuals   
        
        return segment

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
        for net in self.values():             
            net.evaluate(state,center_of_gravity)  
        return   

# ----------------------------------------------------------------------
#  Handle Linking
# ----------------------------------------------------------------------
Network.Container = Container