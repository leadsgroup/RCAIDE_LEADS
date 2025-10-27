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

        for propulsor in propulsors:
            conditions.energy.propulsors[propulsor.tag].inputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.propulsors[propulsor.tag].inputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.propulsors[propulsor.tag].inputs.power.thermal = 0*state.ones_row(1)
            conditions.energy.propulsors[propulsor.tag].outputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.propulsors[propulsor.tag].outputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.propulsors[propulsor.tag].outputs.power.thermal = 0*state.ones_row(1)
        for converter in network.non_propulsive_converters:
            conditions.energy.converters[converter.tag].inputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.converters[converter.tag].inputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.converters[converter.tag].inputs.power.thermal = 0*state.ones_row(1)
            conditions.energy.converters[converter.tag].outputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.converters[converter.tag].outputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.converters[converter.tag].outputs.power.thermal = 0*state.ones_row(1)
        for modulator in modulators:
            conditions.energy.modulators[modulator.tag].inputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.modulators[modulator.tag].inputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.modulators[modulator.tag].inputs.power.thermal = 0*state.ones_row(1)
            conditions.energy.modulators[modulator.tag].outputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.modulators[modulator.tag].outputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.modulators[modulator.tag].outputs.power.thermal = 0*state.ones_row(1)
        for source in sources:
            conditions.energy.sources[source.tag].inputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.sources[source.tag].inputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.sources[source.tag].inputs.power.thermal = 0*state.ones_row(1)
            conditions.energy.sources[source.tag].outputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.sources[source.tag].outputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.sources[source.tag].outputs.power.thermal = 0*state.ones_row(1)
        for system in systems:
            conditions.energy.systems[system.tag].inputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.systems[system.tag].inputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.systems[system.tag].inputs.power.thermal = 0*state.ones_row(1)
            conditions.energy.systems[system.tag].outputs.power.electrical = 0*state.ones_row(1)
            conditions.energy.systems[system.tag].outputs.power.chemical = 0*state.ones_row(1)
            conditions.energy.systems[system.tag].outputs.power.thermal = 0*state.ones_row(1)

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

        # ----------------------------------------------------------
        # Build Power Balance System
        # ----------------------------------------------------------

        n_rows     = len(distributors)

        distributor_tags = []
        for dist in distributors:
            distributor_tags.append(dist.tag)

        for t_idx in range(state.numerics.number_of_control_points):

            b_vector = np.zeros((n_rows,1))

            unknown_cols = {}   # maps a key -> column index
            triplets = []       # (row, col, coeff) to populate A

            # propulsors
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
                                    key = ("propulsor", propulsor.tag, distributor_tag, "elec_out")
                                    if key not in unknown_cols:
                                        unknown_cols[key] = len(unknown_cols)
                                    triplets.append((row_index, unknown_cols[key], +1.0))
                                else:
                                    b_vector[row_index,0] += val
                            elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                                val = conditions.energy.propulsors[propulsor.tag].inputs.power.chemical[t_idx,0]
                                if val == 0.0:
                                    key = ("propulsor", propulsor.tag, distributor_tag, "chem_in")
                                    if key not in unknown_cols:
                                        unknown_cols[key] = len(unknown_cols)
                                    triplets.append((row_index, unknown_cols[key], -1.0))
                                else:
                                    b_vector[row_index,0] -= val

            # converters (non-propulsive) 
            for converter in network.non_propulsive_converters:
                for distributors_tag in converter.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        row_index = 0
                        for i in range(n_rows):
                            if distributor_tags[i] == distributor_tag:
                                row_index = i
                        distributor = distributors[distributor_tag]
                        key = ("converter", converter.tag, "P")
                        if key not in unknown_cols:
                            unknown_cols[key] = len(unknown_cols)
                        col = unknown_cols[key]
                        if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                            val = conditions.energy.converters[converter.tag].outputs.power.electrical[t_idx,0]
                            if val == 0.0:
                                triplets.append((row_index, col, +1.0))
                            else:
                                b_vector[row_index,0] += val
                        elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                            val = conditions.energy.converters[converter.tag].inputs.power.chemical[t_idx,0]
                            if val == 0.0:
                                triplets.append((row_index, col, -1.0))
                            else:
                                b_vector[row_index,0] -= val

            # modulators
            for modulator in modulators:
                for distributors_tag in modulator.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        row_index = 0
                        for i in range(n_rows):
                            if distributor_tags[i] == distributor_tag:
                                row_index = i
                        distributor = distributors[distributor_tag]
                        if isinstance(modulator, RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit):
                            eff = modulator.efficiency.electrical
                            vin  = conditions.energy.modulators[modulator.tag].inputs.power.electrical[t_idx,0]
                            vout = conditions.energy.modulators[modulator.tag].outputs.power.electrical[t_idx,0]
                            if vin != 0.0 and vout == 0.0:
                                conditions.energy.modulators[modulator.tag].outputs.power.electrical[t_idx,0] = vin * eff
                                vout = conditions.energy.modulators[modulator.tag].outputs.power.electrical[t_idx,0]
                            elif vout != 0.0 and vin == 0.0:
                                conditions.energy.modulators[modulator.tag].inputs.power.electrical[t_idx,0]  = vout / eff
                                vin  = conditions.energy.modulators[modulator.tag].inputs.power.electrical[t_idx,0]
                            elif vin != 0.0 and vout != 0.0:
                                conditions.energy.modulators[modulator.tag].outputs.power.electrical[t_idx,0] = vin * eff
                                vout = conditions.energy.modulators[modulator.tag].outputs.power.electrical[t_idx,0]
                            key = ("modulator_tru", modulator.tag, "P")
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols)
                            col = unknown_cols[key]
                            if distributor.type == 'AC':
                                if vin == 0.0:
                                    triplets.append((row_index, col, -1.0))
                                else:
                                    b_vector[row_index,0] -= vin
                            elif distributor.type == 'DC':
                                # Mirror link behavior: always add +eff to DC row when the DC side is unknown
                                if vout == 0.0:
                                    triplets.append((row_index, col, +eff))
                                else:
                                    b_vector[row_index,0] += vout

            # sources 
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
                                key = ("source", source.tag, distributor_tag, "elec_out")
                                if key not in unknown_cols:
                                    unknown_cols[key] = len(unknown_cols)
                                triplets.append((row_index, unknown_cols[key], +1.0))
                            else:
                                b_vector[row_index,0] += val
                        elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                            val = conditions.energy.sources[source.tag].outputs.power.chemical[t_idx,0]
                            if val == 0.0:
                                key = ("source", source.tag, distributor_tag, "chem_out")
                                if key not in unknown_cols:
                                    unknown_cols[key] = len(unknown_cols)
                                triplets.append((row_index, unknown_cols[key], +1.0))
                            else:
                                b_vector[row_index,0] += val

            # systems 
            for system in systems:
                for distributors_tag in system.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        row_index = 0
                        for i in range(n_rows):
                            if distributor_tags[i] == distributor_tag:
                                row_index = i
                        val = conditions.energy.systems[system.tag].inputs.power.electrical[t_idx,0]
                        if val == 0.0:
                            key = ("system", system.tag, distributor_tag, "elec_in")
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols)
                            triplets.append((row_index, unknown_cols[key], -1.0))
                        else:
                            b_vector[row_index,0] -= val

            # distributor ↔ distributor links 
            for dist_a in distributors:
                for distributors_tag in dist_a.assigned_distributors:
                    for distributor_tag in distributors_tag:
                        name_a = dist_a.tag
                        name_b = distributor_tag

                        # A -> B
                        if name_b not in state.conditions.energy.distributors[name_a].links:
                            linkAB                  = Conditions()
                            linkAB.power            = Conditions()
                            linkAB.power.electrical = 0 * state.ones_row(1)
                            linkAB.power.chemical   = 0 * state.ones_row(1)
                            state.conditions.energy.distributors[name_a].links[name_b] = linkAB

                        # B -> A
                        if name_a not in state.conditions.energy.distributors[name_b].links:
                            linkBA                  = Conditions()
                            linkBA.power            = Conditions()
                            linkBA.power.electrical = 0 * state.ones_row(1)
                            linkBA.power.chemical   = 0 * state.ones_row(1)
                            state.conditions.energy.distributors[name_b].links[name_a] = linkBA

                        if name_a < name_b:
                            row_a = 0
                            row_b = 0
                            for i in range(n_rows):
                                if distributor_tags[i] == name_a:
                                    row_a = i
                                if distributor_tags[i] == name_b:
                                    row_b = i
                            key = ("link", name_a, name_b)
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols)
                            col = unknown_cols[key]
                            triplets.append((row_a, col, -1.0))
                            triplets.append((row_b, col, +1.0))

            n_unknowns = len(unknown_cols)
            A_matrix = np.zeros((n_rows, n_unknowns))
            for r, c, coeff in triplets:
                A_matrix[r, c] += coeff

            # ----------------------------------------------------------
            # Solve Power Balance System
            # ----------------------------------------------------------

            x_solution, _, _, _ = np.linalg.lstsq(A_matrix, b_vector, rcond=None)
 
            # ----------------------------------------------------------
            # Save solved unknowns back into conditions.energy
            # ----------------------------------------------------------
            for key, col in unknown_cols.items():
                val = float(x_solution[col,0])

                if key[0] == "propulsor":
                    prop_tag        = key[1]
                    distributor_tag = key[2]
                    side            = key[3]
                    if side == "elec_out":
                        if val < 0.0:
                            if conditions.energy.propulsors[prop_tag].outputs.power.electrical[t_idx,0] == 0.0:
                                conditions.energy.propulsors[prop_tag].outputs.power.electrical[t_idx,0] = -val
                        elif val > 0.0:
                            if conditions.energy.propulsors[prop_tag].inputs.power.electrical[t_idx,0] == 0.0:
                                conditions.energy.propulsors[prop_tag].inputs.power.electrical[t_idx,0] = val
                    elif side == "chem_in":
                        if val < 0.0:
                            if conditions.energy.propulsors[prop_tag].outputs.power.chemical[t_idx,0] == 0.0:
                                conditions.energy.propulsors[prop_tag].outputs.power.chemical[t_idx,0] = -val
                        elif val > 0.0:
                            if conditions.energy.propulsors[prop_tag].inputs.power.chemical[t_idx,0] == 0.0:
                                conditions.energy.propulsors[prop_tag].inputs.power.chemical[t_idx,0] = val

                elif key[0] == "converter":
                    conv_tag = key[1]
                    for distributors_tag in network.non_propulsive_converters[conv_tag].assigned_distributors:
                        for distributor_tag in distributors_tag:
                            distributor = distributors[distributor_tag]
                            if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                                if val < 0.0:
                                    if conditions.energy.converters[conv_tag].outputs.power.electrical[t_idx,0] == 0.0:
                                        conditions.energy.converters[conv_tag].outputs.power.electrical[t_idx,0] = -val
                                elif val > 0.0:
                                    if conditions.energy.converters[conv_tag].inputs.power.electrical[t_idx,0] == 0.0:
                                        conditions.energy.converters[conv_tag].inputs.power.electrical[t_idx,0] = val
                            elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                                if val > 0.0:
                                    if conditions.energy.converters[conv_tag].inputs.power.chemical[t_idx,0] == 0.0:
                                        conditions.energy.converters[conv_tag].inputs.power.chemical[t_idx,0] = val
                                elif val < 0.0:
                                    if conditions.energy.converters[conv_tag].outputs.power.chemical[t_idx,0] == 0.0:
                                        conditions.energy.converters[conv_tag].outputs.power.chemical[t_idx,0] = -val

                elif key[0] == "modulator_tru":
                    tru_tag = key[1]
                    eff = modulators[tru_tag].efficiency.electrical
                    if val < 0.0:
                        if conditions.energy.modulators[tru_tag].inputs.power.electrical[t_idx,0] == 0.0:
                            conditions.energy.modulators[tru_tag].inputs.power.electrical[t_idx,0] = -val
                        if conditions.energy.modulators[tru_tag].outputs.power.electrical[t_idx,0] == 0.0:
                            conditions.energy.modulators[tru_tag].outputs.power.electrical[t_idx,0] = (-val) * eff
                    elif val > 0.0:
                        if conditions.energy.modulators[tru_tag].inputs.power.electrical[t_idx,0] == 0.0:
                            conditions.energy.modulators[tru_tag].inputs.power.electrical[t_idx,0] =  val
                        if conditions.energy.modulators[tru_tag].outputs.power.electrical[t_idx,0] == 0.0:
                            conditions.energy.modulators[tru_tag].outputs.power.electrical[t_idx,0] =  val * eff

                elif key[0] == "source":
                    src_tag        = key[1]
                    distributor_tag= key[2]
                    side           = key[3]
                    if side == "elec_out":
                        if val < 0.0:
                            if conditions.energy.sources[src_tag].outputs.power.electrical[t_idx,0] == 0.0:
                                conditions.energy.sources[src_tag].outputs.power.electrical[t_idx,0] = -val
                        elif val > 0.0:
                            if conditions.energy.sources[src_tag].inputs.power.electrical[t_idx,0] == 0.0:
                                conditions.energy.sources[src_tag].inputs.power.electrical[t_idx,0] = val
                    elif side == "chem_out":
                        if val < 0.0:
                            if conditions.energy.sources[src_tag].outputs.power.chemical[t_idx,0] == 0.0:
                                conditions.energy.sources[src_tag].outputs.power.chemical[t_idx,0] = -val
                        elif val > 0.0:
                            if conditions.energy.sources[src_tag].inputs.power.chemical[t_idx,0] == 0.0:
                                conditions.energy.sources[src_tag].inputs.power.chemical[t_idx,0] = val

                elif key[0] == "system":
                    sys_tag        = key[1]
                    distributor_tag= key[2]
                    if val > 0.0:
                        if conditions.energy.systems[sys_tag].inputs.power.electrical[t_idx,0] == 0.0:
                            conditions.energy.systems[sys_tag].inputs.power.electrical[t_idx,0] = val
                    elif val < 0.0:
                        if conditions.energy.systems[sys_tag].outputs.power.electrical[t_idx,0] == 0.0:
                            conditions.energy.systems[sys_tag].outputs.power.electrical[t_idx,0] = -val

                # distributors links
                for key, col in unknown_cols.items():
                    if key[0] == "link":

                        name_a = key[1]
                        name_b = key[2]
                        val    = float(x_solution[col,0])

                        dist_a = distributors[name_a]
                        dist_b = distributors[name_b]

                        # domain selection (same rule you already use everywhere else)
                        if isinstance(dist_a, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                            fld = "chemical"
                        else:
                            fld = "electrical"

                        # ----------------------------------------------------------
                        # A side (row had -1): flow leaves A → negative
                        # ----------------------------------------------------------
                        if conditions.energy.distributors[name_a].links[name_b].power[fld][t_idx,0] == 0.0:
                            conditions.energy.distributors[name_a].links[name_b].power[fld][t_idx,0] = +val

                        # ----------------------------------------------------------
                        # B side (row had +1): flow enters B → positive
                        # ----------------------------------------------------------
                        if conditions.energy.distributors[name_b].links[name_a].power[fld][t_idx,0] == 0.0:
                            conditions.energy.distributors[name_b].links[name_a].power[fld][t_idx,0] = -val

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