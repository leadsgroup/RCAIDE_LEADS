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
        conditions    = state.conditions
        propulsors    = network.propulsors
        converters    = network.converters  
        distributors  = network.distributors
        modulators    = network.modulators
        sources       = network.sources
        systems       = network.systems

        total_thrust            = 0. * state.ones_row(3)
        total_moment            = 0. * state.ones_row(3)
        total_mdot              = 0. * state.ones_row(1)
        total_propulsive_power  = 0. * state.ones_row(1)
        
        ''' MAJOR ASSUMTION
        
        number of unknowns is the number of distributors 
        '''
        # ----------------------------------------------------------
        # Propulsors
        # ----------------------------------------------------------
        stored_results_flag  = False
        for propulsor in propulsors:
            if propulsor.active:
                if propulsor.identical_propulsors == False or stored_results_flag == False:
                    inputs, outputs, stored_results_flag, stored_propulsor_tag = propulsor.compute_performance(state, center_of_gravity=center_of_gravity)
                else:
                    inputs, outputs = propulsor.reuse_stored_data(state, network, stored_propulsor_tag=stored_propulsor_tag, center_of_gravity=center_of_gravity)

                if propulsor.reverse_thrust == True:
                    total_thrust = outputs.thrust * -1
                    total_moment = outputs.moment * -1

                total_thrust           += outputs.thrust
                total_moment           += outputs.moment
                total_propulsive_power += outputs.power.propulsive 
                total_mdot             += inputs.mdot_fuel  
        
        # ----------------------------------------------------------
        # Systems
        # ----------------------------------------------------------
        for system in systems:
            _, _ = system.compute_performance(state)

        # ----------------------------------------------------------
        # Build Power Balance System
        # ----------------------------------------------------------

        n_rows  = len(distributors)
        n_cpts = state.numerics.number_of_control_points
        A_matrix = np.zeros((n_cpts,n_rows,0))

        distributor_tags = []
        for dist in distributors:
            distributor_tags.append(dist.tag) 

        b_vector     = np.zeros((n_cpts,n_rows,1)) 
        unknown_cols = {}   # maps a key -> column index

        # propulsors
        for propulsor in propulsors:
            if propulsor.assigned_distributors != None: 
                for distributor_tag in propulsor.assigned_distributors[0]: 
                    row_index   = distributor_tags.index(distributor_tag) 
                    distributor = distributors[distributor_tag]
                    
                    
                    # THIS LOOP IS MORE GENERIC THAN MATTEOs
                    # loop through outputs
                    #for output_power_key in conditions.energy.propulsors[propulsor.tag].outputs.power.keys(): 
                        #val = conditions.energy.propulsors[propulsor.tag].outputs.power[output_power_key][:,0] 
                        #if np.all(val == 0.0):
                            #key = ("propulsor", propulsor.tag, distributor_tag, output_power_key+"_out")
                            #if key not in unknown_cols:
                                #unknown_cols[key] = len(unknown_cols)  
                                #vector    =  np.zeros((n_cpts,n_rows,1))
                                #vector[:,row_index,0] = 1
                                #A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            #else:
                                #A_matrix[:,row_index,unknown_cols[key]] += 1 
                        #else:
                            #b_vector[:,row_index,0] += val
                            
                    ## loop through inputs         
                    #for input_power_key in conditions.energy.propulsors[propulsor.tag].inputs.power.keys(): 
                        #val = conditions.energy.propulsors[propulsor.tag].inputs.power[input_power_key][:,0] 
                        #if np.all(val == 0.0):
                            #key = ("propulsor", propulsor.tag, distributor_tag, input_power_key+"_in")
                            #if key not in unknown_cols:
                                #unknown_cols[key] = len(unknown_cols)  
                                #vector    =  np.zeros((n_cpts,n_rows,1))
                                #vector[:,row_index,0] = -1
                                #A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            #else:
                                #A_matrix[:,row_index,unknown_cols[key]] -= 1 
                        #else:
                            #b_vector[:,row_index,0] -= val                            
                            
                    
                    # loop through inputs
                    
                    if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                        val = conditions.energy.propulsors[propulsor.tag].outputs.power.electrical[:,0]
                        if np.all(val == 0.0):
                            key = ("propulsors", propulsor.tag, distributor_tag, "electrical_out")
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols)  
                                vector    =  np.zeros((n_cpts,n_rows,1))
                                vector[:,row_index,0] = 1
                                A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            else:
                                A_matrix[:,row_index,unknown_cols[key]] += 1
                        else:
                            b_vector[:,row_index,0] += val
                            
                            
                    elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                        val = conditions.energy.propulsors[propulsor.tag].inputs.power.chemical[:,0]
                        if np.all(val == 0.0):
                            key = ("propulsors", propulsor.tag, distributor_tag, "chemical_in")
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols)
                                vector    =  np.zeros((n_cpts,n_rows,1))
                                vector[:,row_index,0] = -1
                                A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            else:
                                A_matrix[:,row_index,unknown_cols[key]] -= 1
                        else:
                            b_vector[:,row_index,0] -= val                         

        # converters (non-propulsive) 
        for converter in converters:
            if converter.propulsor_integrated == False:
                if converter.assigned_distributors != None: 
                    for distributor_tag in converter.assigned_distributors[0]: 
                        row_index   = distributor_tags.index(distributor_tag) 
                        distributor = distributors[distributor_tag]
                         
                        # loop through outputs
                        for output_power_key in conditions.energy.converters[converter.tag].outputs.power.keys(): 
                            val = conditions.energy.converters[converter.tag].outputs.power[output_power_key][:,0] 
                            if np.all(val == 0.0):
                                key = ("converters", converter.tag, distributor_tag, output_power_key+"_out")
                                if key not in unknown_cols:
                                    unknown_cols[key] = len(unknown_cols)  
                                    vector    =  np.zeros((n_cpts,n_rows,1))
                                    vector[:,row_index,0] = 1
                                    A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                                else:
                                    A_matrix[:,row_index,unknown_cols[key]] += 1 
                            else:
                                b_vector[:,row_index,0] += val
                                
                        # loop through inputs          
                        for input_power_key in conditions.energy.converters[converter.tag].inputs.power.keys(): 
                            val = conditions.energy.converters[converter.tag].inputs.power[input_power_key][:,0] 
                            if np.all(val == 0.0):
                                key = ("converters", converter.tag, distributor_tag, input_power_key+"_in")
                                if key not in unknown_cols:
                                    unknown_cols[key] = len(unknown_cols)  
                                    vector    =  np.zeros((n_cpts,n_rows,1))
                                    vector[:,row_index,0] = -1
                                    A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                                else:
                                    A_matrix[:,row_index,unknown_cols[key]] -= 1 
                            else:
                                b_vector[:,row_index,0] -= val 
                    
            
        # modulators
        for modulator in modulators:
            if modulator.assigned_distributors != None: 
                for distributor_tag in modulator.assigned_distributors[0]: 
                    row_index = distributor_tags.index(distributor_tag) 
                    distributor = distributors[distributor_tag]
                    

                    #for input_power_key in conditions.energy.modulators[modulator.tag].inputs.power.keys():
                    input_power_key = 'electrical'
                    eff  = modulator.efficiency.electrical
                    
                    
                    # these may actually be all zero there so might be able to remove 
                    vin  = conditions.energy.modulators[modulator.tag].inputs.power[input_power_key][:,0]   
                    vout = conditions.energy.modulators[modulator.tag].outputs.power[input_power_key][:,0]  
                    if np.all(vin != 0.0) and np.all(vout == 0.0): # NEED TO MAKE USE NP.ALL ? 
                        conditions.energy.modulators[modulator.tag].outputs.power[input_power_key][:,0] = vin * eff
                        vout = conditions.energy.modulators[modulator.tag].outputs.power[input_power_key][:,0]
                    elif np.all(vout != 0.0) and np.all(vin == 0.0): # NEED TO MAKE USE NP.ALL ? 
                        conditions.energy.modulators[modulator.tag].inputs.power[input_power_key][:,0]  = vout / eff
                        vin  = conditions.energy.modulators[modulator.tag].inputs.power[input_power_key][:,0]
                    elif np.all(vin != 0.0) and np.all(vout != 0.0): # NEED TO MAKE USE NP.ALL ? 
                        conditions.energy.modulators[modulator.tag].outputs.power[input_power_key][:,0] = vin * eff
                        vout = conditions.energy.modulators[modulator.tag].outputs.power[input_power_key][:,0]
                        
                    #key = ("modulators", modulator.tag, distributor_tag, input_power_key+"_in")
                    key = ("modulators", modulator.tag, "power", input_power_key+"_in")
                    if distributor.type == 'AC':
                        if np.all(vin == 0.0):
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols) 
                                vector    =  np.zeros((n_cpts,n_rows,1))
                                vector[:,row_index,0] = -1
                                A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            else:
                                A_matrix[:,row_index,unknown_cols[key]] -= 1
                        else:
                            b_vector[:,row_index,0] -= vin 
                            
                    elif distributor.type == 'DC':
                        # Mirror link behavior: always add +eff to DC row when the DC side is unknown
                        if np.all(vout == 0.0):
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols)
                                vector    =  np.zeros((n_cpts,n_rows,1))
                                vector[:,row_index,0] = +eff
                                A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            else:
                                A_matrix[:,row_index,unknown_cols[key]] =+eff
                        else:
                            b_vector[:,row_index,0] += vout  
            
        # sources 
        for source in sources:
            if source.assigned_distributors != None: 
                for distributor_tag in source.assigned_distributors[0]: 
                    row_index = distributor_tags.index(distributor_tag) 
                    distributor = distributors[distributor_tag]

                    # loop through outputs
                    #for output_power_key in conditions.energy.converters[converter.tag].outputs.power.keys():                     
                    #val = conditions.energy.sources[source.tag].outputs.power[output_power_key][:,0] # so you dont know how much power the battery is producing 
                    #if np.all(val == 0.0):# NEED TO MAKE USE NP.ALL ? 
                        #key = ("source", source.tag, distributor_tag, output_power_key + "_out")
                        #if key not in unknown_cols:
                            #unknown_cols[key] = len(unknown_cols) 
                            #vector    =  np.zeros((n_cpts,n_rows,1))
                            #vector[:,row_index,0] = 1
                            #A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                        #else:
                            #A_matrix[:,row_index,unknown_cols[key]] += 1                                
    
                        ## TO REMOVE ------                                
                        #for t_idx in range(n_cpts):
                            #triplets.append((t_idx,row_index, unknown_cols[key], +1.0)) 
                        ## TO REMOVE ------
                         
                    #else: # if you do know how much the battery is producing, assign it as a known value on the vector 
                        #b_vector[:,row_index,0] += val
                        
                        
                        
                    if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                        val = conditions.energy.sources[source.tag].outputs.power.electrical[:,0] # so you dont know how much power the battery is producing 
                        if np.all(val == 0.0):# NEED TO MAKE USE NP.ALL ? 
                            key = ("sources", source.tag, distributor_tag,  "electrical_out")
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols) 
                                vector    =  np.zeros((n_cpts,n_rows,1))
                                vector[:,row_index,0] = 1
                                A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            else:
                                A_matrix[:,row_index,unknown_cols[key]] += 1                       
                             
                        else: # if you do know how much the battery is producing, assign it as a known value on the vector 
                            b_vector[:,row_index,0] += val
                    elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                        val = conditions.energy.sources[source.tag].outputs.power.chemical[:,0] # so you dont know how much power the battery is producing 
                        if np.all(val == 0.0):# NEED TO MAKE USE NP.ALL ? 
                            key = ("sources", source.tag, distributor_tag,  "chemical_out")
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols) 
                                vector    =  np.zeros((n_cpts,n_rows,1))
                                vector[:,row_index,0] = 1
                                A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            else:
                                A_matrix[:,row_index,unknown_cols[key]] += 1  
                        else: # if you do know how much the battery is producing, assign it as a known value on the vector 
                            b_vector[:,row_index,0] += val
                                 
        # systems 
        for system in systems:
            if propulsor.assigned_distributors != None:
                for distributor_tag in propulsor.assigned_distributors[0]: 
                    row_index = distributor_tags.index(distributor_tag) 

                    #for input_power_key in conditions.energy.systems[system.tag].inputs.power.keys(): 
                        #val = conditions.energy.systems[system.tag].inputs.power[input_power_key][:,0] # so you dont know how much power the battery is producing 
                        #if np.all(val == 0.0):# NEED TO MAKE USE NP.ALL ? 
                            #key = ("system", source.tag, distributor_tag, input_power_key + "_in")
                            #if key not in unknown_cols:
                                #unknown_cols[key] = len(unknown_cols) 
                                #vector    =  np.zeros((n_cpts,n_rows,1))
                                #vector[:,row_index,0] = -1
                                #A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                            #else:
                                #A_matrix[:,row_index,unknown_cols[key]] -= 1            
                        #else:  
                            #b_vector[:,row_index,0] -= val
                            
                            
                    val =  conditions.energy.systems[system.tag].inputs.power.electrical[:,0] 
                    if np.all(val == 0.0):# NEED TO MAKE USE NP.ALL ? 
                        key = ("systems", source.tag, distributor_tag, "electrical_in")
                        if key not in unknown_cols:
                            unknown_cols[key] = len(unknown_cols) 
                            vector    =  np.zeros((n_cpts,n_rows,1))
                            vector[:,row_index,0] = -1
                            A_matrix =  np.concatenate((A_matrix,vector), axis=2) 
                        else:
                            A_matrix[:,row_index,unknown_cols[key]] -= 1          
                    else:  
                        b_vector[:,row_index,0] -= val                                       

        # distributor ↔ distributor links 
        for distributor in distributors:
            if distributor.assigned_distributors != None: 
                for distributor_2_tag in distributor.assigned_distributors[0]: 
                    for input_power_key in conditions.energy.distributors[distributor.tag].links[distributor_2_tag].power.keys():                    
                        if distributor.tag < distributor_2_tag: 
                            row_a   = distributor_tags.index(distributor.tag)
                            row_b   = distributor_tags.index(distributor_2_tag) 
                            key = ("links", distributor.tag, distributor_2_tag,input_power_key )
                            if key not in unknown_cols:
                                unknown_cols[key] = len(unknown_cols) 
                                vector    =  np.zeros((n_cpts,n_rows,1))
                                vector[:,row_a,0] = -1.0
                                vector[:,row_b,0] = 1.0
                                A_matrix =  np.concatenate((A_matrix,vector), axis=2)    
                            else:
                                A_matrix[:,row_a, col] -= 1.0
                                A_matrix[:,row_b, col] += 1.0


        # ----------------------------------------------------------
        # Solve Power Balance System
        # ---------------------------------------------------------- 
        
        x_solution = np.zeros((n_cpts,len(A_matrix[0, 0, :])))
        for t_idx in range(n_cpts):  # LOOP CAN BE REMOVED 
            x_solution_t, _, _, _ = np.linalg.lstsq(A_matrix[t_idx], b_vector[t_idx], rcond=None) 
            x_solution[t_idx] = x_solution_t[:,0] 
 
        # ----------------------------------------------------------
        # Save solved unknowns back into conditions.energy
        # ----------------------------------------------------------
        for key, col in unknown_cols.items():
            val = x_solution[:,col]
            
            #print(key)
            neg_sign        = val < 0.0
            pos_sign        = val > 0.0
            component_group = key[0]
            component_tag   = key[1]             
            distributor_tag = key[2]
            side            = key[3].split('_') 
            power_type      = side[0]
             
            if component_group == "modulators": 
                eff = modulators[component_tag].efficiency.electrical  
                if np.all(conditions.energy[component_group][component_tag].inputs.power[power_type][:,0] == 0.0):
                    conditions.energy[component_group][component_tag].inputs.power[power_type][neg_sign,0] = -val[neg_sign] 
                    conditions.energy[component_group][component_tag].inputs.power[power_type][pos_sign,0] = val[pos_sign] 
                if np.all(conditions.energy[component_group][component_tag].outputs.power[power_type][:,0] == 0.0):
                    conditions.energy[component_group][component_tag].outputs.power[power_type][neg_sign,0] = -val[neg_sign]  * eff 
                    conditions.energy[component_group][component_tag].outputs.power[power_type][pos_sign,0] = val[pos_sign] * eff  

            elif component_group == "links": 
                name_a = key[1]
                name_b = key[2]  
                conditions.energy.distributors[name_a].links[name_b].power[power_type][pos_sign,0] = val[pos_sign] 
                conditions.energy.distributors[name_b].links[name_a].power[power_type][neg_sign,0] = -val[neg_sign]
            else: 
                if np.all(conditions.energy[component_group][component_tag].outputs.power[power_type][:,0] == 0.0):   
                    conditions.energy[component_group][component_tag].outputs.power[power_type][neg_sign,0] = -val[neg_sign] 
                if np.all(conditions.energy[component_group][component_tag].inputs.power[power_type][:,0] == 0.0):  
                    conditions.energy[component_group][component_tag].inputs.power[power_type][pos_sign,0] = val[pos_sign]
                    
        # Final aggregation
        conditions.energy.total_force_vector     = total_thrust
        conditions.energy.total_moment_vector    = total_moment
        conditions.energy.total_propulsive_power = total_propulsive_power
        conditions.weights.vehicle.mass_rate     = total_mdot  

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
            for propulsor in network.propulsors:
                propulsor.unpack_propulsor_unknowns(segment, network) 
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
            for propulsor_i, propulsor in enumerate(network.propulsors):    
                if propulsor.active:
                    propulsor =  network.propulsors[propulsor.tag]
                    propulsor.pack_propulsor_residuals(segment, network) 
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
        
        for network in segment.analyses.vehicle.networks:
            
            for propulsor in network.propulsors: 
                propulsor.append_operating_conditions(segment)  
                propulsor.append_propulsor_unknowns_and_residuals(segment, network)   
    
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