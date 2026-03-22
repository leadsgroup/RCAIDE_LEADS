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
        self.tag                                 = 'network' 
        self.hybrid_power_split_ratio            = None
        self.battery_fuel_cell_power_split_ratio = None        
        self.propulsors                          = Container() 
        self.converters                          = Container() 
        self.nacelles                            = Container()
        self.modulators                          = Container()
        self.distributors                        = Container()
        self.sources                             = Container()
        self.systems                             = Container()
         

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
        net_electrical_power  = 0. * state.ones_row(1)
        total_chemical_power    = 0. * state.ones_row(1)
        total_current           = 0. * state.ones_row(1)
 
        # ----------------------------------------------------------
        # Propulsors
        # ----------------------------------------------------------
        stored_results_flag  = False
        for propulsor in propulsors:
            if propulsor.active:
                if propulsor.identical_propulsors == False or stored_results_flag == False:
                    # -----------
                    # to remove 
                    state.conditions.energy.propulsors[propulsor.tag].outputs.power.electrical = propulsor.electrical_power_generation_split *  state.unknowns.network['electrical_power']*(1 - state.conditions.energy.hybrid_power_split_ratio)  
                    # -----------
                    inputs, outputs, stored_results_flag, stored_propulsor_tag = propulsor.compute_performance(state,network, center_of_gravity=center_of_gravity)
                else:
                    inputs, outputs = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag, center_of_gravity=center_of_gravity)

                if propulsor.reverse_thrust == True:
                    total_thrust = outputs.thrust * -1
                    total_moment = outputs.moment * -1

                total_thrust           += outputs.thrust
                total_moment           += outputs.moment
                total_propulsive_power += outputs.power.propulsive 
                total_current          += outputs.current 
                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)
                total_chemical_power   += inputs.power.chemical
                total_mdot             += inputs.mdot_fuel  
        
        # ----------------------------------------------------------
        # Systems
        # ----------------------------------------------------------
        for system in systems:
            if system.active: 
                inputs, outputs = system.compute_performance(state) 
                net_electrical_power += (outputs.power.electrical - inputs.power.electrical)
      
        for converter in converters:   
            if converter.active: 
                if converter.identical_propulsors == False or stored_results_flag == False:
                    state.conditions.energy.converters[converter.tag].outputs.power.electrical =  converter.electrical_power_generation_split *  state.unknowns.network['electrical_power']*(1 - state.conditions.energy.hybrid_power_split_ratio )  # NEED TO ASSIGN PRIOR
                    inputs, outputs, stored_results_flag, stored_conveter_tag = converter.compute_performance(state,network)
                else:
                    inputs, outputs = converter.reuse_stored_data(state,network,stored_conveter_tag=stored_conveter_tag)
  
                total_current          += outputs.current 
                net_electrical_power   -= inputs.power.electrical
                total_chemical_power   += inputs.power.chemical
                total_mdot             += inputs.mdot_fuel

        # ----------------------------------------------------------
        # Sources 
        # ----------------------------------------------------------        
        for source in sources: 
            if source.active:
                if issubclass(type(source),RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                    charing_power = 0
                    if state.conditions.energy.recharging:  
                        charing_power  =  (source.nominal_capacity * source.charging_c_rate* source.voltage)                       
                    inputs, outputs, _, _ = source.compute_performance(state,network) 
                    net_electrical_power   += (outputs.power.electrical - inputs.power.electrical) -charing_power    
            
                if issubclass(type(source),RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                    state.conditions.energy.sources[source.tag].outputs.power.chemical = total_chemical_power *  state.conditions.energy.sources[source.tag].power_split_ratio
                    inputs, outputs, _, _ = source.compute_performance(state,network)
                    
                    # pumps 
                    #net_electrical_power   -= inputs.power.electrical              
                            
        # ----------------------------------------------------------
        # Distributors 
        # ----------------------------------------------------------
        # loop through compoments and determine the power in OR out of a distributor, compute power poss, heat transfer
        

        # ----------------------------------------------------------
        # Modulatore  
        # ----------------------------------------------------------
        #  
        
                   
                   
                   
                   
                   
                   
                   
                   
                   
                   
                                                       

        ## ----------------------------------------------------------
        ## Build Power Balance System
        ## ----------------------------------------------------------

        #n_rows  = len(distributors)
        #n_cpts = state.numerics.number_of_control_points
        #A_matrix = np.zeros((n_cpts,n_rows,0))

        #distributor_tags = []
        #for dist in distributors:
            #distributor_tags.append(dist.tag) 

        #b_vector     = np.zeros((n_cpts,n_rows,1)) 
        #unknown_cols = {}   # maps a key -> column index

        

        ## distributor ↔ distributor links 
        #for distributor in distributors:
            #if distributor.assigned_distributors != None: 
                #for distributor_2_tag in distributor.assigned_distributors[0]: 
                    #for power_key in conditions.energy.distributors[distributor.tag].links[distributor_2_tag].power.keys():
                        #if distributor.domain == power_key: 
                            #if distributor.tag < distributor_2_tag: 
                                #row_a   = distributor_tags.index(distributor.tag)
                                #row_b   = distributor_tags.index(distributor_2_tag) 
                                #key = ("links", distributor.tag, distributor_2_tag, 'link', distributor.domain )
                                #if key not in unknown_cols:
                                    #unknown_cols[key] = len(unknown_cols) 
                                    #vector    =  np.zeros((n_cpts,n_rows,1))
                                    #vector[:,row_a,0] = -1.0
                                    #vector[:,row_b,0] = 1.0
                                    #A_matrix =  np.concatenate((A_matrix,vector), axis=2)

        ## ----------------------------------------------------------
        ## Solve Power Balance System
        ## ---------------------------------------------------------- 
        
        #x_solution = np.zeros((n_cpts,len(A_matrix[0, 0, :])))
        #for t_idx in range(n_cpts):   
            #x_solution_t, _, _, _ = np.linalg.lstsq(A_matrix[t_idx], b_vector[t_idx], rcond=None) 
            #x_solution[t_idx] = x_solution_t[:,0] 
 
        ## ----------------------------------------------------------
        ## Save solved unknowns back into conditions.energy
        ## ----------------------------------------------------------
        #for key, col in unknown_cols.items():
            #val = x_solution[:,col]
            
            ##print(key)
            #neg_sign        = val < 0.0
            #pos_sign        = val > 0.0
            #component_group = key[0]
            #component_tag   = key[1]             
            #distributor_tag = key[2]
            #direction       = key[3]
            #side            = key[4].split('_') 
            #power_type      = side[0]
             
            #if component_group == "modulators": 
                #eff = modulators[component_tag].efficiency
                #if np.all(conditions.energy[component_group][component_tag][direction].power[power_type][:,0] == 0.0):
                    #conditions.energy[component_group][component_tag][direction].power[power_type][neg_sign,0] = -val[neg_sign] 
                    #conditions.energy[component_group][component_tag][direction].power[power_type][pos_sign,0] = val[pos_sign] 
                #if np.all(conditions.energy[component_group][component_tag][direction].power[power_type][:,0] == 0.0):
                    #conditions.energy[component_group][component_tag][direction].power[power_type][neg_sign,0] = -val[neg_sign]  * eff 
                    #conditions.energy[component_group][component_tag][direction].power[power_type][pos_sign,0] = val[pos_sign] * eff  

            #elif component_group == "links": 
                #name_a = key[1]
                #name_b = key[2]  
                #conditions.energy.distributors[name_a].links[name_b].power[power_type][pos_sign,0] = val[pos_sign] 
                #conditions.energy.distributors[name_b].links[name_a].power[power_type][neg_sign,0] = -val[neg_sign]
            #elif component_group == "sources":
                #direction_1 = direction.split('_')[0]
                #direction_2 = direction.split('_')[1]
                
                #distributor =  distributors[distributor_tag]
                #conditions.energy[component_group][component_tag][direction_1].power[power_type][pos_sign,0] =  val[pos_sign] * distributor.efficiency
                #conditions.energy[component_group][component_tag][direction_2].power[power_type][neg_sign,0] = -val[neg_sign]  
            #else:    
                #conditions.energy[component_group][component_tag][direction].power[power_type][:,0] = val                        
                    
       
        # ----------------------------------------------------------
        ## Compute performance of distributors  
        ## ---------------------------------------------------------- 
        #for distributor in distributors:         
            #distributors.compute_performance(state,network)

        ## Step 4 : Battery Thermal Management Calculations                    
        #for coolant_line in coolant_lines: 
            #for heat_exchanger in coolant_line.heat_exchangers: 
                #heat_exchanger.compute_heat_exchanger_performance(state,coolant_line) 
            #for reservoir in coolant_line.reservoirs:   
                #reservoir.compute_reservior_coolant_temperature(state,coolant_line)

        # pack residuals 
        # state.residuals.network[ 'electrical_power'] = net_electrical_power
                
        # Final aggregation for system level performance 
        conditions.energy.total_force_vector       = total_thrust
        conditions.energy.total_moment_vector      = total_moment
        conditions.energy.power.outputs.propulsive = total_propulsive_power 
        conditions.weights.vehicle.mass_rate       = total_mdot  

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
                propulsor.unpack_unknowns(segment) 
            for source in network.sources:
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
            for propulsor_i, propulsor in enumerate(network.propulsors):    
                if propulsor.active:
                    propulsor =  network.propulsors[propulsor.tag]
                    propulsor.pack_residuals(segment) 
            for source in network.sources:
                source.pack_residuals(segment) 
            for modulator in network.modulators:
                modulator.pack_residuals(segment) 
            for distributor in network.distributors:
                distributor.pack_residuals(segment) 
            for system in network.systems:
                system.pack_residuals(segment) 
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
    
            for converter in network.converters: 
                converter.append_operating_conditions(segment)  

            for modulator in network.modulators: 
                modulator.append_operating_conditions(segment)  

            for source in  network.sources: 
                source.append_operating_conditions(segment)  

            for system in network.systems:
                system.append_operating_conditions(segment)             
    
            for distributor in network.distributors:
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

        self.evaluate(state,center_of_gravity)



# ----------------------------------------------------------------------
#  Handle Linking
# ----------------------------------------------------------------------
Network.Container = Container