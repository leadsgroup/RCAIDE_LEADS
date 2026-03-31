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
        self.hybrid_power_split_ratio            = 1.0
        self.battery_fuel_cell_power_split_ratio = 1.0        
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
        total_chemical_power    = 0. * state.ones_row(1) 
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
                    #################################
                    # TO REMOVE
                    state.conditions.energy.propulsors[propulsor.tag].outputs.power.electrical = propulsor.electrical_power_generation_split \
                        *  state.unknowns.network['electrical_power']*(1 - state.conditions.energy.hybrid_power_split_ratio)  
                    #################################
                    inputs, outputs, stored_results_flag, stored_propulsor_tag = propulsor.compute_performance(state,network,center_of_gravity=center_of_gravity)
                else:
                    inputs, outputs = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag, center_of_gravity=center_of_gravity)

                if propulsor.reverse_thrust == True:
                    total_thrust = outputs.thrust * -1
                    total_moment = outputs.moment * -1

                total_thrust           += outputs.thrust
                total_moment           += outputs.moment
                total_propulsive_power += outputs.power.propulsive  
                total_chemical_power   += inputs.power.chemical
                total_mdot             += state.conditions.energy.propulsors[propulsor.tag].fuel_mass_flow_rate  
                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)  # must handle tank integrated pump
                net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic)
        
        # ----------------------------------------------------------
        # Systems
        # ----------------------------------------------------------
        for system in systems:
            if system.active: 
                inputs, outputs,_,_= system.compute_performance(state)  
                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)
                net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic)
                
        stored_results_flag  = False
        for converter in converters:   
            if converter.active: 
                if converter.identical_converters == False or stored_results_flag == False:
                    state.conditions.energy.converters[converter.tag].outputs.power.electrical =  converter.electrical_power_generation_split * state.unknowns.network['electrical_power']*(1 - state.conditions.energy.hybrid_power_split_ratio )  # NEED TO ASSIGN PRIOR
                    converter.reverse_mode_computation = True
                    inputs, outputs, stored_results_flag, stored_conveter_tag = converter.compute_performance(state,network)
                else:
                    inputs, outputs = converter.reuse_stored_data(state,network,stored_conveter_tag=stored_conveter_tag) 
                total_chemical_power   += inputs.power.chemical
                total_mdot             += state.conditions.energy.propulsors[propulsor.tag].fuel_mass_flow_rate  

                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)
                net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic)                

        # ----------------------------------------------------------
        # Sources 
        # ----------------------------------------------------------   
        state.conditions.energy.outputs.power.chemical = total_chemical_power
        for source in sources: 
            if source.active:    
                inputs, outputs, _, _  = source.compute_performance(state,network)   
                net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)
                net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic) # thermal pump included                    
                           
                           
                           
                           
                           
                           
                           
                           
                           
                           
                           
                           
                           
                            
        # ----------------------------------------------------------
        # Power Balance 
        # ----------------------------------------------------------
        # loop through compoments and determine the power in OR out of a distributor, compute power poss, heat transfer
        

                                                       

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
        # Distributors 
        # ----------------------------------------------------------
        #for distributor in distributors:
            #if distributor.active:    
                #for propulsor in  network.propulsors:
                    #if distributor.tag in propulsor.assigned_distributors[0]:
                        #state.conditions.energy.distributors[distributor.tag].outputs.power[distributor.domain] +=  state.conditions.energy.propulsors[propulsor.tag].inputs.power[distributor.domain]
                        
                ## this computes the input power (output power is suppled to the components of various forms )
                #inputs, outputs, _, _  = distributor.compute_performance(state,network)
                
                ## determine system losses (should be negative )
                #net_electrical_power   += (outputs.power.electrical - inputs.power.electrical)  
                #net_thermal_power      += (outputs.power.thermal - inputs.power.thermal)
                #net_hydraulic_power    += (outputs.power.hydraulic - inputs.power.hydraulic)  # line integrated pump 
                
        # Final aggregation for system level performance 
        conditions.energy.total_force_vector       = total_thrust
        conditions.energy.total_moment_vector      = total_moment
        conditions.energy.outputs.power.propulsive = total_propulsive_power 
        conditions.weights.vehicle.mass_rate       = total_mdot  
        conditions.energy.net_electrical_power     = net_electrical_power 
        conditions.energy.net_thermal_power        = net_thermal_power 
        conditions.energy.net_hydraulic_power      = net_hydraulic_power 

        return
     
       
    def append_segment_conditions(self,segment): 
 
        segment.conditions.energy.inputs.power.propulsive[:,0]    = 0
        segment.conditions.energy.inputs.power.mechanical[:,0]     = 0
        segment.conditions.energy.inputs.power.electrical[:,0]     = 0
        segment.conditions.energy.inputs.power.chemical[:,0]       = 0
        segment.conditions.energy.inputs.power.pneumatic[:,0]      = 0
        segment.conditions.energy.inputs.power.hydraulic[:,0]      = 0
        segment.conditions.energy.inputs.power.thermal[:,0]        = 0 
        segment.conditions.energy.outputs.power.propulsive[:,0]    = 0
        segment.conditions.energy.outputs.power.mechanical[:,0]    = 0
        segment.conditions.energy.outputs.power.electrical[:,0]    = 0
        segment.conditions.energy.outputs.power.chemical[:,0]      = 0
        segment.conditions.energy.outputs.power.pneumatic[:,0]     = 0
        segment.conditions.energy.outputs.power.hydraulic[:,0]     = 0
        segment.conditions.energy.outputs.power.thermal[:,0]       = 0        
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