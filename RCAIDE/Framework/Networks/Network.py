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
    """ Generalized Hybrid Energy Network (powertrain) Class capable of creating all derivatives of hybrid
    networks, the conventional fuel network and the all-electric network.
    
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
        self.coolant_lines                = Container()
        self.identical_propulsors         = True 
        self.reverse_thrust               = False
        self.wing_mounted                 = True   
        self.system_voltage               = None  
        
    # linking the different network components
    def evaluate(network,state,center_of_gravity):
        """ Computes the performance of the network
        """  
        # unpack   
        conditions           = state.conditions 
        propulsors           = network.propulsors  
        converters           = network.converters  
        distributors         = network.distributors
        sources              = network.sources     
        systems              = network.systems
        coolant_lines        = network.coolant_lines
        total_thrust         = 0. * state.ones_row(3) 
        total_mech_power     = 0. * state.ones_row(1) 
        total_elec_power     = 0. * state.ones_row(1) 
        total_moment         = 0. * state.ones_row(3)  
        total_mdot           = 0. * state.ones_row(1)   
        reverse_thrust       = network.reverse_thrust 
    
        # ----------------------------------------------------------       
        # Section 1.0 Propulsor Performance 
        # ----------------------------------------------------------
        for distributor in distributors:
            if isinstance(distributor,RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                systems              = distributor.systems 
        
                # Avionics Power Consumtion 
                compute_systems_power_draw(systems,distributor,conditions) 
        
                # Bus Voltage 
                bus_voltage = distributor.voltage * state.ones_row(1)       
        
                if conditions.energy.recharging:             
                    distributor.charging_current         = distributor.nominal_capacity * distributor.charging_c_rate 
                    charging_power               = (distributor.charging_current*bus_voltage*distributor.power_split_ratio) 
                    conditions.energy.busses[distributor.tag].power_draw   -= charging_power/distributor.efficiency
                    conditions.energy.busses[distributor.tag].current_draw  = -conditions.energy.busses[distributor.tag].power_draw/distributor.voltage
    
            for propulsor_group in distributor.assigned_propulsors:
                stored_results_flag  = False
                stored_propulsor_tag = None 
                for propulsor_tag in propulsor_group:
                    propulsor            = propulsors[propulsor_tag]
                    if propulsor.active and distributor.active:   
                        if network.identical_propulsors == False:
                            # run analysis  
                            T,M,P,P_elec,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state, center_of_gravity= center_of_gravity)
                        else:             
                            if stored_results_flag == False: 
                                # run propulsor analysis 
                                T,M,P,P_elec,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state, network, center_of_gravity= center_of_gravity)
                            else:
                                # use previous propulsor results 
                                T,M,P,P_elec = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag,center_of_gravity= center_of_gravity)
        
                        total_thrust      += T   
                        total_moment      += M   
                        total_mech_power  += P 
                        total_elec_power  += P_elec   
         
                        if isinstance(distributor,RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                            conditions.energy.fuel_lines[distributor.tag].fuel_mass_flow_rate += conditions.energy.propulsors[propulsor.tag].fuel_mass_flow_rate
                        
                        if isinstance(distributor,RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                            conditions.energy.busses[distributor.tag].power_draw         += (P_elec) * distributor.power_split_ratio /distributor.efficiency

                if isinstance(distributor,RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):   
            
                    conditions.energy.busses[distributor.tag].power_draw         += state.conditions.energy.busses[distributor.tag].regenerative_power*bus_voltage* distributor.power_split_ratio  /distributor.efficiency   
                    conditions.energy.busses[distributor.tag].current_draw       = conditions.energy.busses[distributor.tag].power_draw/bus_voltage  
                
        # ------------------------------------------------------------------------------------------------------------------- 
        # Section 2.0 Converters
        # -------------------------------------------------------------------------------------------------------------------
        ## 2.1 Fuel Converters 
        #for fuel_line in fuel_lines: 
            #if fuel_line.active: 
                #for converter_group in fuel_line.assigned_converters:
                    #stored_conveter_tag = False
                    #for converter_tag in converter_group:
                        #converter =  converters[converter_tag]
                        #if converter.active:
                            #converter.inverse_calculation = True 
                            #if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator): 
                                #if stored_conveter_tag is False:
                                    #generator             = converter.generator   
                                    #state.conditions.energy.converters[generator.tag].outputs.power  =  total_elec_power*(1 - state.conditions.energy.hybrid_power_split_ratio ) 
                                    #P_mech, P_elec, stored_results_flag,stored_conveter_tag          = converter.compute_performance(state,fuel_line,bus)  
                                    #conditions.energy.busses[bus.tag].power_draw                    -= P_elec/bus.efficiency
                                    #conditions.energy.fuel_lines[fuel_line.tag].fuel_mass_flow_rate += conditions.energy.converters[converter.tag].fuel_mass_flow_rate   
                                #else:
                                    #generator             = converter.generator   
                                    #state.conditions.energy.converters[generator.tag].outputs.power  =  total_elec_power*(1 - state.conditions.energy.hybrid_power_split_ratio ) 
                                    #P_mech, P_elec                                                   = converter.reuse_stored_data(state,network,stored_conveter_tag,fuel_line,bus)  
                                    #conditions.energy.busses[bus.tag].power_draw                     -= P_elec/bus.efficiency
                                    #conditions.energy.fuel_lines[fuel_line.tag].fuel_mass_flow_rate  += conditions.energy.converters[converter.tag].fuel_mass_flow_rate   

                            #if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.Turboshaft):   
                                #state.conditions.energy.converters[converter.tag].power     = total_mech_power*(1 - state.conditions.energy.hybrid_power_split_ratio )   
                                #P_mech, P_elec,stored_results_flag,stored_propulsor_tag     = converter.compute_performance(state)   
                                #conditions.energy.fuel_lines[fuel_line.tag].fuel_mass_flow_rate  += conditions.energy.converters[converter.tag].fuel_mass_flow_rate  
                    
        ## 2.1 Electric Converters                            
        #for bus in busses: 
            #if bus.active == True:         
                #for converter_group in bus.assigned_converters:
                    #for converter_tag in converter_group:
                        #converter =  converters[converter_tag]
                        #if converter.active: 
                            #converter.inverse_calculation = True
                            #if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.DC_Motor) or isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor):  
                                #compute_motor_performance(converter,conditions)
                                #conditions.energy.busses[bus.tag].power_draw   += conditions.energy.converters[converter.tag].inputs.power/bus.efficiency
                                #conditions.energy.busses[bus.tag].current_draw  = conditions.energy.busses[bus.tag].power_draw/bus.voltage                            
                                
                            #if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.DC_Generator) or isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.PMSM_Generator):                              
                                #compute_generator_performance(converter,conditions) 
                                #conditions.energy.busses[bus.tag].power_draw   -= conditions.energy.converters[converter.tag].outputs.power/bus.efficiency
                                #conditions.energy.busses[bus.tag].current_draw  = conditions.energy.busses[bus.tag].power_draw/bus.voltage                            
        
                       
         
        # ----------------------------------------------------------        
        # Section 3.0 Sources
        # ----------------------------------------------------------

        time               = state.conditions.frames.inertial.time[:,0] 
        delta_t            = np.diff(time)
        
        for distributor in distributors:
            for source_tag in distributor.assigned_sources: 
                source =  network.sources[source_tag[0]]
                if issubclass(type(source),RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):    
                    # Determine mass flow from each tank 
                    source.compute_tank_properties(state,distributor)   
        
                    # Update total mass flow of system   
                    total_mdot  += conditions.energy.fuel_lines[distributor.tag].fuel_mass_flow_rate
                
                if issubclass(type(source),RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module):   
                    # 3.2 Electric Sources 
                    for t_idx in range(state.numerics.number_of_control_points):   
                
                        stored_results_flag       = False
                        stored_battery_cell_tag   = None
                    
                        if distributor.identical_battery_modules == False and stored_results_flag == False: 
                            # run analysis  
                            stored_results_flag, stored_battery_cell_tag =  source.energy_calc(state,distributor,coolant_lines, t_idx, delta_t)
                        else:             
                            # use previous battery results 
                            source.reuse_stored_data(state,distributor,stored_results_flag, stored_battery_cell_tag)
                        # Step 3: Compute bus properties          
                        distributor.compute_distributor_conditions(state,t_idx,delta_t)

        # ------------------------------------------------------------------------------------------------------------------- 
        # Section 4.0  Thermal Management
        # -------------------------------------------------------------------------------------------------------------------        
        for t_idx in range(state.numerics.number_of_control_points):        
            for coolant_line in network.coolant_lines:
                if t_idx != state.numerics.number_of_control_points-1: 
                    for heat_exchanger in coolant_line.heat_exchangers: 
                        heat_exchanger.compute_heat_exchanger_performance(state,distributor,coolant_line,delta_t[t_idx],t_idx) 
                    for reservoir in coolant_line.reservoirs:   
                        reservoir.compute_reservior_coolant_temperature(state,coolant_line,delta_t[t_idx],t_idx)
                                 
        if reverse_thrust ==  True:
            total_thrust =  total_thrust * -1    
            total_moment =  total_moment * -1                        
        conditions.energy.thrust_force_vector  = total_thrust
        conditions.energy.power                = total_mech_power 
        conditions.energy.thrust_moment_vector = total_moment 
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
            # Fuel unknowns 
            for distributor_i, distributor in enumerate(network.distributors):
                if distributor.active:
                    for propulsor_group in  distributor.assigned_propulsors:
                        propulsor = network.propulsors[propulsor_group[0]]
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
            for distributor_i, distributor in enumerate(network.distributors):    
                if distributor.active:
                    for propulsor_group in  distributor.assigned_propulsors:
                        propulsor =  network.propulsors[propulsor_group[0]]
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
    
            for converter in network.converters: 
                converter.append_operating_conditions(segment)  

            for modulator in network.modulators: 
                modulator.append_operating_conditions(segment)                
    
            for distributor_i, distributor in enumerate(network.distributors):
                
                distributor.append_operating_conditions(segment)              
                
                # Assign network-specific  residuals, unknowns and results data structures 
                if distributor.active:
                    for propulsor_group in  distributor.assigned_propulsors:
                        propulsor =  network.propulsors[propulsor_group[0]]
                        propulsor.append_propulsor_unknowns_and_residuals(segment)

                    for converter_group in  distributor.assigned_converters:
                        propulsor =  network.propulsors[propulsor_group[0]]
                        propulsor.append_propulsor_unknowns_and_residuals(segment)

                    for source in  network.sources: 
                        source.append_operating_conditions(segment, distributor) 
                                                                    
            for coolant_line_i, coolant_line in enumerate(network.coolant_lines):  
                # ------------------------------------------------------------------------------------------------------            
                # Create coolant_lines results data structure  
                # ------------------------------------------------------------------------------------------------------
                segment.state.conditions.energy.coolant_lines[coolant_line.tag] = RCAIDE.Framework.Mission.Common.Conditions()        
                
                # ------------------------------------------------------------------------------------------------------
                # Assign network-specific  residuals, unknowns and results data structures
                # ------------------------------------------------------------------------------------------------------       
                for battery_module in coolant_line.battery_modules: 
                    for btms in battery_module:
                        btms.append_operating_conditions(segment,coolant_line)
                        
                for heat_exchanger in coolant_line.heat_exchangers: 
                    heat_exchanger.append_operating_conditions(segment, coolant_line)
                        
                for reservoir in coolant_line.reservoirs: 
                    reservoir.append_operating_conditions(segment, coolant_line)                           
    
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