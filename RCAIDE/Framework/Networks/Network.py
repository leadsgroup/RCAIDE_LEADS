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
from RCAIDE.Library.Methods.Powertrain.Systems.compute_avionics_power_draw                import compute_avionics_power_draw
from RCAIDE.Library.Methods.Powertrain.Systems.compute_payload_power_draw                 import compute_payload_power_draw
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
    cryogenic tanks and batteries are also updates. Propulsor groups can be "active" or "inactive" to simulate
    engine out conditions. Energy consumtion from payload and avionics is also modeled 
    
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
        self.busses                       = Container()
        self.coolant_lines                = Container()
        self.fuel_lines                   = Container()
        self.converters                   = Container()
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
        busses               = network.busses 
        fuel_lines           = network.fuel_lines 
        coolant_lines        = network.coolant_lines
        converters           = network.converters 
        total_thrust         = 0. * state.ones_row(3) 
        total_mech_power     = 0. * state.ones_row(1) 
        total_elec_power     = 0. * state.ones_row(1) 
        total_moment         = 0. * state.ones_row(3)  
        fuel_mdot            = 0. * state.ones_row(1)
        total_mdot           = 0. * state.ones_row(1)
        cryogen_mdot         = 0. * state.ones_row(1)  
        reverse_thrust       = network.reverse_thrust 
    
        # ----------------------------------------------------------       
        # Section 1.0 Propulsor Performance 
        # ----------------------------------------------------------
        # 1.1 Fuel Propulsors  
        for fuel_line in fuel_lines: 
            for propulsor_group in fuel_line.assigned_propulsors:
                stored_results_flag  = False
                stored_propulsor_tag = None 
                for propulsor_tag in propulsor_group:
                    propulsor            = network.propulsors[propulsor_tag]
                    if propulsor.active and fuel_line.active:   
                        if network.identical_propulsors == False:
                            # run analysis  
                            T,M,P,P_elec,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state, center_of_gravity= center_of_gravity)
                        else:             
                            if stored_results_flag == False: 
                                # run propulsor analysis 
                                T,M,P,P_elec,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state,center_of_gravity= center_of_gravity)
                            else:
                                # use previous propulsor results 
                                T,M,P,P_elec = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag,center_of_gravity= center_of_gravity)
        
                        total_thrust      += T   
                        total_moment      += M   
                        total_mech_power  += P   
        
                        # compute total mass flow rate 
                        fuel_mdot     += conditions.energy.propulsors[propulsor.tag].fuel_flow_rate
                
        # 1.2 Electric Propulsors         
        for bus in busses:           
            bus_conditions       = state.conditions.energy[bus.tag] 
            avionics             = bus.avionics
            payload              = bus.payload  
    
            # Avionics Power Consumtion 
            compute_avionics_power_draw(avionics,bus,conditions)
    
            # Payload Power 
            compute_payload_power_draw(payload,bus,conditions)
    
            # Bus Voltage 
            bus_voltage = bus.voltage * state.ones_row(1)       
    
            if conditions.energy.recharging:             
                bus.charging_current         = bus.nominal_capacity * bus.charging_c_rate 
                charging_power               = (bus.charging_current*bus_voltage*bus.power_split_ratio) 
                bus_conditions.power_draw   -= charging_power/bus.efficiency
                bus_conditions.current_draw  = -bus_conditions.power_draw/bus.voltage
    
            else:
                for propulsor_group in bus.assigned_propulsors:
                    stored_results_flag  = False
                    stored_propulsor_tag = None   
                    for propulsor_tag in propulsor_group:
                        propulsor =  network.propulsors[propulsor_tag]
                        if propulsor.active and bus.active:       
                            if network.identical_propulsors == False:
                                # run analysis  
                                T,M,P_mech,P_elec,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state,center_of_gravity= center_of_gravity)
                            else:             
                                if stored_results_flag == False: 
                                    # run propulsor analysis 
                                    T,M,P_mech,P_elec, stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state,center_of_gravity= center_of_gravity)
                                else:
                                    # use previous propulsor results 
                                    T,M,P_mech,P_elec  = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag,center_of_gravity=center_of_gravity)
    
                            total_thrust      += T   
                            total_moment      += M   
                            total_mech_power  += P_mech 
                            total_elec_power  += P_elec 
    
                # compute power from each componemnt 
                bus_conditions.power_draw        += (total_elec_power- state.conditions.energy[bus.tag].regenerative_power*bus_voltage ) * bus.power_split_ratio  /bus.efficiency   
                bus_conditions.current_draw       = bus_conditions.power_draw/bus_voltage  
             
        # ------------------------------------------------------------------------------------------------------------------- 
        # Section 2.0 Converters
        # -------------------------------------------------------------------------------------------------------------------  
        # 2.1 Fuel Converters         
        for fuel_line in fuel_lines: 
            if fuel_line.active: 
                for converter_group in fuel_line.assigned_converters:
                    for converter_tag in converter_group:
                        converter =  converters[converter_tag]
                        if converter.active:
                            converter.inverse_calculation = True           
                            if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator): 
                                generator             = converter.generator   
                                state.conditions.energy.converters[generator.tag].outputs.power  =  total_elec_power*(1 - state.conditions.energy.hybrid_power_split_ratio ) 
                                P_mech, P_elec, stored_results_flag,stored_propulsor_tag         = converter.compute_performance(state,fuel_line,bus)  
                                bus_conditions.power_draw   -= P_elec/bus.efficiency
                                fuel_mdot                   += conditions.energy.converters[converter.tag].fuel_flow_rate  
                 
                            if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.Turboshaft):   
                                state.conditions.energy.converters[converter.tag].power = total_mech_power*(1 - state.conditions.energy.hybrid_power_split_ratio )   
                                P_mech, P_elec,stored_results_flag,stored_propulsor_tag = converter.compute_performance(state)   
                                fuel_mdot                   += conditions.energy.converters[converter.tag].fuel_flow_rate   
                    
        # 2.1 Electric Converters                            
        for bus in busses: 
            if bus.active == True:
                bus_conditions  = state.conditions.energy[bus.tag]            
                for converter_group in bus.assigned_converters:
                    for converter_tag in converter_group:
                        converter =  converters[converter_tag]
                        if converter.active: 
                            converter.inverse_calculation = True
                            if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.DC_Motor) or isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor):  
                                compute_motor_performance(converter,conditions)
                                bus_conditions.power_draw   += conditions.energy.converters[converter.tag].inputs.power/bus.efficiency
                                bus_conditions.current_draw  = bus_conditions.power_draw/bus.voltage                            
                                
                            if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.DC_Generator) or isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.PMSM_Generator):                              
                                compute_generator_performance(converter,conditions) 
                                bus_conditions.power_draw   -= conditions.energy.converters[converter.tag].outputs.power/bus.efficiency
                                bus_conditions.current_draw  = bus_conditions.power_draw/bus.voltage                            
                        
        # ----------------------------------------------------------        
        # Section 3.0 Sources
        # ----------------------------------------------------------
        # 3.1 Fuel Sources  
        for fuel_line in fuel_lines:     
            for fuel_tank in fuel_line.fuel_tanks:  
                conditions.energy[fuel_line.tag][fuel_tank.tag].mass_flow_rate  += fuel_tank.fuel_selector_ratio*fuel_mdot + fuel_tank.secondary_fuel_flow
                   
        time               = state.conditions.frames.inertial.time[:,0] 
        delta_t            = np.diff(time)
        total_mdot         += fuel_mdot
        
        # 3.2 Electric Sources   
        for bus in  busses:
            if bus.active: 
                for t_idx in range(state.numerics.number_of_control_points):            
                    stored_results_flag       = False
                    stored_battery_cell_tag   = None
                    
                    # ------------------------------------------------------------------------------------------------------------------- 
                    # 3.1 Batteries
                    # -------------------------------------------------------------------------------------------------------------------                
                    for battery_module in  bus.battery_modules:                   
                        if bus.identical_battery_modules == False:
                            # run analysis  
                            stored_results_flag, stored_battery_cell_tag =  battery_module.energy_calc(state,bus,coolant_lines, t_idx, delta_t)
                        else:             
                            if stored_results_flag == False: 
                                # run battery analysis 
                                stored_results_flag, stored_battery_cell_tag  =  battery_module.energy_calc(state,bus,coolant_lines, t_idx, delta_t)
                            else:
                                # use previous battery results 
                                battery_module.reuse_stored_data(state,bus,stored_results_flag, stored_battery_cell_tag)
                      
                    # ------------------------------------------------------------------------------------------------------------------- 
                    # 3.2 Fuel Cell Stacks
                    # ------------------------------------------------------------------------------------------------------------------- 
                    stored_results_flag       = False   
                    stored_fuel_cell_tag      = None                  
                    for fuel_cell_stack in  bus.fuel_cell_stacks:                   
                        if bus.identical_fuel_cell_stacks == False:
                            # run analysis  
                            stored_results_flag, stored_fuel_cell_tag =  fuel_cell_stack.energy_calc(state,bus,coolant_lines, t_idx, delta_t)
                        else:             
                            if stored_results_flag == False: 
                                # run battery analysis 
                                stored_results_flag, stored_fuel_cell_tag  =  fuel_cell_stack.energy_calc(state,bus,coolant_lines, t_idx, delta_t)
                            else:
                                # use previous battery results 
                                fuel_cell_stack.reuse_stored_data(state,bus,stored_results_flag, stored_fuel_cell_tag)
                             
                        # compute cryogen mass flow rate 
                        fuel_cell_stack_conditions  = state.conditions.energy[bus.tag].fuel_cell_stacks[fuel_cell_stack.tag]                        
                        cryogen_mdot[t_idx]        += fuel_cell_stack_conditions.H2_mass_flow_rate[t_idx]
                        
                        # compute total mass flow rate 
                        total_mdot[t_idx]     += fuel_cell_stack_conditions.H2_mass_flow_rate[t_idx]    
                       
                    # Step 3: Compute bus properties          
                    bus.compute_distributor_conditions(state,t_idx, delta_t)
                    
                    # Step 4 : Battery Thermal Management Calculations                    
                    for coolant_line in coolant_lines:
                        if t_idx != state.numerics.number_of_control_points-1: 
                            for heat_exchanger in coolant_line.heat_exchangers: 
                                heat_exchanger.compute_heat_exchanger_performance(state,bus,coolant_line,delta_t[t_idx],t_idx) 
                            for reservoir in coolant_line.reservoirs:   
                                reservoir.compute_reservior_coolant_temperature(state,coolant_line,delta_t[t_idx],t_idx) 
           
                # Step 5: Determine mass flow from cryogenic tanks 
                for cryogenic_tank in bus.cryogenic_tanks:
                    # Step 5.1: Determine the cumulative flow from each cryogen tank
                    fuel_tank_mdot = cryogenic_tank.croygen_selector_ratio*cryogen_mdot + cryogenic_tank.secondary_cryogenic_flow 
                    
                    # Step 5.2: DStore mass flow results 
                    conditions.energy[bus.tag][cryogenic_tank.tag].mass_flow_rate  = fuel_tank_mdot 
                                 
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
        
    
        if type(segment) != RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion: 
            unknowns(segment)  
            for network in segment.analyses.energy.vehicle.networks:
                # Fuel unknowns 
                for fuel_line_i, fuel_line in enumerate(network.fuel_lines):    
                    if fuel_line.active:
                        for propulsor_group in  fuel_line.assigned_propulsors:
                            propulsor = network.propulsors[propulsor_group[0]]
                            propulsor.unpack_propulsor_unknowns(segment)
                            
                # electric unknowns 
                for bus_i, bus in enumerate(network.busses):    
                    if bus.active:
                        for propulsor_group in  bus.assigned_propulsors:
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

        if type(segment) != RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion: 
            for network in segment.analyses.energy.vehicle.networks:
                for fuel_line_i, fuel_line in enumerate(network.fuel_lines):    
                    if fuel_line.active:
                        for propulsor_group in  fuel_line.assigned_propulsors:
                            propulsor =  network.propulsors[propulsor_group[0]]
                            propulsor.pack_propulsor_residuals(segment) 
                for bus_i, bus in enumerate(network.busses):    
                    if bus.active:
                        for propulsor_group in  bus.assigned_propulsors:
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
                propulsor.append_operating_conditions(segment,segment.state.conditions.energy,segment.state.conditions.noise)     
    
            for converter in network.converters: 
                converter.append_operating_conditions(segment,segment.state.conditions.energy)                 
    
            for fuel_line_i, fuel_line in enumerate(network.fuel_lines):   
                # ------------------------------------------------------------------------------------------------------            
                # Create fuel_line results data structure  
                # ------------------------------------------------------------------------------------------------------   
                segment.state.conditions.energy[fuel_line.tag] = RCAIDE.Framework.Mission.Common.Conditions() 
                segment.state.conditions.noise[fuel_line.tag]  = RCAIDE.Framework.Mission.Common.Conditions()   
                 
                # ------------------------------------------------------------------------------------------------------
                # Assign network-specific  residuals, unknowns and results data structures
                # ------------------------------------------------------------------------------------------------------
                if fuel_line.active:
                    for propulsor_group in  fuel_line.assigned_propulsors:
                        propulsor =  network.propulsors[propulsor_group[0]] 
                        if type(segment) != RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion:                               
                            propulsor.append_propulsor_unknowns_and_residuals(segment)
                        
                # ------------------------------------------------------------------------------------------------------
                # Assign sub component results data structures
                # ------------------------------------------------------------------------------------------------------  
                for fuel_tank in  fuel_line.fuel_tanks: 
                    fuel_tank.append_operating_conditions(segment,fuel_line) 
    
            # ------------------------------------------------------------------------------------------------------            
            # Create bus results data structure  
            # ------------------------------------------------------------------------------------------------------     
            for bus_i, bus in enumerate(network.busses):   
                # ------------------------------------------------------------------------------------------------------            
                # Create bus results data structure  
                # ------------------------------------------------------------------------------------------------------
                segment.state.conditions.energy[bus.tag] = RCAIDE.Framework.Mission.Common.Conditions() 
                segment.state.conditions.noise[bus.tag]  = RCAIDE.Framework.Mission.Common.Conditions()   
    
                # ------------------------------------------------------------------------------------------------------
                # Assign network-specific  residuals, unknowns and results data structures
                # ------------------------------------------------------------------------------------------------------
                if bus.active:
                    for propulsor_group in  bus.assigned_propulsors:
                        propulsor =  network.propulsors[propulsor_group[0]] 
                        if type(segment) != RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion:                        
                            propulsor.append_propulsor_unknowns_and_residuals(segment)
                        
                # ------------------------------------------------------------------------------------------------------
                # Assign sub component results data structures
                # ------------------------------------------------------------------------------------------------------ 
                bus.append_operating_conditions(segment)
                for battery_module in  bus.battery_modules: 
                    battery_module.append_operating_conditions(segment,bus) 
    
                for fuel_cell_stack in  bus.fuel_cell_stacks: 
                    fuel_cell_stack.append_operating_conditions(segment,bus)      
                    
                for tag, bus_item in bus.items():  
                    if issubclass(type(bus_item), RCAIDE.Library.Components.Component):
                        bus_item.append_operating_conditions(segment,bus)
         
                for cryogenic_tank in  bus.cryogenic_tanks: 
                    cryogenic_tank.append_operating_conditions(segment,bus)
                                                    
    
            for coolant_line_i, coolant_line in enumerate(network.coolant_lines):  
                # ------------------------------------------------------------------------------------------------------            
                # Create coolant_lines results data structure  
                # ------------------------------------------------------------------------------------------------------
                segment.state.conditions.energy[coolant_line.tag] = RCAIDE.Framework.Mission.Common.Conditions()        
                
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