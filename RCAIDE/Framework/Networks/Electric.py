# RCAIDE/Energy/Networks/Electric.py
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar
#           Jan 2025, M. Clarke
  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE  
from RCAIDE.Framework.Mission.Common                      import Residuals
from RCAIDE.Library.Mission.Common.Unpack_Unknowns.energy import unknowns
from .Network                                             import Network              
from RCAIDE.Library.Methods.Powertrain.Systems.compute_avionics_power_draw import compute_avionics_power_draw
from RCAIDE.Library.Methods.Powertrain.Systems.compute_payload_power_draw  import compute_payload_power_draw

# Python imports
import  numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  Electric
# ----------------------------------------------------------------------------------------------------------------------  
class Electric(Network):
    """ An electric network comprising one or more electrichemical energy source and/or fuel cells that to power electro-
        mechanical conversion machines eg. electric motors via a bus. Electronic speed controllers, thermal management
        system, avionics, and other eletric power systes paylaods are also modeled. Ducted fans, generators, rotors and
        motors etc. arranged into groups, called propulsor groups, to siginify how they are connected in the network.
        The network also takes into consideration thermal management components that are connected to a coolant line.
        Based on the propulsor, additional unknowns and residuals are added to the mission  


        Assumptions:
        The y axis rotation is used for rotating the rotor about the Y-axis for tilt rotors and tiltwings

        Source:
        None
    """  
    def __defaults__(self):
        """ This sets the default values for the network to function.

            Assumptions:
            None

            Source:
            N/A

            Inputs:
            None

            Outputs:
            None

            Properties Used:
            N/A
        """         

        self.tag                          = 'electric'
        self.system_voltage               = None   
        self.reverse_thrust               = False
        self.wing_mounted                 = True        

    # manage process with a driver function
    def evaluate(network,state,center_of_gravity):
        """ Calculate thrust given the current state of the vehicle

            Assumptions:
            Caps the throttle at 110% and linearly interpolates thrust off that

            Source:
            N/A

            Inputs:
            state [state()]

            Outputs:
            results.thrust_force_vector         [newtons]
            results.vehicle_mass_rate           [kg/s]
            conditions.energy                   [-]    

            Properties Used:
            Defaulted values
        """          

        # unpack   
        conditions      = state.conditions 
        busses          = network.busses 
        coolant_lines   = network.coolant_lines
        total_thrust    = 0. * state.ones_row(3) 
        total_power     = 0. * state.ones_row(1) 
        total_moment    = 0. * state.ones_row(3)  
        total_mdot      = 0. * state.ones_row(1)   
        reverse_thrust  = network.reverse_thrust
       
        # Step 1: Compute performance of electric propulsor components 
        for bus in busses:
            cryogen_mdot    = 0. * state.ones_row(1)  
            T               = 0. * state.ones_row(1) 
            total_power     = 0. * state.ones_row(1) 
            M               = 0. * state.ones_row(1)  
            avionics        = bus.avionics
            payload         = bus.payload  

            # Avionics Power Consumtion
            avionics_conditions = state.conditions.energy[bus.tag][avionics.tag]
            compute_avionics_power_draw(avionics,avionics_conditions,conditions)

            # Payload Power
            payload_conditions = state.conditions.energy[bus.tag][payload.tag]
            compute_payload_power_draw(payload,payload_conditions,conditions)

            # Bus Voltage 
            bus_voltage = bus.voltage * state.ones_row(1)

            if conditions.energy.recharging:
                avionics_power         = (avionics_conditions.power*bus.power_split_ratio)* state.ones_row(1)
                payload_power          = (payload_conditions.power*bus.power_split_ratio)* state.ones_row(1)            
                total_esc_power        = 0 * state.ones_row(1)
                bus.charging_current   = bus.nominal_capacity * bus.charging_c_rate 
                charging_power         = (bus.charging_current*bus_voltage*bus.power_split_ratio)

                # append bus outputs to bus
                bus_conditions                    = state.conditions.energy[bus.tag]
                bus_conditions.power_draw         = ((avionics_power + payload_power + total_esc_power) - charging_power)/bus.efficiency
                bus_conditions.current_draw       = -bus_conditions.power_draw/bus.voltage

            else:       
                # compute energy consumption of each electrochemical energy source on bus 
                stored_results_flag  = False
                stored_propulsor_tag = None 
                for propulsor_group in bus.assigned_propulsors:
                    for propulsor_tag in propulsor_group:
                        propulsor =  network.propulsors[propulsor_tag]
                        if propulsor.active and bus.active:       
                            if network.identical_propulsors == False:
                                # run analysis  
                                T,M,P,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state,bus_voltage,center_of_gravity)
                            else:             
                                if stored_results_flag == False: 
                                    # run propulsor analysis 
                                    T,M,P,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state,bus_voltage,center_of_gravity)
                                else:
                                    # use previous propulsor results 
                                    T,M,P = propulsor.reuse_stored_data(state,network,stored_propulsor_tag,center_of_gravity)
    
                            total_thrust += T   
                            total_moment += M   
                            total_power  += P 

                # compute power from each componemnt 
                avionics_power  = (avionics_conditions.power*bus.power_split_ratio)* state.ones_row(1) 
                payload_power   = (payload_conditions.power*bus.power_split_ratio)* state.ones_row(1)   
                charging_power  = (state.conditions.energy[bus.tag].regenerative_power*bus_voltage*bus.power_split_ratio) 
                total_esc_power = total_power*bus.power_split_ratio  

                # append bus outputs to electrochemical energy source 
                bus_conditions                    = state.conditions.energy[bus.tag]
                bus_conditions.power_draw        += ((avionics_power + payload_power + total_esc_power) - charging_power)/bus.efficiency
                bus_conditions.current_draw       = bus_conditions.power_draw/bus_voltage

        
        # Step 2.1: Compute performance of electro-chemical energy (battery) compoments   
        time               = state.conditions.frames.inertial.time[:,0] 
        delta_t            = np.diff(time)
        for bus in  busses:    
            for t_idx in range(state.numerics.number_of_control_points):            
                stored_results_flag       = False
                stored_battery_cell_tag   = None
                
                stored_fuel_cell_tag      = None  
                # Step 2.1.a : Battery Module Performance          
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
                 
                # Step 2.1.b : Fuel Cell Stack Performance           
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
                    fuel_cell_stack_conditions  = bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag]                        
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
        conditions.energy.power                = total_power 
        conditions.energy.thrust_moment_vector = total_moment 
        conditions.energy.vehicle_mass_rate    = total_mdot  

        return




    def unpack_unknowns(self,segment):
        """ This adds additional unknowns which are unpacked from the mission solver and send to the network.

            Assumptions:
            None

            Source:
            N/A

            Inputs:
            unknowns specific to the rotors                        [None] 
            unknowns specific to the battery cell                  

            Outputs:
            state.conditions.energy.bus.rotor.power_coefficient(s) [None] 
            conditions specific to the battery cell 

            Properties Used:
            N/A
        """
 
        unknowns(segment)
         
        for network in segment.analyses.energy.vehicle.networks:
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
              
        for network in segment.analyses.energy.vehicle.networks:
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
            estimated_battery_voltages                                    [v]
            estimated_rotor_power_coefficients                            [float]s
            estimated_battery_cell_temperature                            [Kelvin]
            estimated_battery_state_of_charges                            [unitless]
            estimated_battery_cell_currents                               [Amperes]

            Outputs: 
            segment

            Properties Used:
            N/A
        """                
        segment.state.residuals.network = Residuals()
        
        for network in segment.analyses.energy.vehicle.networks:
            for p_i, propulsor in enumerate(network.propulsors): 
                propulsor.append_operating_conditions(segment)           
    
                for tag, propulsor_item in  propulsor.items():  
                    if issubclass(type(propulsor_item), RCAIDE.Library.Components.Component):
                        propulsor_item.append_operating_conditions(segment,propulsor)            
            
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
                        propulsor.append_propulsor_unknowns_and_residuals(segment)
                        
                # ------------------------------------------------------------------------------------------------------
                # Assign sub component results data structures
                # ------------------------------------------------------------------------------------------------------ 
                bus.append_operating_conditions(segment)
                for battery_module in  bus.battery_modules: 
                    battery_module.append_operating_conditions(segment,bus) 
    
                for fuel_cell_stack in  bus.fuel_cell_stacks: 
                    fuel_cell_stack.append_operating_conditions(segment,bus)      
                    
                for tag, bus_item in  bus.items():  
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
    __call__ = evaluate 