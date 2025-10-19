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

    # linking the different network components
    def evaluate(network,state,center_of_gravity):
        """ Computes the performance of the network.
        
            This routine evaluates propulsors, converters, modulators, distributors, sources, and systems,
            and assembles forces, moments, and power balances.
        
            Energetic domains (Effort–Flow pairs) used in the model follow the power-conjugate convention:
        
            +------------+--------------------------+--------------------+----------------+----------------+
            | Domain     | Effort                   | Flow               | Power Relation | Units          |
            +============+==========================+====================+================+================+
            | Propulsive | Force (F)                | Velocity (v)       | P = F · v      | N, m/s → W     |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Mechanical | Torque (τ)               | Angular speed (ω)  | P = τ · ω      | N·m, rad/s → W |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Electrical | Voltage (V)              | Current (I)        | P = V · I      | V, A → W       |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Chemical   | Lower Heating Value (LHV)| Mass flow (ṁ)      | P = ṁ·LHV      | J/kg, kg/s → W |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Pneumatic  | Pressure (p)             | Vol. flow rate (Ṽ) | P = p · Ṽ      | Pa, m³/s → W   |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Hydraulic  | Pressure (p)             | Vol. flow rate (Ṽ) | P = p · Ṽ      | Pa, m³/s → W   |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Thermal    | Temperature (T)          | Entropy flow (Ṡ)   | P = T · Ṡ      | K, W/K → W     |
            +------------+--------------------------+--------------------+----------------+----------------+

            Notes
            -----
            * Electrical bus power balance uses the sign convention: 
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

        for propulsor in propulsors:

            stored_results_flag  = False

            if propulsor.active:   
                if propulsor.identical_propulsors == False or stored_results_flag == False:
                    Thrust, Moment, Power, stored_results_flag, stored_propulsor_tag = propulsor.compute_performance(state, network, center_of_gravity= center_of_gravity)
                else:             
                    Thrust, Moment, Power = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag,center_of_gravity= center_of_gravity)

                if propulsor.reverse_thrust  ==  True:
                    total_thrust =  total_thrust * -1    
                    total_moment =  total_moment * -1 

                total_thrust             += Thrust   
                total_moment             += Moment  
                total_propulsive_power   += Power.propulsive 

                Network.update_distributor_net_power(propulsor, network, conditions, Power)

        # ----------------------------------------------------------
        # Systems
        # ----------------------------------------------------------

        for system in systems:

            Power = system.compute_performance(state)
            
            Network.update_distributor_net_power(system, network, conditions, Power)

        # ------------------------------------------------------------------------------------------------------------------- 
        # Converters
        # -------------------------------------------------------------------------------------------------------------------

        for converter_tag in network.non_propulsive_converters: 

            converter = converters[converter_tag]

            stored_results_flag = False

            if type(converter) == RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator or \
                    type(converter) == RCAIDE.Library.Components.Powertrain.Converters.Turboshaft: 
                converter.inverse_calculation = True 

            if converter.active:   
                Power, stored_results_flag, stored_converter_tag = converter.compute_performance(state)

                Network.update_distributor_net_power(converter, network, conditions, Power)  

        # ------------------------------------------------------------------------------------------------------------------- 
        # Modulators
        # -------------------------------------------------------------------------------------------------------------------

        for modulator in modulators:
                
            Power, stored_results_flag, stored_modulator_tag = modulator.compute_performance(network, state)
            
            Network.update_distributor_net_power(modulator, network, conditions, Power)  
       
        # -------------------------------------------------------------------------------------------------------------------
        # Other Distributors 
        # -------------------------------------------------------------------------------------------------------------------

        for distributor in network.distributors:
            for distributor_tag in distributor.assigned_distributors:
                
                Power = network.distributors[distributor_tag[0]].compute_performance(state)

                Network.update_distributor_net_power(network.distributors[distributor_tag[0]], network, conditions, Power)  

        # ----------------------------------------------------------        
        # Sources
        # ----------------------------------------------------------

        time               = state.conditions.frames.inertial.time[:,0] 
        delta_t            = np.diff(time)
                
        stored_results_flag       = False
        stored_battery_cell_tag   = None

        for source in sources: 
            for distributor_tag in source.assigned_distributors:
                distributor = distributors[distributor_tag[0]]

                if issubclass(type(source),RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):    
                    
                    Power = source.compute_performance(state,distributor)   
                    
                    total_mdot  += conditions.energy.distributors[distributor.tag].fuel_mass_flow_rate
                
                elif issubclass(type(source),RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module):   
                    for t_idx in range(state.numerics.number_of_control_points):   
                        if distributor.identical_battery_modules == False or stored_results_flag == False: 
                            Power, stored_results_flag, stored_battery_cell_tag =  source.compute_performance(state,distributor,network, t_idx, delta_t)
                        else:             
                            Power = source.reuse_stored_data(state, stored_battery_cell_tag)        
                        
                        distributor.compute_distributor_conditions(source, state, t_idx,delta_t)
                
                Network.update_distributor_net_power(source, network, conditions, Power)  
                    
        # # ----------------------------------------------------------
        # # Regenerative Power 
        # # ----------------------------------------------------------

        # if isinstance(distributor,RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):   
        #     total_elec_power        -= state.conditions.energy.distributors[distributor.tag].regenerative_power*bus_voltage* distributor.power_split_ratio  /distributor.efficiency   
          

    # # ----------------------------------------------------------
    # # Finalize distributor residual 
    # # ----------------------------------------------------------
    # if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
    #     conditions.energy.distributors[distributor.tag].power_draw   = total_elec_power
    #     conditions.energy.distributors[distributor.tag].current_draw = total_elec_power / bus_voltage

        # # ------------------------------------------------------------------------------------------------------------------- 
        # # Thermal Management
        # # -------------------------------------------------------------------------------------------------------------------        
        # for t_idx in range(state.numerics.number_of_control_points):        
        #     for distributor in network.distributors:
        #         if isinstance(distributor,RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line):
        #             if t_idx != state.numerics.number_of_control_points-1: 
        #                 for heat_exchanger in distributor.heat_exchangers: 
        #                     heat_exchanger.compute_heat_exchanger_performance(state,distributor,distributor,delta_t[t_idx],t_idx) 
        #                 for reservoir in distributor.reservoirs:   
        #                     reservoir.compute_reservior_coolant_temperature(state,distributor,delta_t[t_idx],t_idx)
                                                        
        conditions.energy.thrust_force_vector  = total_thrust
        conditions.energy.thrust_moment_vector = total_moment 
        conditions.energy.net_power            = total_propulsive_power
        conditions.weights.vehicle_mass_rate   = total_mdot  
    
        return
    
    @staticmethod
    def update_distributor_net_power(component, network, conditions, Power):

        """
        Accumulate a component's multi-domain power into the residuals of its assigned distributors.

        Sign convention
        ---------------
        Positive values mean *loads/draws* on the distributor; negative values mean *supplies/sources*
        (e.g., regeneration). Units are Watts for all domains.

        """

        for dist_tag in component.assigned_distributors[0]:
            dist = network.distributors[dist_tag] 

            if isinstance(network.distributors[dist_tag], RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                if isinstance(component, RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit):
                    if network.distributors[dist_tag].bus_type == 'AC':
                        conditions.energy.distributors[dist_tag].net_electrical_power  += Power.electrical * dist.power_split_ratio / dist.electrical_efficiency
                    else:
                        conditions.energy.distributors[dist_tag].net_electrical_power  += - Power.electrical * component.electrical_efficiency * dist.power_split_ratio / dist.electrical_efficiency
                else:    
                    conditions.energy.distributors[dist_tag].net_electrical_power  += Power.electrical * dist.power_split_ratio / dist.electrical_efficiency
            
            # elif isinstance(network.distributors[dist_tag], RCAIDE.Library.Components.Powertrain.Distributors.Mechanical_Line):
            #     conditions.energy.distributors[dist_tag].net_mechanical_power  += P_mech * dist.power_split_ratio / dist.mechanical_efficiency
            
            elif isinstance(network.distributors[dist_tag], RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                conditions.energy.distributors[dist_tag].net_hydraulic_power   += Power.chemical * dist.power_split_ratio / dist.hydraulic_efficiency
                if isinstance(component, RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor):
                    m_dot_fuel = conditions.energy.propulsors[component.tag].fuel_mass_flow_rate
                elif isinstance(component, RCAIDE.Library.Components.Powertrain.Converters.Converter):
                    m_dot_fuel = conditions.energy.converters[component.tag].fuel_mass_flow_rate
                elif isinstance(component, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank) or \
                    isinstance(component, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank) or \
                    isinstance(component, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank) or \
                    isinstance(component, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank):
                    m_dot_fuel = conditions.energy.sources[component.tag].mass_flow_rate
                conditions.energy.distributors[dist_tag].fuel_mass_flow_rate += m_dot_fuel
            
            elif isinstance(network.distributors[dist_tag], RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line):
                conditions.energy.distributors[dist_tag].net_thermal_power     += Power.thermal * dist.power_split_ratio / dist.thermal_efficiency
        
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