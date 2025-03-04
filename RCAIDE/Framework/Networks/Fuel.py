# RCAIDE/Framework/Energy/Networks/Fuel.py
# 
# Created:  Oct 2023, M. Clarke
#           Jan 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports
import  RCAIDE 
from RCAIDE.Framework.Mission.Common                      import Residuals 
from RCAIDE.Library.Mission.Common.Unpack_Unknowns.energy import unknowns
from .Network                                             import Network   

# ----------------------------------------------------------------------------------------------------------------------
# Fuel
# ----------------------------------------------------------------------------------------------------------------------  
class Fuel(Network):
    """ This is a  fuel-based network. 
    
        Assumptions:
        None
        
        Source: 
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

        self.tag                          = 'fuel'
        self.reverse_thrust               = False
        self.wing_mounted                 = True   
    # linking the different network components
    def evaluate(network,state,center_of_gravity):
        """ Calculate thrust given the current state of the vehicle
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs: 
    
            Outputs:  
        """           

        # Step 1: Unpack
        conditions     = state.conditions  
        fuel_lines     = network.fuel_lines 
        reverse_thrust = network.reverse_thrust
        total_thrust   = 0. * state.ones_row(3) 
        total_moment   = 0. * state.ones_row(3) 
        total_power    = 0. * state.ones_row(1) 
        total_mdot     = 0. * state.ones_row(1)   
        
        # Step 2: loop through compoments of network and determine performance
        for fuel_line in fuel_lines:
            fuel_line_mdot       = 0. * state.ones_row(1)  
            stored_results_flag  = False
            stored_propulsor_tag = None
            # Step 2.1: Compute thrust,moment and power of propulsors
            for propulsor_group in fuel_line.assigned_propulsors:
                for propulsor_tag in propulsor_group:
                    propulsor =  network.propulsors[propulsor_tag]
                    if propulsor.active and fuel_line.active:   
                        if network.identical_propulsors == False:
                            # run analysis  
                            T,M,P,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state,center_of_gravity)
                        else:             
                            if stored_results_flag == False: 
                                # run propulsor analysis 
                                T,M,P,stored_results_flag,stored_propulsor_tag = propulsor.compute_performance(state,center_of_gravity)
                            else:
                                # use previous propulsor results 
                                T,M,P = propulsor.reuse_stored_data(state,network,stored_propulsor_tag,center_of_gravity)
                          
                        total_thrust += T   
                        total_moment += M   
                        total_power  += P
                        
                        # compute fuel line mass flow rate 
                        fuel_line_mdot += conditions.energy[propulsor.tag].fuel_flow_rate
                        
                        # compute total mass flow rate 
                        total_mdot     += conditions.energy[propulsor.tag].fuel_flow_rate 
                
            # Step 2.2: Determine cumulative fuel flow from each fuel tank  
            for fuel_tank in fuel_line.fuel_tanks:  
                conditions.energy[fuel_line.tag][fuel_tank.tag].mass_flow_rate  = fuel_tank.fuel_selector_ratio*fuel_line_mdot + fuel_tank.secondary_fuel_flow         
                            
        # Step 3: Pack results
        if reverse_thrust ==  True:
            total_thrust =  total_thrust* -1
            total_moment =  total_moment* -1
            
        conditions.energy.thrust_force_vector  = total_thrust
        conditions.energy.thrust_moment_vector = total_moment
        conditions.energy.power                = total_power 
        conditions.energy.vehicle_mass_rate    = total_mdot    
        
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
        
        if issubclass(type(segment), type(RCAIDE.Framework.Mission.Segments.Ground)):
            pass 
        for network in segment.analyses.energy.vehicle.networks:
            for fuel_line_i, fuel_line in enumerate(network.fuel_lines):    
                if fuel_line.active:
                    for propulsor_group in  fuel_line.assigned_propulsors:
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
            for fuel_line_i, fuel_line in enumerate(network.fuel_lines):    
                if fuel_line.active:
                    for propulsor_group in  fuel_line.assigned_propulsors:
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
            for p_i, propulsor in enumerate(network.propulsors): 
                propulsor.append_operating_conditions(segment)           
    
                for tag, propulsor_item in  propulsor.items():  
                    if issubclass(type(propulsor_item), RCAIDE.Library.Components.Component):
                        propulsor_item.append_operating_conditions(segment,propulsor)            
    
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
                        propulsor.append_propulsor_unknowns_and_residuals(segment)
    
                # ------------------------------------------------------------------------------------------------------
                # Assign sub component results data structures
                # ------------------------------------------------------------------------------------------------------  
                for fuel_tank in  fuel_line.fuel_tanks: 
                    fuel_tank.append_operating_conditions(segment,fuel_line)  
    
                for tag, fuel_line_item in  fuel_line.items():  
                    if issubclass(type(fuel_line_item), RCAIDE.Library.Components.Component):
                        fuel_line_item.append_operating_conditions(segment,fuel_line)                     
    
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
                     
        segment.process.iterate.unknowns.network   = self.unpack_unknowns      
        segment.process.iterate.residuals.network  = self.residuals
        
        return segment

    __call__ = evaluate     