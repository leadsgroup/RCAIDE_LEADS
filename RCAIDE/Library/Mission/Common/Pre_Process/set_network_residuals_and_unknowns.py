# RCAIDE/Library/Missions/Common/Pre_Process/set_residuals_and_unknowns.py
#
#
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2026, M. Clarke
import RCAIDE
import numpy as np
import warnings

# ----------------------------------------------------------------------------------------------------------------------
#  set_residuals_and_unknowns
# ----------------------------------------------------------------------------------------------------------------------
def set_network_residuals_and_unknowns(mission):
    """Register network-level power unknowns and residuals based on topology.

    Power unknowns are only added when the network topology creates a degree
    of freedom that throttle alone cannot resolve:

    - **Pure fuel** — throttle drives the engine, fuel flow is determined.
      No power unknowns needed.
    - **Pure electric** — throttle drives the motor, electrical demand is
      determined by the load. No power unknowns needed.
    - **Hybrid (both chemical and electrical paths)** — throttle sets total
      propulsive demand, but the electrical power magnitude is an additional
      degree of freedom. ``electrical_power`` unknown added with a
      ``net_electrical_power = 0`` residual.
    - **Electrical consumers with no providers** — parasitic loads that
      cannot be balanced. Warning issued, no unknown added.
    """

    for segment in mission.segments:
        segment.state.number_of_network_unknowns   = 0
        segment.state.number_of_network_residuals  = 0

        if type(segment) != RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge:
            ones_row = segment.state.ones_row
            topology = segment.state.conditions.energy.topology

            is_hybrid = topology.has_chemical_path and topology.has_electrical_path

            # ------------------------------------------------------------------
            # Electrical power unknown — only for hybrid networks
            # ------------------------------------------------------------------
            if is_hybrid and topology.has_electrical_path:
                has_electrical_flow = False
                for dist_tag, domain in topology.distributor_domains.items():
                    if domain == 'electrical':
                        conn = topology.distributor_connections[dist_tag]
                        if len(conn.providers) > 0 and len(conn.consumers) > 0:
                            has_electrical_flow = True
                        elif len(conn.consumers) > 0 and len(conn.providers) == 0:
                            warnings.warn(
                                f"Distributor '{dist_tag}' has electrical consumers "
                                f"{[c.tag for c in conn.consumers]} but no providers. "
                                f"These loads will not be accounted for in the power balance.",
                                stacklevel=2)

                if has_electrical_flow:
                    # Estimate initial electrical power from system loads
                    initial_electrical_power = 0.0
                    for network in segment.analyses.vehicle.networks:
                        for system in network.systems:
                            if system.active and hasattr(system, 'power_draw'):
                                initial_electrical_power += system.power_draw

                    if initial_electrical_power == 0.0:
                        initial_electrical_power = 1000.0

                    segment.state.unknowns.network['electrical_power']              = initial_electrical_power * ones_row(1)
                    segment.state.residuals.network['electrical_power']             = 0.     * ones_row(1)
                    segment.state.unknowns_upper_bounds.network['electrical_power'] =  np.inf * ones_row(1)
                    segment.state.unknowns_lower_bounds.network['electrical_power'] = -np.inf * ones_row(1)
                    segment.state.number_of_network_unknowns  += 1
                    segment.state.number_of_network_residuals += 1
            
        for network in segment.analyses.vehicle.networks:         

            # ---------------------------------------------------------------------------------------------
            # Propulsors 
            # ---------------------------------------------------------------------------------------------
            for p_i,propulsor in  enumerate(network.propulsors):  
                if propulsor.active and (propulsor.identical_propulsors == False or p_i == 0): 
                    propulsor.append_unknowns_and_residuals(segment)
                    
            # ---------------------------------------------------------------------------------------------            
            # Distributors 
            # --------------------------------------------------------------------------------------------- 
            for distributor in network.distributors:                 
                distributor.append_unknowns_and_residuals(segment)                
    
            # ---------------------------------------------------------------------------------------------            
            # Source 
            # ---------------------------------------------------------------------------------------------          
            for source in network.sources:
                source.append_unknowns_and_residuals(segment) 
    
            # ---------------------------------------------------------------------------------------------            
            # System 
            # ---------------------------------------------------------------------------------------------          
            for system in network.systems:
                system.append_unknowns_and_residuals(segment)
    
            # ---------------------------------------------------------------------------------------------            
            # Modulator 
            # ---------------------------------------------------------------------------------------------          
            for modulator in network.modulators:
                modulator.append_unknowns_and_residuals(segment)              
                 
            # Ensure the mission knows how to pack and unpack the unknowns and residuals
            segment.process.iterate.unknowns.mission.network   = network.unpack_unknowns 
            segment.process.iterate.residuals.mission.network  = network.residuals
             