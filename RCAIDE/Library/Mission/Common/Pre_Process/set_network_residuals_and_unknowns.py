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

    The ``electrical_power`` unknown exists for exactly one reason: a
    propulsor with an integrated drive generator/motor (e.g. a turbofan's
    IDG) needs its own electrical power *before* the rest of the network
    (which determines that demand) has been evaluated -- a genuine
    circularity that throttle alone cannot resolve. Every other consumer
    (batteries, fuel cells, standalone generators) is evaluated after its
    bus's demand is already known and reads it directly, so it needs no
    unknown.

    Electrical distributors with consumers but no providers are parasitic
    loads that cannot be balanced -- a warning is issued, no unknown added.
    """

    for segment in mission.segments:
        segment.state.number_of_network_unknowns   = 0
        segment.state.number_of_network_residuals  = 0

        if type(segment) != RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge:
            ones_row = segment.state.ones_row
            topology = segment.state.conditions.energy.topology

            # Warn about parasitic electrical loads that have no provider on their bus.
            for dist_tag, domain in topology.distributor_domains.items():
                if domain == 'electrical':
                    conn = topology.distributor_connections[dist_tag]
                    if len(conn.consumers) > 0 and len(conn.providers) == 0:
                        warnings.warn(
                            f"Distributor '{dist_tag}' has electrical consumers "
                            f"{[c.tag for c in conn.consumers]} but no providers. "
                            f"These loads will not be accounted for in the power balance.",
                            stacklevel=2)

            # Electrical power unknown -- only needed when a propulsor's
            # integrated drive generator/motor requires the network's
            # electrical power before the network itself has been evaluated.
            has_electrical_power_circularity = False
            for network in segment.analyses.vehicle.networks:
                for propulsor in network.propulsors:
                    if getattr(propulsor, 'integrated_drive_generator', None) is not None or \
                       getattr(propulsor, 'integrated_drive_motor', None) is not None:
                        has_electrical_power_circularity = True

            if has_electrical_power_circularity:
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
            # A propulsor only needs its own unknown/residual if it isn't a
            # reuse-eligible duplicate of the most recently registered one --
            # which requires both identical_propulsors=True *and* the same
            # distributor group (see the matching comments in Network.py;
            # e.g. a vehicle with cruise propellers and lift rotors has two
            # distinct groups even though every propulsor defaults to
            # identical_propulsors=True).
            reference_distributors = None
            for p_i,propulsor in  enumerate(network.propulsors):
                if propulsor.active:
                    if propulsor.identical_propulsors == False or reference_distributors is None or propulsor.assigned_distributors != reference_distributors:
                        propulsor.append_unknowns_and_residuals(segment)
                        reference_distributors = propulsor.assigned_distributors
                    
            # ---------------------------------------------------------------------------------------------            
            # Distributors 
            # --------------------------------------------------------------------------------------------- 
            for distributor in network.distributors:                 
                distributor.append_unknowns_and_residuals(segment)                
    
            # ---------------------------------------------------------------------------------------------
            # Source
            # ---------------------------------------------------------------------------------------------
            # Same reuse-eligibility rule as propulsors above -- must stay
            # consistent with Network.unpack_unknowns()/residuals(), which
            # skip a source under the identical condition.
            reference_source_distributors = None
            for source in network.sources:
                if source.active:
                    if source.identical_sources == False or reference_source_distributors is None or source.assigned_distributors != reference_source_distributors:
                        source.append_unknowns_and_residuals(segment)
                        reference_source_distributors = source.assigned_distributors
    
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
             