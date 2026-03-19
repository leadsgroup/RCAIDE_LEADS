# RCAIDE/Library/Missions/Common/Pre_Process/set_residuals_and_unknowns.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  set_residuals_and_unknowns
# ----------------------------------------------------------------------------------------------------------------------  
def set_network_residuals_and_unknowns(mission):
     
    for segment in mission.segments: 
        segment.state.number_of_network_unknowns   = 0 
        segment.state.number_of_network_residuals  = 0 
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
                source.append_unknowns_and_residuals(segment) #NEED TO UPDATE TO HANDLE MULTIPLE BATTERIES   
    
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