# RCAIDE/Library/Missions/Common/Update/aero_propulsion_coupling.py
# 
# 
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Aero Propulsion Coupling
# ---------------------------------------------------------------------------------------------------------------------- 
def aero_propulsion_coupling(segment):
    """ Re-evaluates the network, thrust and aerodynamics updates so rotors and wings see each other's latest state
        
        Assumptions:
        Runs after the first network, thrust and aerodynamics pass of the iteration, and only for vehicles
        with a network whose aero_propulsion_coupling is enabled. The number of passes is fixed rather than
        set by a convergence check, so the residuals stay a smooth function of the unknowns; the mission
        solver iterates the coupling to convergence.
        
        Inputs:
            segment.state.numerics.aero_propulsion_coupling.number_of_passes    [-]
                 
        Outputs: 
            None
      
        Properties Used:
        N/A
                    
    """ 
    coupled = any(network.aero_propulsion_coupling for network in segment.analyses.vehicle.networks)
    if not coupled:
        return

    steps = segment.process.iterate.conditions
    for i in range(segment.state.numerics.aero_propulsion_coupling.number_of_passes):
        steps.network(segment)
        steps.thrust(segment)
        steps.aerodynamics(segment)
    return
