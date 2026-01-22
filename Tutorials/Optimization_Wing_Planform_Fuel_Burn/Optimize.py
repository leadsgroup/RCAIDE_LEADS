# Optimize.py
# Created:  Feb 2016, M. Vegh
# Modified: Aug 2017, E. Botero
#           Aug 2018, T. MacDonald

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import RCAIDE 
from RCAIDE.Framework.Core import Units, Data
import numpy as np
import Vehicles
import Analyses
import Missions
import Procedure
import Plot_Mission
import matplotlib.pyplot as plt
from RCAIDE.Framework.Optimization.Common import  Nexus
from RCAIDE.Framework.Optimization .Packages.scipy import scipy_setup 

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    
    problem = setup()
    
    ## Base Input Values
    output = problem.objective()
     
     
    # Uncomment for the first optimization
    output = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    print (output)    

    print('fuel burn = ', problem.summary.base_mission_fuelburn)
    print('fuel margin = ', problem.all_constraints())
    
    Plot_Mission.plot_mission(problem)
    
    return

# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def setup():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    #   [ tag                   , initial,     lb , ub        , scaling , units ]
    problem.inputs = np.array([
        [ 'wing_area'           ,  92    ,    50. ,   130.    ,   100.  , 1*Units.meter**2],
        [ 'cruise_altitude'     ,   10    ,    8. ,    12.    ,   10.   , 1*Units.km],
    ],dtype=object)

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'fuel_burn', 10000, 1*Units.kg ]
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        [ 'design_range_fuel_margin' , '>', 0., 1E-1, 1*Units.less], #fuel margin defined here as fuel 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        [ 'wing_area'                        ,   ['vehicle_configurations.*.wings.main_wing.areas.reference',
                                                  'vehicle_configurations.*.reference_area'                    ]],
        [ 'cruise_altitude'                  , 'missions.base.segments.climb_3.altitude_end'                    ],
        [ 'fuel_burn'                        ,    'summary.base_mission_fuelburn'                               ],
        [ 'design_range_fuel_margin'         ,    'summary.max_zero_fuel_margin'                                ],
    ]    
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup()
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.setup(nexus.analyses)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()    
    nexus.total_number_of_iterations = 0
    return nexus
     
if __name__ == '__main__':
    main()
    plt.show()
