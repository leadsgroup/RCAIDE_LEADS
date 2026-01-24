# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE 
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Optimization.Common import  Nexus
from RCAIDE.Framework.Optimization .Packages.scipy import scipy_setup

# local file imports 
import Vehicles     # defines aircraft 
import Analyses     # defines analyses
import Missions     # defines flight profile 
import Procedure    # defines optimization interation process 
import Plot_Mission # plots mission 

# python imports 
import matplotlib.pyplot as plt
import numpy as np
import time 

# ----------------------------------------------------------------------        
#  Main Script
# ----------------------------------------------------------------------  
def main():
    # start clock 
    ti                   = time.time()
    
    # define optmiztion problem
    problem = define_optimization_problem()
     
    # solve optimization problem
    solution = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    print (solution)    
    
    # stop clock 
    tf                   = time.time()
    elapsed_time         = round((tf-ti)/60,2)
    print('Simulation Time: ' + str(elapsed_time) + ' mins')    
        

    print('fuel burn = ', problem.summary.base_mission_fuelburn)
    print('fuel margin = ', problem.all_constraints())
    
    
    Plot_Mission.plot_mission(problem)
    
    return

# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  
def define_optimization_problem():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs - i.e. design variables 
    # ------------------------------------------------------------------- 
    #   [ tag                   , initial,     lb , ub        , scaling , units ]
    problem.inputs = np.array([
        [ 'wing_area'           ,  92     ,    50. ,   130.    ,   100.  , 1*Units.meter**2],
        [ 'wing_span'           ,  28.72  ,    25  ,    30.    ,   100.  , 1*Units.meter**2],
        [ 'cruise_altitude'     ,   10    ,    8.  ,    12.    ,   10.   , 1*Units.km],
        [ 'cruise_distance'     ,  1000   ,   10.  ,   5000.   ,   1000. , 1*Units.nmi   ],
    ],dtype=object)

    # -------------------------------------------------------------------
    # Objective - i.e. goal 
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
        [ 'design_range_fuel_margin' , '>', 0.   , 1E-1, 1*Units.less], # fuel margin defined here as fuel
        [ 'design_range_residual'    , '>', 0.   , 1E-1, 1*Units.less], # 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    #  Aliases - links between user specified terms and RCAIDE terms 
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ] 
    problem.aliases = [
        [ 'wing_area'                ,  ['vehicle_configurations.*.wings.main_wing.areas.reference','vehicle_configurations.*.reference_area' ]  ],
        [ 'wing_span'                ,  'vehicle_configurations.*.wings.main_wing.spans.projected'                                               ],
        [ 'cruise_altitude'          ,  ['missions.base_mission.segments.climb_3.altitude_end', 'missions.base_mission.segments.cruise.altitude']],
        [ 'cruise_distance'          ,  'missions.base_mission.segments.cruise.distance'                                                         ],
        [ 'fuel_burn'                ,  'summary.base_mission_fuelburn'                                                                          ],
        [ 'design_range_residual'    ,  'summary.design_range_residual'                                                                          ],
        [ 'design_range_fuel_margin' ,  'summary.max_zero_fuel_margin'                                                                           ],
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
