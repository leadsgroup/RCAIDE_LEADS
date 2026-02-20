# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE 
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Optimization.Common import * 
from RCAIDE.Framework.Optimization.Packages.scipy import scipy_setup

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
    
    # define optmiztion problem
    problem = define_optimization_problem()
    
    # create carpet plot  
    carpet_plot(problem,
                design_input_1_index            = 0, 
                design_input_2_index            = 1,                
                number_of_points                = 5,
                generate_objective_plot         = True, 
                objective_plot_constraint_index = 0, 
                generate_constraint_plots       = True)
    
    ## create line plot 
    #line_plot(problem,
              #design_input_1_index = 0, 
              #number_of_points     = 3, 
              #plot_objective       = True,
              #plot_constraint      = True) 
     
    ## start clock 
    #ti = time.time()
    
    ## solve optimization problem
    #solution = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    
    ## stop clock 
    #tf                   = time.time()
    #elapsed_time         = round((tf-ti)/60,2)
    #print('Simulation Time: ' + str(elapsed_time) + ' mins')    

    ## write optimization outputs     
    #print_optimization_results(problem)  
    
    ## plot mission with optimized aircraft 
    #Plot_Mission.plot_mission(problem)
    
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
        [ 'cruise_altitude'     ,   10    ,    6.  ,    12.    ,   10.   , 1*Units.km],
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
        [ 'fuel_margin' , '>', 0.15 , 1, 1*Units.less], # fuel margin defined here as fuel
        [ 'range_residual'    , '>', 0.   , 1, 1*Units.less], # 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    #  Aliases - links between user specified terms and RCAIDE terms 
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ] 
    problem.aliases = [
        [ 'wing_area'                ,  ['vehicle_configurations.*.wings.main_wing.areas.reference','vehicle_configurations.*.reference_area' ]  ], 
        [ 'cruise_altitude'          ,  ['missions.base_mission.segments.climb_3.altitude_end', 'missions.base_mission.segments.cruise.altitude']],
        [ 'cruise_distance'          ,  'missions.base_mission.segments.cruise.distance'                                                         ],
        [ 'fuel_burn'                ,  'summary.fuel_burn'                                                                                      ],
        [ 'range_residual'           ,  'summary.range_residual'                                                                                 ],
        [ 'fuel_margin'              ,  'summary.fuel_margin'                                                                                    ],
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
