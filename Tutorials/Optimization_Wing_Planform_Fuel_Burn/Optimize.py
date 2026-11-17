# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Units, Data, base_vehicle
from RCAIDE.Framework.Optimization.Common import Design_Problem, solve_design_problem

# local file imports
import Vehicles     # defines aircraft
import Analyses     # defines analyses
import Missions     # defines flight profile

# python imports
import time
import jax

# the derivatives need double precision
jax.config.update("jax_enable_x64", True)

# ----------------------------------------------------------------------------------------------------------------------
#  Main Script
# ----------------------------------------------------------------------------------------------------------------------
def main():

    # define optimization problem
    problem = define_optimization_problem()

    # start clock
    ti = time.time()

    # each design solves the mission once; the gradients of the objective and the constraints come from the mission
    # derivative without solving it again. Few iterations keep the tutorial short.
    solution = solve_design_problem(problem, optimizer='SLSQP', iterations=3, tolerance=1e-4)

    # stop clock
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Simulation Time: ' + str(elapsed_time) + ' mins')

    # write optimization outputs
    print('\nOptimal design')
    for path, value in solution.design.items():
        print(f'  {path:45s} {value:.6g}')
    print('Outputs')
    for name, value in solution.outputs.items():
        print(f'  {name:45s} {value:.6g}')
    return

# ----------------------------------------------------------------------------------------------------------------------
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------------------------------------------------------
def define_optimization_problem():

    # the vehicle and its configurations, analyses and missions, as the tutorial files build them
    configs  = Vehicles.setup()
    analyses = Analyses.setup(configs)
    missions = Missions.setup(analyses)

    problem          = Design_Problem()
    problem.vehicle  = base_vehicle(configs.base)
    problem.missions = Data(base_mission=missions.base_mission)

    # the gradients are taken at the converged state, so the segments are solved tightly: at the default 1e-6 the
    # range against span has the wrong sign
    for segment in problem.missions.base_mission.segments.values():
        segment.state.numerics.mission_solver.tolerance = 1e-10

    # -------------------------------------------------------------------
    # Variables: paths to floats of the vehicle or of a segment
    # -------------------------------------------------------------------
    # the wing is defined by its span, root chord and segments, so its area and aspect ratio follow the span
    #   [ path                                              , initial          , lower bound      , upper bound       , scale            ]
    problem.variables = [
        [ 'vehicle.wings.main_wing.spans.projected'           , 28.72            , 24.              , 34.               , 28.72            ],
        [ 'missions.base_mission.segments.cruise.altitude'    , 10.668 * Units.km, 9.    * Units.km , 12.   * Units.km  , 10.   * Units.km ],
        [ 'missions.base_mission.segments.cruise.distance'    , 1000 * Units.nmi , 500   * Units.nmi, 2000  * Units.nmi , 1000  * Units.nmi],
    ]

    # the last climb ends at the cruise altitude
    problem.update_missions = climb_to_cruise_altitude

    # -------------------------------------------------------------------
    # Objective and constraints, on the outputs
    # -------------------------------------------------------------------
    problem.outputs     = mission_outputs
    #   [ output, scale ]
    problem.objective   = [ 'fuel_burn', 10000. ]
    #   [ output        , sense, edge               , scale            ]
    problem.constraints = [
        [ 'fuel_margin' , '>'  , 0.15               , 1.               ],
        [ 'range'       , '>'  , 1500. * Units.nmi  , 1000. * Units.nmi],
    ]
    return problem


def climb_to_cruise_altitude(missions):
    # the climb ends where the cruise flies
    segments                       = missions.base_mission.segments
    segments.climb_3.altitude_end  = segments.cruise.altitude
    return

# ----------------------------------------------------------------------------------------------------------------------
#   Outputs
# ----------------------------------------------------------------------------------------------------------------------
def mission_outputs(missions):
    segments    = missions.base_mission.segments
    vehicle     = segments.takeoff.analyses.vehicle
    fuel_burn   = segments.takeoff.conditions.weights.vehicle.mass[0, 0] - segments.landing.conditions.weights.vehicle.mass[-1, 0]
    max_fuel    = vehicle.mass_properties.max_fuel
    return {'fuel_burn':   float(fuel_burn),
            'fuel_margin': float((max_fuel - fuel_burn) / max_fuel),
            'range':       float(segments.landing.conditions.frames.inertial.aircraft_range[-1, 0])}


if __name__ == '__main__':
    main()
