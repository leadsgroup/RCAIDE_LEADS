# test_declared_values.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Mission.Common.Pre_Process import record_undeclared_values, check_undeclared_values, find_undeclared_values

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_declared_vehicle()
    check_script_value_warns()
    check_method_value_raises()
    check_method_output_container()
    print('test_declared_values passed')
    return


def build_mission():
    # a mission with one segment flying a vehicle with one main wing
    vehicle      = RCAIDE.Vehicle()
    wing         = RCAIDE.Library.Components.Wings.Main_Wing()
    vehicle.append_component(wing)
    segment      = RCAIDE.Framework.Mission.Segments.Cruise.Constant_Speed_Constant_Altitude()
    segment.tag  = 'cruise'
    segment.analyses.vehicle = vehicle
    mission      = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.append_segment(segment)
    return mission, vehicle


def check_declared_vehicle():
    # a vehicle holding only declared values has nothing to report
    mission, vehicle = build_mission()
    paths = find_undeclared_values(mission)
    assert paths == [], paths
    return


def check_script_value_warns():
    # a value the script made up is recorded, and the check at the end of setup accepts it
    mission, vehicle = build_mission()
    vehicle.wings.main_wing.made_up_value = 1.0
    record_undeclared_values(mission)
    assert mission.undeclared_vehicle_values == ['vehicle.wings.main_wing.made_up_value'], mission.undeclared_vehicle_values
    check_undeclared_values(mission)
    return


def check_method_value_raises():
    # a value added after the record, as a setup method would, is an error
    mission, vehicle = build_mission()
    record_undeclared_values(mission)
    vehicle.wings.main_wing.method_value = 2.0
    try:
        check_undeclared_values(mission)
    except AttributeError as error:
        assert 'vehicle.wings.main_wing.method_value' in str(error), str(error)
        return
    raise AssertionError('check_undeclared_values did not raise for a value a method added')


def check_method_output_container():
    # a key the class declares as None holds a method's output, whatever its contents
    mission, vehicle = build_mission()
    record_undeclared_values(mission)
    vehicle.mass_properties.weight_breakdown            = Data()
    vehicle.mass_properties.weight_breakdown.structural = 100.0
    check_undeclared_values(mission)
    return


if __name__ == '__main__':
    main()
