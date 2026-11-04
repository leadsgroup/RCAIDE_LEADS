# VnV/Verification/geometry/test_compute_fuel_volume.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import pytest

from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Geometry.Planform.compute_fuel_volume import tanks_in_pack_order


def tank(tag, packs_after=None):
    return Data(tag=tag, packs_after_tank=packs_after)


def test_dependencies_are_placed_first():
    tanks = [tank('outer', 'middle'), tank('middle', 'inner'), tank('inner'), tank('other', 'missing')]
    assert [t.tag for t in tanks_in_pack_order(tanks)] == ['inner', 'middle', 'outer', 'other']


def test_circular_chain_keeps_every_tank_once():
    tanks = [tank('a', 'b'), tank('b', 'a')]
    with pytest.warns(UserWarning, match="circular packs_after_tank"):
        ordered = tanks_in_pack_order(tanks)
    assert sorted(t.tag for t in ordered) == ['a', 'b']
