# populate_control_sections_test.py
#
# Created: Jul 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Geometry.Planform.populate_control_sections import populate_control_sections

import numpy as np
import time

# ----------------------------------------------------------------------------------------------------------------------
#  Wing segments used across all tests: root(0.0), sec1(0.3), sec2(0.7), tip(1.0)
#  Segment indices after calling list(wing.segments.values()):
#    [0] root  — always skipped by populate_control_sections
#    [1] sec1  — i=1: prev=0.0, current=0.3
#    [2] sec2  — i=2: prev=0.3, current=0.7
#    [3] tip   — i=3: prev=0.7, current=1.0
# ----------------------------------------------------------------------------------------------------------------------
SEG_LOCS = [0.0, 0.3, 0.7, 1.0]

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    ti = time.time()

    test_case_6_contained()
    test_case_7_ends_at_boundary()
    test_case_8_and_1_straddles_boundary()
    test_case_1_starts_before_prev()
    test_case_2_ends_at_current()
    test_case_3_spans_full_segment()
    test_case_4_starts_at_prev()
    test_case_5_full_segment_match()
    test_else_no_overlap()
    test_multiple_control_surfaces()

    elapsed_time = time.time() - ti
    print(f'\nElapsed time (min): {elapsed_time / 60:.4f}')
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_wing(seg_spans, cs_list):
    wing = RCAIDE.Library.Components.Wings.Main_Wing()
    for i, loc in enumerate(seg_spans):
        seg = RCAIDE.Library.Components.Wings.Segments.Segment()
        seg.tag = f'seg_{i}'
        seg.percent_span_location = loc
        wing.segments.append(seg)
    for cs in cs_list:
        wing.control_surfaces.append(cs)
    return wing

def make_aileron(tag, sf_start, sf_end, span=1.0):
    cs = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    cs.tag                = tag
    cs.span_fraction_start = sf_start
    cs.span_fraction_end   = sf_end
    cs.span               = span
    return cs

def get_segs(wing):
    return list(wing.segments.values())

def get_cs(seg, idx=0):
    return list(seg.control_surfaces.values())[idx]

# ----------------------------------------------------------------------------------------------------------------------
#  Individual case tests
# ----------------------------------------------------------------------------------------------------------------------
def test_case_6_contained():
    """Case 6: sf[0] > prev AND sf[1] < current — CS entirely within sec2."""
    print('\n--- Case 6: CS entirely within sec2 [0.3, 0.7] ---')
    wing = build_wing(SEG_LOCS, [make_aileron('cs6', 0.35, 0.65)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 0
    assert len(segs[2].control_surfaces) == 1
    assert len(segs[3].control_surfaces) == 0

    cs = get_cs(segs[2])
    assert np.isclose(cs.span_fraction_start, 0.35)
    assert np.isclose(cs.span_fraction_end,   0.65)
    assert np.isclose(cs.span, 1.0)
    print('  PASSED')


def test_case_7_ends_at_boundary():
    """Case 7: sf[0] > prev AND sf[1] == current — CS ends exactly at the sec2 boundary."""
    print('\n--- Case 7: CS ends exactly at sec2 boundary (0.7) ---')
    wing = build_wing(SEG_LOCS, [make_aileron('cs7', 0.35, 0.70)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 0
    assert len(segs[2].control_surfaces) == 1
    assert len(segs[3].control_surfaces) == 0

    cs = get_cs(segs[2])
    assert np.isclose(cs.span_fraction_start, 0.35)
    assert np.isclose(cs.span_fraction_end,   0.70)
    assert np.isclose(cs.span, 1.0)
    print('  PASSED')


def test_case_8_and_1_straddles_boundary():
    """Case 8: sf[0] within segment, sf[1] extends beyond current — clips to [sf[0], current].
    Case 1: sf[0] before prev, sf[1] within segment — clips to [prev, sf[1]].
    Both fire when a CS straddles a segment boundary."""
    print('\n--- Cases 8 & 1: CS straddles sec2/tip boundary ---')
    cs_span = 1.0
    sf = (0.35, 0.80)
    wing = build_wing(SEG_LOCS, [make_aileron('cs81', *sf, span=cs_span)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 0
    assert len(segs[2].control_surfaces) == 1  # Case 8
    assert len(segs[3].control_surfaces) == 1  # Case 1

    cs_sec2 = get_cs(segs[2])
    assert np.isclose(cs_sec2.span_fraction_start, 0.35)
    assert np.isclose(cs_sec2.span_fraction_end,   0.70)
    assert np.isclose(cs_sec2.span, cs_span * (0.70 - 0.35) / (sf[1] - sf[0]))

    cs_tip = get_cs(segs[3])
    assert np.isclose(cs_tip.span_fraction_start, 0.70)
    assert np.isclose(cs_tip.span_fraction_end,   0.80)
    assert np.isclose(cs_tip.span, cs_span * (0.80 - 0.70) / (sf[1] - sf[0]))
    print('  PASSED')


def test_case_1_starts_before_prev():
    """Case 1 isolated: CS starts before prev, ends strictly within (prev, current)."""
    print('\n--- Case 1: CS starts before sec1, ends within sec2 ---')
    cs_span = 1.0
    sf = (0.10, 0.50)
    wing = build_wing(SEG_LOCS, [make_aileron('cs1', *sf, span=cs_span)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 1  # Case 8
    assert len(segs[2].control_surfaces) == 1  # Case 1
    assert len(segs[3].control_surfaces) == 0

    cs_sec1 = get_cs(segs[1])
    assert np.isclose(cs_sec1.span_fraction_start, 0.10)
    assert np.isclose(cs_sec1.span_fraction_end,   0.30)
    assert np.isclose(cs_sec1.span, cs_span * (0.30 - 0.10) / (sf[1] - sf[0]))

    cs_sec2 = get_cs(segs[2])
    assert np.isclose(cs_sec2.span_fraction_start, 0.30)
    assert np.isclose(cs_sec2.span_fraction_end,   0.50)
    assert np.isclose(cs_sec2.span, cs_span * (0.50 - 0.30) / (sf[1] - sf[0]))
    print('  PASSED')


def test_case_2_ends_at_current():
    """Case 2: sf[0] < prev AND sf[1] == current — clips to [prev, current]."""
    print('\n--- Case 2: CS starts before sec1, ends exactly at sec2 boundary ---')
    cs_span = 1.0
    sf = (0.10, 0.70)
    wing = build_wing(SEG_LOCS, [make_aileron('cs2', *sf, span=cs_span)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 1  # Case 8
    assert len(segs[2].control_surfaces) == 1  # Case 2
    assert len(segs[3].control_surfaces) == 0

    cs_sec2 = get_cs(segs[2])
    assert np.isclose(cs_sec2.span_fraction_start, 0.30)
    assert np.isclose(cs_sec2.span_fraction_end,   0.70)
    assert np.isclose(cs_sec2.span, cs_span * (0.70 - 0.30) / (sf[1] - sf[0]))
    print('  PASSED')


def test_case_3_spans_full_segment():
    """Case 3: sf[0] < prev AND sf[1] > current — clips to full [prev, current]."""
    print('\n--- Case 3: CS overflows both ends of sec2 ---')
    cs_span = 1.0
    sf = (0.10, 0.80)
    wing = build_wing(SEG_LOCS, [make_aileron('cs3', *sf, span=cs_span)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 1  # Case 8
    assert len(segs[2].control_surfaces) == 1  # Case 3
    assert len(segs[3].control_surfaces) == 1  # Case 1

    cs_sec2 = get_cs(segs[2])
    assert np.isclose(cs_sec2.span_fraction_start, 0.30)
    assert np.isclose(cs_sec2.span_fraction_end,   0.70)
    assert np.isclose(cs_sec2.span, cs_span * (0.70 - 0.30) / (sf[1] - sf[0]))
    print('  PASSED')


def test_case_4_starts_at_prev():
    """Case 4: sf[0] == prev AND sf[1] < current — CS starts exactly at segment start."""
    print('\n--- Case 4: CS starts at sec2 lower boundary, ends within sec2 ---')
    cs_span = 1.0
    sf = (0.30, 0.60)
    wing = build_wing(SEG_LOCS, [make_aileron('cs4', *sf, span=cs_span)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 0  # sf[0] == current of sec1 → else
    assert len(segs[2].control_surfaces) == 1  # Case 4
    assert len(segs[3].control_surfaces) == 0

    cs = get_cs(segs[2])
    assert np.isclose(cs.span_fraction_start, 0.30)
    assert np.isclose(cs.span_fraction_end,   0.60)
    assert np.isclose(cs.span, 1.0)
    print('  PASSED')


def test_case_5_full_segment_match():
    """Case 5: sf[0] == prev AND sf[1] == current — CS matches segment exactly."""
    print('\n--- Case 5: CS matches sec2 span exactly ---')
    sf = (0.30, 0.70)
    wing = build_wing(SEG_LOCS, [make_aileron('cs5', *sf)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 0
    assert len(segs[2].control_surfaces) == 1  # Case 5
    assert len(segs[3].control_surfaces) == 0

    cs = get_cs(segs[2])
    assert np.isclose(cs.span_fraction_start, 0.30)
    assert np.isclose(cs.span_fraction_end,   0.70)
    assert np.isclose(cs.span, 1.0)
    print('  PASSED')


def test_else_no_overlap():
    """else: CS bounds lie entirely outside a segment — no assignment to that segment."""
    print('\n--- else: sec1 and tip receive no CS when CS is contained in sec2 ---')
    # cs [0.35, 0.65] falls in sec2 only; sec1 and tip get else (no overlap)
    wing = build_wing(SEG_LOCS, [make_aileron('cs_else', 0.35, 0.65)])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    # sec1 (i=1, prev=0.0, current=0.3): sf[0]=0.35 > current=0.3 → none of Cases 6/7/8 fire → else
    assert len(segs[1].control_surfaces) == 0, 'sec1 should not receive a CS starting beyond its boundary'
    # tip (i=3, prev=0.7, current=1.0): sf[1]=0.65 < prev=0.7 → no overlap → else
    assert len(segs[3].control_surfaces) == 0, 'tip should not receive a CS ending before its start'
    print('  PASSED')


def test_multiple_control_surfaces():
    """Verify multiple control surfaces are independently distributed to the correct segments."""
    print('\n--- Multiple CS: flap on sec2, aileron spanning sec2+tip ---')
    flap    = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                = 'flap'
    flap.span_fraction_start = 0.30
    flap.span_fraction_end   = 0.70
    flap.span               = 10.0

    aileron = make_aileron('aileron', 0.70, 1.00, span=5.0)

    wing = build_wing(SEG_LOCS, [flap, aileron])
    wing = populate_control_sections(wing)
    segs = get_segs(wing)

    assert len(segs[1].control_surfaces) == 0
    assert len(segs[2].control_surfaces) == 1   # flap (Case 5)
    assert len(segs[3].control_surfaces) == 1   # aileron (Case 4: sf[0]==prev, sf[1]<current... no, 1.0==1.0)

    cs_flap = get_cs(segs[2])
    assert cs_flap.tag == 'flap'
    assert np.isclose(cs_flap.span_fraction_start, 0.30)
    assert np.isclose(cs_flap.span_fraction_end,   0.70)

    # aileron: sf=[0.70, 1.00], tip segment (prev=0.7, current=1.0)
    # sf[0]=0.70 == prev, sf[1]=1.00 == current → Case 5
    cs_ail = get_cs(segs[3])
    assert cs_ail.tag == 'aileron'
    assert np.isclose(cs_ail.span_fraction_start, 0.70)
    assert np.isclose(cs_ail.span_fraction_end,   1.00)
    assert np.isclose(cs_ail.span, 5.0)
    print('  PASSED')


if __name__ == '__main__':
    main()
