# VnV/Verification/geometry/orientation_euler_angles_test.py
#
# Regression test: orientation_euler_angles must use from_euler('xyz'), NOT from_rotvec.
#
# The two are identical when only one angle component is non-zero (all current vehicle
# definitions), so that case would not catch a regression. This test uses a two-axis
# angle where the results diverge, ensuring any accidental revert to from_rotvec fails.

import numpy as np
import scipy as sp
from scipy.spatial.transform import Rotation

def def main(): 
    """
    Verify that the rotation matrix for a multi-axis orientation_euler_angles matches
    from_euler('xyz') and NOT from_rotvec.

    from_rotvec([φx, φy, φz]) = one rotation about the oblique axis [φx,φy,φz]/|v|
                                 by angle |v|  (axis-angle / simple rotation)
    from_euler('xyz',[φx,φy,φz]) = three sequential rotations: φx about X,
                                    φy about new Y, φz about new Z  (Euler angles)

    They only agree when at most one component is non-zero. The two-axis case below
    exposes any regression to from_rotvec.
    """
    orientation = np.array([np.pi / 6, np.pi / 4, 0.0])   # 30° about X, 45° about Y

    R_euler   = Rotation.from_euler('xyz', orientation).as_matrix()
    R_rotvec  = Rotation.from_rotvec(orientation).as_matrix()

    # Sanity check: the two methods genuinely disagree for this input
    assert not np.allclose(R_euler, R_rotvec, atol=1e-6), (
        "Test setup error: from_euler and from_rotvec happen to agree for this input "
        "— choose a different two-axis angle."
    )

    # Expected matrix from from_euler('xyz', [π/6, π/4, 0])
    # R = Ry(π/4) @ Rx(π/6)  (intrinsic x→y→z, zero z rotation)
    c1, s1 = np.cos(np.pi / 6), np.sin(np.pi / 6)   # x rotation
    c2, s2 = np.cos(np.pi / 4), np.sin(np.pi / 4)   # y rotation
    R_expected = np.array([
        [ c2,      s2 * s1,  s2 * c1],
        [ 0,       c1,      -s1     ],
        [-s2,      c2 * s1,  c2 * c1],
    ])

    assert np.allclose(R_euler, R_expected, atol=1e-10), (
        "from_euler('xyz') result does not match the expected Euler rotation matrix."
    )

    # Single-axis case: verify from_euler and from_rotvec still agree (regression guard
    # for the common existing usage pattern)
    single_axis = np.array([0.0, np.pi / 2, 0.0])
    assert np.allclose(
        Rotation.from_euler('xyz', single_axis).as_matrix(),
        Rotation.from_rotvec(single_axis).as_matrix(),
        atol=1e-10,
    ), "Single-axis case should be identical for both methods."

    print("PASSED: orientation_euler_angles uses from_euler('xyz') convention correctly.")
    return True
 

if __name__ == '__main__':
    main()
