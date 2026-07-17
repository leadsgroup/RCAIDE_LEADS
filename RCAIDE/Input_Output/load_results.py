# RCAIDE/Input_Output/load_results.py
#
# Created: Jun 2026, M. Clarke

import numpy as np
import h5py
from RCAIDE.Framework.Core import Data

def load_results(filename):
    """Load mission segment conditions from an HDF5 file written by save_results.

    Parameters
    ----------
    filename : str
        Path to the HDF5 file (including extension).

    Returns
    -------
    results : Data
        A Data object keyed by segment tag.  Each value is a nested Data
        tree mirroring the segment conditions written by save_results.

    Examples
    --------
    results = load_results('my_mission.h5')
    velocity = results.cruise.frames.inertial.velocity_vector
    """
    with h5py.File(filename, 'r') as f:
        results = Data()
        for seg_tag in f.keys():
            results[seg_tag] = _read_group(f[seg_tag])
    return results


def _read_group(group):
    obj = Data()
    for key in group.keys():
        item = group[key]
        if isinstance(item, h5py.Dataset):
            obj[key] = item[()]
        elif isinstance(item, h5py.Group):
            obj[key] = _read_group(item)
    for key, val in group.attrs.items():
        obj[key] = val
    return obj
