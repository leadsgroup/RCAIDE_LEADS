# RCAIDE/Library/Mission/Common/save_results.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import h5py

# ----------------------------------------------------------------------------------------------------------------------
#  save_results
# ----------------------------------------------------------------------------------------------------------------------
def save_results(mission, filename):
    """Save mission segment conditions to an HDF5 file.

    Only the conditions data (frames, freestream, aerodynamics, energy, etc.)
    is written — vehicle and analyses objects are excluded.  Each top-level
    HDF5 group corresponds to one segment, identified by its tag.

    Parameters
    ----------
    mission  : Mission
        Evaluated mission object (return value of mission.evaluate()).
    filename : str
        Path to the output file.  A '.h5' extension is conventional.

    Returns
    -------
    None

    Examples
    --------
    results = missions.base_mission.evaluate()
    save_results(results, 'my_mission.h5')
    """
    with h5py.File(filename, 'w') as f:
        for seg_tag, segment in mission.segments.items():
            seg_group = f.create_group(seg_tag)
            _write_conditions(seg_group, segment.conditions)


def _write_conditions(group, obj):
    """Recursively write a Conditions/Data object into an HDF5 group."""
    if not isinstance(obj, dict):
        return

    for key, val in obj.items():
        key = str(key)

        if isinstance(val, np.ndarray):
            if val.size == 0:
                # preserve shape of empty arrays (e.g. uninitialised transforms)
                ds = group.create_dataset(key, shape=val.shape, dtype=val.dtype)
            else:
                group.create_dataset(key, data=val, compression='gzip', compression_opts=4)

        elif isinstance(val, (bool, np.bool_)):
            group.create_dataset(key, data=bool(val))

        elif isinstance(val, (int, float, np.integer, np.floating)):
            group.create_dataset(key, data=val)

        elif isinstance(val, str):
            group.attrs[key] = val

        elif val is None:
            pass  # None fields are uninitialised placeholders — skip

        elif isinstance(val, dict):
            sub = group.create_group(key)
            _write_conditions(sub, val)

        # anything else (functions, analysis objects, etc.) is silently skipped
