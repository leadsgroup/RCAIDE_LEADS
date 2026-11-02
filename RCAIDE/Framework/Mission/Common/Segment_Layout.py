# RCAIDE/Framework/Mission/Common/Segment_Layout.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Segment_Layout
# ----------------------------------------------------------------------------------------------------------------------
class Segment_Layout:
    """One contiguous float64 buffer holding a segment state's numeric leaves, each leaf rebound to a view of it.

    Parameters
    ----------
    state : RCAIDE.Framework.Mission.Common.State
        Segment state after ``expand_state`` and initialization, before converge.

    Attributes
    ----------
    buffer : numpy.ndarray
        The float64 storage, ordered unknowns, residuals, then conditions.
    slices : dict
        Slice of ``buffer`` for ``"unknowns"``, ``"residuals"``, ``"conditions"`` and each solver group
        (``"unknowns.mission"``, ``"residuals.network"``, ...).
    views : dict
        Key path -> the slot view now held at that path; aliased leaves (one array at several paths) share a slot.
    excluded : dict
        Key path -> reason, for numeric condition leaves left as they are.

    Notes
    -----
    Unknown and residual slots follow ``Data.pack_array`` order (dict order, column-major), so a solver group's
    slice equals its ``pack_array()``. Condition slots keep each array's memory order. A condition leaf that
    shares memory with a different array in the state, or is also held outside unknowns, residuals and
    conditions (e.g. in initials), is excluded, so every write still reaches the memory it reached before.
    The layout is a converge-time object; after converge the leaves are ordinary arrays.
    """

    def __init__(self, state):
        slots = []
        for group in ("unknowns", "residuals"):
            for path, v in leaves(state[group], (group,)):
                if not isinstance(v, (int, float, np.ndarray)) or (isinstance(v, np.ndarray) and v.ndim > 2):
                    continue                                       # pack_array skips these too
                if not (isinstance(v, np.ndarray) and v.dtype == np.float64):
                    raise TypeError(f"{'.'.join(path)} is packed by the solver but is not a float64 array")
                slots.append((path, v, "F"))

        conditions    = [(path, v) for path, v in leaves(state.conditions, ("conditions",))
                         if isinstance(v, np.ndarray) and v.ndim <= 2 and v.dtype.kind == "f"]
        arrays        = [(path, v) for path, v in leaves(state, ()) if isinstance(v, np.ndarray)]
        sharing       = {}
        for _, v in arrays:
            sharing.setdefault(id(root(v)), set()).add(id(v))
        outside       = {id(v) for path, v in arrays if path[0] not in ("unknowns", "residuals", "conditions")}
        self.excluded = {}
        for path, v in conditions:
            if len(sharing[id(root(v))]) > 1:
                self.excluded[path] = "shares memory with another array in the state"
            elif id(v) in outside:
                self.excluded[path] = "also held outside unknowns, residuals and conditions"
            else:
                slots.append((path, v, "F" if v.flags.f_contiguous and not v.flags.c_contiguous else "C"))

        offsets, bounds, size = {}, {}, 0
        for path, v, order in slots:
            if id(v) in offsets:
                continue
            offsets[id(v)] = (size, order)
            keys = (path[:1],) if path[0] == "conditions" else (path[:1], path[:2])   # solver groups get slices too
            for key in keys:
                low, high   = bounds.get(key, (size, size))
                bounds[key] = (min(low, size), max(high, size + v.size))
            size += v.size

        self.buffer = np.empty(size)
        self.views  = {}
        by_id       = {}
        for path, v, _ in slots:
            if id(v) not in by_id:
                start, order = offsets[id(v)]
                by_id[id(v)] = self.buffer[start:start + v.size].reshape(v.shape, order=order)
                by_id[id(v)][...] = v
            parent(state, path)[path[-1]] = by_id[id(v)]
            self.views[path] = by_id[id(v)]
        self.slices = {".".join(key): slice(*bounds[key]) for key in bounds}

    def verify(self, state):
        """Paths whose leaf is no longer its slot view, i.e. rebound to new memory since the layout was built."""
        rebound = []
        for path, view in self.views.items():
            try:
                leaf = parent(state, path)[path[-1]]
            except KeyError:
                leaf = None
            if leaf is not view:
                rebound.append(".".join(path))
        return rebound


def leaves(D, prefix):
    """(path, value) for every non-dict value of a nested Data, in dict order."""
    for k, v in D.items():
        if isinstance(v, dict):
            yield from leaves(v, prefix + (k,))
        else:
            yield prefix + (k,), v


def parent(state, path):
    """The Data that holds the leaf at path."""
    D = state
    for k in path[:-1]:
        D = D[k]
    return D


def root(v):
    """The array that owns a view's memory."""
    while isinstance(v.base, np.ndarray):
        v = v.base
    return v
