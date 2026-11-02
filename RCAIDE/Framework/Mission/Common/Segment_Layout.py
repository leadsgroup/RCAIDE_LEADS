# RCAIDE/Framework/Mission/Common/Segment_Layout.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import collections

import numpy as np

from .Conditions import Conditions

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
    owned : set
        ids of the slot views held at exactly one path; assigning a same-shape float array over one of these
        writes into it (``Conditions.__setitem__``).

    Notes
    -----
    Unknown and residual slots follow ``Data.pack_array`` order (dict order, column-major), so a solver group's
    slice equals its ``pack_array()``; a scalar unknown or residual gets a one-entry rank-0 view. Condition slots,
    of any rank (e.g. one 3x3 transform per row), keep each array's memory order. A condition leaf whose memory
    overlaps a different array in the state, or that is also held outside unknowns, residuals and conditions (e.g.
    in initials), is excluded, so every write still reaches the memory it reached before. The layout is a
    converge-time object; after converge the leaves are ordinary arrays.
    """

    def __init__(self, state):
        slots = []
        for group in ("unknowns", "residuals"):
            for path, v in leaves(state[group], (group,)):
                if not isinstance(v, (int, float, np.ndarray)) or (isinstance(v, np.ndarray) and v.ndim > 2):
                    continue                                       # pack_array skips these too
                if isinstance(v, bool) or (isinstance(v, np.ndarray) and v.dtype != np.float64):
                    raise TypeError(f"{'.'.join(path)} is packed by the solver but is not a float64 array or number")
                slots.append((path, v, "F"))

        conditions    = [(path, v) for path, v in leaves(state.conditions, ("conditions",))
                         if isinstance(v, np.ndarray) and v.dtype.kind == "f"]
        arrays        = [(path, v) for path, v in leaves(state, ()) if isinstance(v, np.ndarray)]
        overlapping   = overlaps([v for _, v in arrays])
        outside       = {id(v) for path, v in arrays if path[0] not in ("unknowns", "residuals", "conditions")}
        self.excluded = {}
        for path, v in conditions:
            if id(v) in overlapping:
                self.excluded[path] = "memory overlaps another array in the state"
            elif id(v) in outside:
                self.excluded[path] = "also held outside unknowns, residuals and conditions"
            else:
                slots.append((path, v, "F" if v.flags.f_contiguous and not v.flags.c_contiguous else "C"))

        # one slot per array object; a number is never shared, since equal numbers can be one object
        key = lambda path, v: id(v) if isinstance(v, np.ndarray) else path
        offsets, bounds, size = {}, {}, 0
        for path, v, order in slots:
            if key(path, v) in offsets:
                continue
            offsets[key(path, v)] = (size, order)
            keys = (path[:1],) if path[0] == "conditions" else (path[:1], path[:2])   # solver groups get slices too
            for k in keys:
                low, high = bounds.get(k, (size, size))
                bounds[k] = (min(low, size), max(high, size + np.size(v)))
            size += np.size(v)

        self.buffer = np.empty(size)
        self.views  = {}
        by_key      = {}
        for path, v, _ in slots:
            if key(path, v) not in by_key:
                start, order = offsets[key(path, v)]
                view = self.buffer[start:start + np.size(v)].reshape(np.shape(v), order=order)
                view[...] = v
                by_key[key(path, v)] = view
            dict.__setitem__(parent(state, path), path[-1], by_key[key(path, v)])   # bypasses the storage rule
            self.views[path] = by_key[key(path, v)]
        self.slices = {".".join(k): slice(*bounds[k]) for k in bounds}

        # aliased slots keep today's semantics (assignment rebinds one name), so only single-path slots are owned
        counts     = collections.Counter(id(view) for view in self.views.values())
        self.owned = {id(view) for view in self.views.values() if counts[id(view)] == 1}
        for path in self.views:
            node = parent(state, path)
            if isinstance(node, Conditions):
                object.__setattr__(node, 'segment_layout', self)

    def owns(self, v):
        """Whether v is one of this layout's single-path slot views."""
        return id(v) in self.owned

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        """A layout binds one state's memory; a copied state shares the reference but owns none of its slots."""
        return self

    def __reduce__(self):
        """Saved states are plain data: a pickled layout loads as None."""
        return (no_layout, ())

    def verify(self, state, group=None):
        """Paths (only those under one top-level group, e.g. "residuals", if given) whose leaf is no longer its
        slot view, i.e. rebound to new memory since the layout was built."""
        rebound = []
        for path, view in self.views.items():
            if group is not None and path[0] != group:
                continue
            try:
                leaf = parent(state, path)[path[-1]]
            except KeyError:
                leaf = None
            if leaf is not view:
                rebound.append(".".join(path))
        return rebound

    def added(self, state):
        """Paths of float array leaves in unknowns, residuals or conditions that the layout does not know, i.e.
        created after it was built."""
        known = set(self.views) | set(self.excluded)
        return [".".join(path) for group in ("unknowns", "residuals", "conditions")
                for path, v in leaves(state[group], (group,))
                if isinstance(v, np.ndarray) and v.dtype.kind == "f" and path not in known]


def no_layout():
    """What a pickled layout loads as."""
    return None


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


def overlaps(arrays):
    """ids of arrays whose memory overlaps a different array in the list, by a sweep over byte ranges per owner."""
    by_root = {}
    for v in arrays:
        if v.size:
            by_root.setdefault(id(root(v)), {})[id(v)] = v
    found = set()
    for group in by_root.values():
        spans = sorted(byte_range(v) + (k,) for k, v in group.items())
        reach, reach_key = -1, None
        for low, high, k in spans:
            if low < reach:
                found.update((k, reach_key))
            if high > reach:
                reach, reach_key = high, k
    return found


def byte_range(v):
    """First and one-past-last byte an array can touch."""
    start = v.__array_interface__["data"][0]
    low   = start + sum(min(0, (n - 1) * s) for n, s in zip(v.shape, v.strides))
    high  = start + sum(max(0, (n - 1) * s) for n, s in zip(v.shape, v.strides)) + v.itemsize
    return low, high
