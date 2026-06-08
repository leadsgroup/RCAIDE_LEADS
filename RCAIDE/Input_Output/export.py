# RCAIDE/Input_Output/export.py
#
# Created: May 2026, M. Clarke

import json
import numpy as np
import types as _types_module
from collections import OrderedDict

_SKIP_KEYS = frozenset({
    '_component_root_map',
    '_energy_network_root_map',
    '_base',
    '_diff',
    'vehicle',
})


def _is_unit_argument_pair(value):
    return (
        isinstance(value, list)
        and len(value) == 2
        and isinstance(value[1], int)
        and not isinstance(value[1], bool)
    )


def _make_json_safe(value):
    if isinstance(value, dict) and hasattr(value, 'items'):
        safe = OrderedDict()
        for key, item in value.items():
            if isinstance(key, type):
                key = key.__name__
            safe[str(key)] = _make_json_safe(item)
        return safe
    if isinstance(value, list):
        return [_make_json_safe(item) for item in value]
    if isinstance(value, tuple):
        return [_make_json_safe(item) for item in value]
    return value


def _add_unit_arguments(value):
    if isinstance(value, dict) and hasattr(value, 'items'):
        wrapped = OrderedDict()
        for key, item in value.items():
            wrapped[key] = _add_unit_arguments(item)
        return wrapped
    if _is_unit_argument_pair(value):
        return value
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        safe_items = [_make_json_safe(item) for item in value]
        if all(isinstance(item, dict) and hasattr(item, 'items') for item in safe_items):
            return [_add_unit_arguments(item) for item in safe_items]
        return [safe_items, 0]
    if isinstance(value, tuple):
        return [_make_json_safe(value), 0]
    return [value, 0]


def _build_dict_r(v):
    tv = type(v)
    if tv is type or tv is _types_module.FunctionType:
        return None
    if tv in (str, bool) or tv is type(None):
        return v
    if tv in (float, int):
        return v
    if tv is np.ndarray or tv is np.float64:
        return v.tolist()
    if tv is list:
        return v
    try:
        keys = v.keys()
    except AttributeError:
        return None if callable(tv) else None
    ret = {}
    module   = getattr(tv, '__module__',  '') or ''
    qualname = getattr(tv, '__qualname__', '') or ''
    if module and qualname and not qualname.startswith('<'):
        ret['__type__'] = f"{module}.{qualname}"
    for k in keys:
        if k in _SKIP_KEYS:
            continue
        ret[k] = _build_dict_r(v[k])
    return ret


def _serialise(obj):
    d = {}
    for k in obj.keys():
        if k in _SKIP_KEYS:
            continue
        d[k] = _build_dict_r(obj[k])
    return _add_unit_arguments(_make_json_safe(d))


def _serialise_config_entry(config):
    config.store_diff()
    diff = config._diff
    diff_raw = {}
    for k in diff.keys():
        if k in _SKIP_KEYS:
            continue
        diff_raw[k] = _build_dict_r(diff[k])
    diff_serialised = _add_unit_arguments(_make_json_safe(diff_raw))
    tv = type(config)
    module   = getattr(tv, '__module__',  '') or ''
    qualname = getattr(tv, '__qualname__', '') or ''
    return {
        "__type__": f"{module}.{qualname}",
        "tag":      config.tag,
        "diff":     diff_serialised,
    }


_SEGMENT_SKIP_KEYS = frozenset(_SKIP_KEYS | {'analyses', 'state', 'conditions', 'process'})


def _build_dict_r_segment(v):
    tv = type(v)
    if tv is type or tv is _types_module.FunctionType:
        return None
    if tv in (str, bool) or tv is type(None):
        return v
    if tv in (float, int):
        return v
    if tv is np.ndarray or tv is np.float64:
        return v.tolist()
    if tv is list:
        return v
    try:
        keys = v.keys()
    except AttributeError:
        return None if callable(tv) else None
    ret = {}
    module   = getattr(tv, '__module__', '') or ''
    qualname = getattr(tv, '__qualname__', '') or ''
    if module and qualname and not qualname.startswith('<'):
        ret['__type__'] = f"{module}.{qualname}"
    for k in keys:
        if k in _SEGMENT_SKIP_KEYS:
            continue
        ret[k] = _build_dict_r_segment(v[k])
    return ret


_ANALYSIS_DIFF_SKIP = frozenset(_SKIP_KEYS | {'process', 'tag'})


def _analysis_settings_diff(actual, default):
    result = {}
    if not hasattr(actual, 'keys') or not hasattr(default, 'keys'):
        return result
    for k in actual.keys():
        if k in _ANALYSIS_DIFF_SKIP:
            continue
        v_act = actual[k]
        if callable(v_act) or isinstance(v_act, type):
            continue
        try:
            v_def = default[k]
        except (KeyError, AttributeError):
            serialized = _build_dict_r(v_act)
            if serialized is not None:
                result[k] = serialized
            continue
        if callable(v_def) or isinstance(v_def, type):
            continue
        if hasattr(v_act, 'keys') and hasattr(v_def, 'keys'):
            sub_diff = _analysis_settings_diff(v_act, v_def)
            if sub_diff:
                result[k] = sub_diff
        else:
            try:
                equal = bool(np.array_equal(np.asarray(v_act), np.asarray(v_def)))
            except Exception:
                try:
                    equal = (v_act == v_def)
                except Exception:
                    equal = False
            if not equal:
                result[k] = _build_dict_r(v_act)
    return result


def _build_analysis_data(analyses):
    result = []
    for tag, analysis_vehicle in analyses.items():
        sub_analyses = []
        try:
            for k in analysis_vehicle.keys():
                if k in _SKIP_KEYS or k == 'tag':
                    continue
                sub = analysis_vehicle[k]
                tv       = type(sub)
                module   = getattr(tv, '__module__',  '') or ''
                qualname = getattr(tv, '__qualname__', '') or ''
                if not (module and qualname and not qualname.startswith('<')):
                    continue
                type_str = f"{module}.{qualname}"
                entry = {'__type__': type_str}
                try:
                    default_sub = tv()
                    diff = _analysis_settings_diff(sub, default_sub)
                    if diff:
                        entry['diff'] = _add_unit_arguments(_make_json_safe(diff))
                except Exception:
                    pass
                sub_analyses.append(entry)
        except Exception:
            pass
        result.append({'tag': str(tag), 'sub_analyses': sub_analyses})
    return result


def _build_mission_data(missions):
    result = []
    for mission_tag, mission in missions.items():
        if isinstance(mission, str):
            continue
        tv = type(mission)
        module   = getattr(tv, '__module__', '') or ''
        qualname = getattr(tv, '__qualname__', '') or ''
        segs = []
        if hasattr(mission, 'segments'):
            for seg_tag, seg in mission.segments.items():
                seg_dict = _build_dict_r_segment(seg) or {}
                seg_dict['tag'] = str(seg_tag)
                config_tag = ''
                try:
                    av  = seg.analyses
                    veh = getattr(av, 'vehicle', None)
                    config_tag = (getattr(veh, 'tag', '') or
                                  getattr(av,  'tag', '') or '')
                except Exception:
                    pass
                if config_tag:
                    seg_dict['config_tag'] = config_tag
                segs.append(seg_dict)
        entry = {
            "__type__":    f"{module}.{qualname}",
            "mission_tag": str(mission_tag),
            "segments":    segs,
        }
        result.append(_add_unit_arguments(_make_json_safe(entry)))
    return result


def export(vehicle, configurations, analyses, missions, filename):
    """Export a RCAIDE study to a compact, type-annotated JSON file.

    Parameters
    ----------
    vehicle        : RCAIDE.Vehicle
    configurations : Config.Container
    analyses       : Analysis.Container
    missions       : RCAIDE.Framework.Mission.Missions
    filename       : str  (written to filename + '.json')
    """
    config_data = [_serialise_config_entry(cfg) for _, cfg in configurations.items()]
    data = {
        "rcaide_vehicle": _serialise(vehicle),
        "config_data":    config_data,
        "analysis_data":  _build_analysis_data(analyses),
        "mission_data":   _build_mission_data(missions),
    }
    with open(filename + '.json', 'w') as f:
        json.dump(data, f, indent=4)
