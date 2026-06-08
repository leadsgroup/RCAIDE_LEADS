# RCAIDE/Input_Output/import_data.py
#
# Created: May 2026, M. Clarke

import json
import importlib
import numpy as np
from collections import OrderedDict

import RCAIDE
from RCAIDE.Framework.Core import Data, DataOrdered
from RCAIDE.Input_Output.load import read_RCAIDE_json_dict


def _is_mapping(v):
    return hasattr(v, 'items') and hasattr(v, '__setitem__')


def _has_key(v, key):
    return hasattr(v, 'keys') and key in v.keys()


def _class_for_type_string(type_str):
    if not type_str or not isinstance(type_str, str):
        return None
    parts = type_str.split('.')
    for i in range(len(parts) - 1, 0, -1):
        module_path = '.'.join(parts[:i])
        attr_chain  = parts[i:]
        try:
            mod = importlib.import_module(module_path)
            obj = mod
            for attr in attr_chain:
                obj = getattr(obj, attr)
            if isinstance(obj, type):
                return obj
        except Exception:
            continue
    return None


def _make_typed_component(data, fallback_cls=None):
    if fallback_cls is not None and isinstance(data, fallback_cls):
        return data
    ts = data.get('__type__') if hasattr(data, 'get') else None
    if ts:
        cls = _class_for_type_string(ts)
        if cls is not None and cls is not Data and cls is not DataOrdered:
            if not (getattr(cls, '__module__', '') or '').startswith('RCAIDE.Framework.Core'):
                try:
                    obj = cls()
                    obj.update(data)
                    return obj
                except Exception:
                    pass
    if fallback_cls is not None:
        try:
            obj = fallback_cls()
            obj.update(data)
            return obj
        except Exception:
            pass
    return data


_VEHICLE_CONTAINER_TABLE = [
    ('fuselages',     RCAIDE.Library.Components.Fuselages.Fuselage,        '_component_root_map'),
    ('booms',         RCAIDE.Library.Components.Booms.Boom,                '_component_root_map'),
    ('landing_gears', RCAIDE.Library.Components.Landing_Gear.Landing_Gear, '_component_root_map'),
    ('cargo_bays',    RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay,      '_component_root_map'),
    ('wings',         RCAIDE.Library.Components.Wings.Wing,                '_component_root_map'),
    ('nacelles',      RCAIDE.Library.Components.Nacelles.Nacelle,          '_component_root_map'),
    ('networks',      RCAIDE.Framework.Networks.Network,                   '_energy_network_root_map'),
]


def _restore_vehicle_components(vehicle_obj):
    for container_key, base_cls, map_attr in _VEHICLE_CONTAINER_TABLE:
        if not _has_key(vehicle_obj, container_key) or not _is_mapping(vehicle_obj[container_key]):
            continue
        container_cls = getattr(base_cls, 'Container', None)
        if container_cls is None:
            continue
        restored = container_cls()
        for _key, item in list(vehicle_obj[container_key].items()):
            if not _is_mapping(item):
                continue
            restored.append(_make_typed_component(item, base_cls))
        vehicle_obj[container_key] = restored
        root_map = getattr(vehicle_obj, map_attr, None)
        if root_map is not None:
            root_map[base_cls] = restored


def _is_serialized_airfoil(v):
    return _is_mapping(v) and (
        _has_key(v, 'coordinate_file') or _has_key(v, 'NACA_4_Series_code')
    )


def _restore_airfoil_components(value):
    if _is_mapping(value):
        for key, item in list(value.items()):
            if key == 'airfoil' and _is_serialized_airfoil(item):
                value[key] = _make_typed_component(
                    item, RCAIDE.Library.Components.Airfoils.Airfoil
                )
            else:
                _restore_airfoil_components(item)
    elif isinstance(value, list):
        for item in value:
            _restore_airfoil_components(item)


_RESTORE_SKIP = frozenset({'__type__', '_component_root_map', '_energy_network_root_map'})


def _restore_typed_subcomponents(obj):
    if not _is_mapping(obj):
        return
    for key in list(obj.keys()):
        if key in _RESTORE_SKIP:
            continue
        child = obj[key]
        if not _is_mapping(child):
            continue
        _restore_typed_subcomponents(child)
        if type(child) is not DataOrdered:
            continue
        ts = child.get('__type__') if hasattr(child, 'get') else None
        if not ts or not isinstance(ts, str):
            continue
        cls = _class_for_type_string(ts)
        if cls is None or cls is Data or cls is DataOrdered:
            continue
        if (getattr(cls, '__module__', '') or '').startswith('RCAIDE.Framework.Core'):
            continue
        try:
            new_obj = cls()
            for k, v in child.items():
                if k != '__type__':
                    new_obj[k] = v
            obj[key] = new_obj
        except Exception:
            pass


def _strip_unit_arguments(value):
    if _is_mapping(value):
        clean = OrderedDict()
        for key, item in value.items():
            clean[key] = _strip_unit_arguments(item)
        return clean
    if (isinstance(value, list) and len(value) == 2
            and isinstance(value[1], int) and not isinstance(value[1], bool)):
        return _strip_unit_arguments(value[0])
    if isinstance(value, list):
        return [_strip_unit_arguments(item) for item in value]
    return value


def vehicle_setup(vehicle_raw):
    vehicle_clean = _strip_unit_arguments(vehicle_raw)
    vehicle = RCAIDE.Vehicle()
    vehicle.update(read_RCAIDE_json_dict(vehicle_clean))
    _restore_vehicle_components(vehicle)
    _restore_airfoil_components(vehicle)
    _restore_typed_subcomponents(vehicle)
    return vehicle


def _coerce_leaf(new_val, obj, key):
    if isinstance(new_val, list):
        try:
            if isinstance(obj[key], np.ndarray):
                return np.array(new_val)
        except Exception:
            pass
    return new_val


def _apply_diff(obj, diff_dict):
    for key, value in diff_dict.items():
        if key == '__type__':
            continue
        if isinstance(value, (dict, OrderedDict)):
            try:
                child = obj[key]
                _apply_diff(child, value)
            except (KeyError, TypeError, AttributeError):
                pass
        else:
            try:
                obj[key] = _coerce_leaf(value, obj, key)
            except (KeyError, TypeError, AttributeError):
                pass


def configs_setup(config_data, base_vehicle):
    from RCAIDE.Library.Components.Configs.Config import Config
    configs = Config.Container()
    for entry in config_data:
        if not isinstance(entry, dict):
            continue
        name = entry.get('tag') or entry.get('config name', '')
        if not name:
            continue
        config = Config(base_vehicle)
        config.tag = name
        diff_raw = entry.get('diff', {})
        if diff_raw:
            diff_clean = _strip_unit_arguments(diff_raw)
            _apply_diff(config, diff_clean)
        configs.append(config)
    return configs


def _apply_analysis_settings(obj, data_dict):
    _skip = frozenset({'__type__', 'vehicle'})
    for key, value in data_dict.items():
        if key in _skip:
            continue
        if isinstance(value, (dict, OrderedDict)):
            try:
                child = obj[key]
                _apply_analysis_settings(child, value)
            except (KeyError, TypeError, AttributeError):
                try:
                    obj[key] = value
                except Exception:
                    pass
        else:
            try:
                obj[key] = _coerce_leaf(value, obj, key)
            except (KeyError, TypeError, AttributeError):
                pass


def base_analysis(vehicle, sub_analyses):
    analysis = RCAIDE.Framework.Analyses.Vehicle()
    analysis['vehicle'] = vehicle
    for entry in sub_analyses:
        ts = entry.get('__type__', '')
        if not ts:
            continue
        cls = _class_for_type_string(ts)
        if cls is None:
            continue
        try:
            sub = cls()
            sub['vehicle'] = vehicle
            diff_raw = entry.get('diff', {})
            if diff_raw:
                diff_clean = _strip_unit_arguments(diff_raw)
                _apply_analysis_settings(sub, diff_clean)
            analysis.append(sub)
        except Exception:
            pass
    return analysis


def analyses_setup(analysis_data, configurations):
    from RCAIDE.Framework.Analyses.Analysis import Analysis
    analyses = Analysis.Container()
    for entry in analysis_data:
        if not isinstance(entry, dict):
            continue
        tag = entry.get('tag', '')
        if not tag:
            continue
        config = None
        for cfg_tag, cfg in configurations.items():
            if cfg_tag == tag or getattr(cfg, 'tag', '') == tag:
                config = cfg
                break
        if config is None:
            continue
        analysis     = base_analysis(config, entry.get('sub_analyses', []))
        analysis.tag = tag
        analyses[tag] = analysis
    return analyses


def _apply_segment_settings(obj, data_dict):
    _skip = frozenset({'__type__', 'tag', 'config_tag', 'analyses', 'vehicle', 'process'})
    for key, value in data_dict.items():
        if key in _skip:
            continue
        if isinstance(value, (dict, OrderedDict)):
            try:
                child = obj[key]
                _apply_segment_settings(child, value)
            except (KeyError, TypeError, AttributeError):
                try:
                    obj[key] = value
                except Exception:
                    pass
        else:
            try:
                obj[key] = _coerce_leaf(value, obj, key)
            except (KeyError, TypeError, AttributeError):
                pass


def missions_setup(mission_data, analyses=None):
    missions = RCAIDE.Framework.Mission.Missions()
    for entry in mission_data:
        if not isinstance(entry, dict):
            continue
        clean       = _strip_unit_arguments(entry)
        mission_tag = clean.get('mission_tag', clean.get('tag', 'mission'))
        ts          = clean.get('__type__', '')
        mission_cls = _class_for_type_string(ts) if ts else None
        try:
            mission = mission_cls() if mission_cls else RCAIDE.Framework.Mission.Sequential_Segments()
        except Exception:
            mission = RCAIDE.Framework.Mission.Sequential_Segments()
        mission.tag = mission_tag
        for seg_info in clean.get('segments', []):
            seg_tag      = seg_info.get('tag', '')
            seg_type_str = seg_info.get('__type__', '')
            config_tag   = seg_info.get('config_tag', '')
            seg = None
            if seg_type_str:
                cls = _class_for_type_string(seg_type_str)
                if cls is not None:
                    try:
                        seg     = cls()
                        seg.tag = seg_tag
                    except Exception:
                        seg = None
            if seg is None:
                seg     = RCAIDE.Framework.Mission.Segments.Segment()
                seg.tag = seg_tag
            _apply_segment_settings(seg, seg_info)
            if config_tag and analyses is not None:
                try:
                    seg.analyses.extend(analyses[config_tag])
                except Exception:
                    pass
            mission.append_segment(seg)
        missions.append(mission)
    return missions


def import_data(filename):
    """Load a RCAIDE study previously saved by RCAIDE.io.export.

    Parameters
    ----------
    filename : str  (reads from filename + '.json')

    Returns
    -------
    result : RCAIDE.Framework.Core.Data
        Attributes: vehicle, configurations, analyses, missions
    """
    with open(filename + '.json') as f:
        raw = json.load(f, object_pairs_hook=OrderedDict)

    vehicle        = vehicle_setup(raw['rcaide_vehicle'])
    configurations = configs_setup(raw.get('config_data', []), vehicle)
    analyses       = analyses_setup(raw.get('analysis_data', []), configurations)
    missions       = missions_setup(raw.get('mission_data', []), analyses)

    result                = Data()
    result.vehicle        = vehicle
    result.configurations = configurations
    result.analyses       = analyses
    result.missions       = missions
    return result
