import importlib.abc
import importlib.machinery
import importlib.metadata
import re
import sys
import types

_ENTRY_POINT_GROUP = "mujoco_ros.plugins"
_REGISTRY = {}


def _entry_points():
    entry_points = importlib.metadata.entry_points()
    if hasattr(entry_points, "select"):
        return list(entry_points.select(group=_ENTRY_POINT_GROUP))
    return list(entry_points.get(_ENTRY_POINT_GROUP, []))


def _sanitize_name(name):
    sanitized = re.sub(r"[^0-9A-Za-z_]", "_", str(name))
    if sanitized and sanitized[0].isdigit():
        sanitized = "_" + sanitized
    return sanitized


def _candidate_names(plugin):
    names = []
    for value in (getattr(plugin, "type", ""), getattr(plugin, "name", "")):
        if not value:
            continue
        names.append(value)
        names.append(_sanitize_name(value))
        names.append(value.split("/")[-1])
        names.append(_sanitize_name(value.split("/")[-1]))
    return list(dict.fromkeys(names))


def entry_points():
    return _entry_points()


def available_plugins():
    return list(
        dict.fromkeys([*_REGISTRY.keys(), *[entry_point.name for entry_point in _entry_points()]])
    )


def register_plugin_binding(name, binding):
    _REGISTRY[name] = binding


def unregister_plugin_binding(name):
    _REGISTRY.pop(name, None)


def load_plugin_binding(name):
    for registered_name, binding in _REGISTRY.items():
        if registered_name == name or _sanitize_name(registered_name) == name:
            return binding
    for entry_point in _entry_points():
        if entry_point.name == name or _sanitize_name(entry_point.name) == name:
            return entry_point.load()
    raise ImportError(
        f"No Python binding entry point named '{name}' in group '{_ENTRY_POINT_GROUP}'"
    )


def _apply_binding(binding, plugin):
    if hasattr(binding, "bind"):
        return binding.bind(plugin)
    if hasattr(binding, "from_mujoco_plugin"):
        return binding.from_mujoco_plugin(plugin)
    if callable(binding):
        return binding(plugin)
    return plugin


def bind_plugin(plugin):
    for candidate in _candidate_names(plugin):
        try:
            binding = load_plugin_binding(candidate)
        except ImportError:
            continue
        return _apply_binding(binding, plugin)
    return plugin


_entry_points_list = available_plugins()


class _PluginLoader(importlib.abc.Loader):
    def __init__(self, entry_point):
        self._entry_point = entry_point

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        loaded = self._entry_point.load()
        if isinstance(loaded, types.ModuleType):
            module.__dict__.update(loaded.__dict__)
        else:
            module.__dict__[self._entry_point.name] = loaded


class _PluginFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        package, _, name = fullname.rpartition(".")
        if package != __name__:
            return None

        for registered_name, binding in _REGISTRY.items():
            if registered_name == name or _sanitize_name(registered_name) == name:
                spec = importlib.machinery.ModuleSpec(
                    fullname, _RegisteredPluginLoader(registered_name, binding)
                )
                spec.submodule_search_locations = []
                return spec

        for entry_point in _entry_points():
            if entry_point.name == name or _sanitize_name(entry_point.name) == name:
                spec = importlib.machinery.ModuleSpec(fullname, _PluginLoader(entry_point))
                spec.submodule_search_locations = []
                return spec
        return None


class _RegisteredPluginLoader(importlib.abc.Loader):
    def __init__(self, name, binding):
        self._name = name
        self._binding = binding

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        module.__dict__[self._name] = self._binding


if not any(isinstance(finder, _PluginFinder) for finder in sys.meta_path):
    sys.meta_path.insert(0, _PluginFinder())


__all__ = [
    "_ENTRY_POINT_GROUP",
    "_entry_points_list",
    "available_plugins",
    "bind_plugin",
    "entry_points",
    "load_plugin_binding",
    "register_plugin_binding",
    "unregister_plugin_binding",
]
