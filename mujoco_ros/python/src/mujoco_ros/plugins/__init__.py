import pkg_resources

## Opt 3
import sys
import importlib
import importlib.machinery
import importlib.metadata

_ENTRY_POINT_GROUP = "mujoco_ros.plugins"

_entry_points_list = [
    ep.name for ep in pkg_resources.iter_entry_points(group=_ENTRY_POINT_GROUP)
]


class _PluginLoader(importlib.abc.Loader):
    def __init__(self, entry_point):
        self._entry_point = entry_point

    def create_module(self, spec):
        # Use default module creation
        return None

    def exec_module(self, module):
        # Load the module from the entry point
        loaded = self._entry_point.load()
        if hasattr(loaded, "__dict__"):
            module.__dict__.update(loaded.__dict__)
        else:
            module.__dict__[self._entry_point.name] = loaded


class _PluginFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        pkg, _, name = fullname.rpartition(".")
        if pkg != __name__:
            return None

        eps = importlib.metadata.entry_points().get(_ENTRY_POINT_GROUP, [])
        for ep in eps:
            if ep.name == name:
                spec = importlib.machinery.ModuleSpec(fullname, _PluginLoader(ep))
                spec.submodule_search_locations = []
                return spec
        return None


sys.meta_path.insert(0, _PluginFinder())
