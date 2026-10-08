from pathlib import Path
import copy
import os
import sys
import tempfile
import threading
import time
import warnings

from pymujoco_ros import _MujocoEnvWrapper
from pymujoco_ros import __mujoco_version__
from pymujoco_ros import __ros_version__

from .ros_context import RosCore
from .ros_context import ensure_ros_initialized
from .plugins import bind_plugin

import mujoco

if mujoco.__version__ != __mujoco_version__:
    raise Exception(
        f"mujoco_ros ({__mujoco_version__}) and mujoco (python module: {mujoco.__version__}) version mismatch!"
    )


def _is_ros1():
    # Compile-time ROS version of the pymujoco_ros binding is the source of truth.
    return __ros_version__ == 1


# Loggers touched by verbose=True. Core entries mirror launch_server.launch.py/.xml;
# the plugin-package entries target the plugins directly, at package granularity
# (ROS 1 rosconsole loggers are log4cxx-hierarchical, so "ros.<pkg>" also covers any
# NAMED sub-loggers a plugin uses internally, e.g. mujoco_ros_sensors' "sensors").
# Deliberately excludes generic pluginlib/class_loader loggers -- they only log
# load/unload noise, not anything plugin-specific.
_DEFAULT_VERBOSE_LOGGERS_ROS2 = {
    "mujoco_server": "debug",
    "mujoco_ros_plugin_loader": "debug",
    "Viewer": "debug",
    "mujoco_ros_control": "debug",
    "sensors": "debug",
    "lasers": "debug",
}
_DEFAULT_VERBOSE_LOGGERS_ROS1 = {
    "ros.mujoco_ros": "debug",
    "ros.mujoco_ros_control": "debug",
    "ros.mujoco_ros_sensors": "debug",
    "ros.mujoco_ros_laser": "debug",
    "ros.mujoco_ros_mocap": "debug",
}


def _resolve_log_levels(verbose, log_levels):
    defaults = _DEFAULT_VERBOSE_LOGGERS_ROS1 if _is_ros1() else _DEFAULT_VERBOSE_LOGGERS_ROS2
    resolved = dict(defaults) if verbose else {}
    resolved.update(log_levels or {})
    return resolved


def _apply_ros2_log_levels(log_levels):
    # rclcpp has no post-init logger-level API exposed here, so this must run
    # before EnsureRosInitialized() (mujoco_env.cpp) consumes sys.argv -- i.e.
    # before _MujocoEnvWrapper is constructed. Mirrors _prepare_parameters's
    # --params-file injection below.
    if not log_levels:
        return
    if "--ros-args" not in sys.argv:
        sys.argv.append("--ros-args")
    for name, level in log_levels.items():
        sys.argv.extend(["--log-level", f"{name}:={level}"])


def _apply_ros1_log_levels(log_levels):
    # roscpp has no --log-level CLI equivalent, so this runs the other way
    # around: after roscpp is initialized, via the ros::console::set_logger_level
    # wrapper exposed as pymujoco_ros.set_ros1_logger_level (mujoco_env.cpp).
    if not log_levels:
        return
    from pymujoco_ros import set_ros1_logger_level

    for name, level in log_levels.items():
        set_ros1_logger_level(name, level)


def _deep_merge(base, update):
    result = copy.deepcopy(base)
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _load_yaml_file(path):
    import yaml

    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _ros2_plugin_config(plugin_config):
    if plugin_config is None:
        return {}
    if isinstance(plugin_config, dict):
        return plugin_config

    ros_parameters = {}
    plugin_nodes = {}
    plugin_names = []
    for index, plugin in enumerate(plugin_config):
        plugin_name = plugin.get("name", f"python_plugin_{index}")
        plugin_names.append(plugin_name)
        ros_parameters[f"MujocoPlugins.{plugin_name}.type"] = plugin["type"]
        plugin_params = {
            key: value for key, value in plugin.items() if key not in ("name", "type")
        }
        if plugin_params:
            plugin_nodes[plugin_name] = {"ros__parameters": plugin_params}
    ros_parameters["MujocoPlugins.names"] = plugin_names
    return {"/mujoco_server": {"ros__parameters": ros_parameters, **plugin_nodes}}


def _ros1_plugin_config(plugin_config):
    if plugin_config is None:
        return {}
    if isinstance(plugin_config, dict):
        return plugin_config
    return {"MujocoPlugins": plugin_config}


def _is_existing_file(value):
    try:
        return Path(os.fspath(value)).is_file()
    except OSError:
        return False


def _fetch_topic_content(topic, timeout):
    """Read one std_msgs/String off `topic`, mirroring the C++ ResolveTopicSource
    contract (parameter_interface.cpp): the publisher must be latched
    (transient_local + reliable QoS on ROS 2, latch=True on ROS 1)."""
    if _is_ros1():
        import rospy
        from std_msgs.msg import String

        if not rospy.core.is_initialized():
            rospy.init_node(
                "mujoco_ros_python_description_reader", anonymous=True, disable_signals=True
            )
        try:
            return rospy.wait_for_message(topic, String, timeout=timeout).data
        except rospy.ROSException as exc:
            raise RuntimeError(
                f"Timed out after {timeout:.1f}s waiting for a message on topic "
                f"'{topic}'. The publisher must publish std_msgs/String with "
                "latch=True (a latched publisher)."
            ) from exc

    import rclpy
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import HistoryPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from std_msgs.msg import String

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("mujoco_ros_python_description_reader")
    result = {}
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    sub = node.create_subscription(
        String, topic, lambda msg: result.setdefault("data", msg.data), qos
    )
    try:
        deadline = time.monotonic() + timeout
        while "data" not in result:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"Timed out after {timeout:.1f}s waiting for a message on topic "
                    f"'{topic}'. The publisher must publish std_msgs/String with "
                    "transient_local + reliable QoS (a latched publisher)."
                )
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
    return result["data"]


def _write_temp_description_file(content, suffix):
    fd, path = tempfile.mkstemp(prefix="mujoco_ros_python_description_", suffix=suffix)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(content)
    return path


class RuntimeSettings:
    def __init__(self, env):
        object.__setattr__(self, "_env", env)

    def _snapshot(self):
        return self._env.binding.settings

    def snapshot(self):
        return self._snapshot()

    def __getattr__(self, name):
        return getattr(self._snapshot(), name)

    def __setattr__(self, name, value):
        writable = {"running", "rt_factor", "busywait", "gravity", "render_backpressure_policy"}
        if name == "_env":
            object.__setattr__(self, name, value)
            return
        if name in writable:
            type(self).__dict__[name].fset(self, value)
            return
        raise AttributeError(f"'{name}' is a read-only runtime setting snapshot field")

    @property
    def running(self):
        return self._env.is_running

    @running.setter
    def running(self, value):
        self._env.toggle_paused(not bool(value))

    @property
    def rt_factor(self):
        return self._env.sim_info.rt_setting

    @rt_factor.setter
    def rt_factor(self, value):
        self._env.set_rt_factor(float(value))

    @property
    def busywait(self):
        return self._snapshot().busywait

    @busywait.setter
    def busywait(self, value):
        self._env.binding.set_busywait(int(value))

    @property
    def gravity(self):
        return self._env.get_gravity()

    @gravity.setter
    def gravity(self, value):
        self._env.set_gravity(value)

    @property
    def render_backpressure_policy(self):
        return self._env.render_backpressure_policy

    @render_backpressure_policy.setter
    def render_backpressure_policy(self, value):
        self._env.render_backpressure_policy = str(value)


_ENABLE_FLAG_FIELDS = {
    int(mujoco.mjtEnableBit.mjENBL_OVERRIDE): 'override_contacts',
    int(mujoco.mjtEnableBit.mjENBL_ENERGY): 'energy',
    int(mujoco.mjtEnableBit.mjENBL_FWDINV): 'fwd_inv',
    int(mujoco.mjtEnableBit.mjENBL_INVDISCRETE): 'inv_discrete',
    int(mujoco.mjtEnableBit.mjENBL_MULTICCD): 'multiccd',
    int(mujoco.mjtEnableBit.mjENBL_ISLAND): 'island',
}

_DISABLE_FLAG_FIELDS = {
    int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT): 'constraint_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_EQUALITY): 'equality_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_FRICTIONLOSS): 'frictionloss_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_LIMIT): 'limit_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_CONTACT): 'contact_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_PASSIVE): 'passive_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_GRAVITY): 'gravity_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL): 'clampctrl_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_WARMSTART): 'warmstart_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT): 'filterparent_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_ACTUATION): 'actuation_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_REFSAFE): 'refsafe_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_SENSOR): 'sensor_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_MIDPHASE): 'midphase_disabled',
    int(mujoco.mjtDisableBit.mjDSBL_EULERDAMP): 'eulerdamp_disabled',
}


class MujocoEnv:
    def __init__(
        self,
        admin_hash=None,
        initialize_ros=True,
        manage_ros_core=False,
        ros_core_timeout=10.0,
        use_sim_time=True,
        config_files=None,
        parameters=None,
        plugin_config=None,
        model_path=None,
        python_reload_service=False,
        verbose=False,
        log_levels=None,
        runtime_options=None,
    ):
        self._ros_initialized = False
        self._ros_core = RosCore(
            managed=manage_ros_core, timeout=ros_core_timeout, use_sim_time=use_sim_time
        )
        self._ros_core.start()

        self._temp_param_file = None
        self._reload_service = None
        self._reload_node = None
        self._reload_executor = None
        self._reload_thread = None
        self._current_model_source = model_path

        self._prepare_parameters(config_files or [], parameters or {}, plugin_config)

        resolved_log_levels = _resolve_log_levels(verbose, log_levels)
        if not _is_ros1():
            _apply_ros2_log_levels(resolved_log_levels)

        if initialize_ros:
            self._ros_initialized = ensure_ros_initialized("mujoco_server")

        if _is_ros1():
            _apply_ros1_log_levels(resolved_log_levels)

        self._env = _MujocoEnvWrapper(
            admin_hash=admin_hash,
            python_reload_service=python_reload_service,
            runtime_options=runtime_options,
        )
        self._settings = RuntimeSettings(self)

        if python_reload_service:
            self._start_reload_service()

        if model_path is not None:
            self.load_from_path(model_path)

    @classmethod
    def from_description(
        cls,
        urdf_path,
        srdf_path="",
        generate_actuators=False,
        attach_prefix="",
        ros_params=None,
        **launch_kw_args,
    ):
        from pymujoco_ros import load_model_from_description

        model, data = load_model_from_description(
            urdf_path,
            srdf_path,
            generate_actuators=generate_actuators,
            attach_prefix=attach_prefix,
        )
        env = cls(parameters=ros_params, **launch_kw_args)
        env._load_python_model(model, data, filename=urdf_path)
        return env

    @classmethod
    def from_description_topic(
        cls,
        urdf_topic='/robot_description',
        srdf_topic="",
        srdf_path="",
        timeout=5.0,
        generate_actuators=False,
        attach_prefix="",
        ros_params=None,
        **launch_kw_args,
    ):
        """Same as from_description, but reads URDF content off a ROS topic
        instead of a file path. srdf_path (a plain filesystem path, e.g. for
        deployments where SRDF stays local-only) takes precedence over
        srdf_topic when both are given. The publisher(s) must be latched
        (see _fetch_topic_content); this mirrors the server-side kTopic
        DescriptionSource convention (description_bundle.hpp). ros_params is
        forwarded to MujocoEnv's own `parameters` kwarg, becoming top-level
        mujoco_server ROS params -- e.g. domain_id, or any other param a
        plugin reads off env_ptr_->get_parameter(...)."""
        from pymujoco_ros import load_model_from_description

        urdf_content = _fetch_topic_content(urdf_topic, timeout)

        urdf_path = _write_temp_description_file(urdf_content, ".urdf")
        resolved_srdf_path = srdf_path
        temp_srdf_path = ""
        if not resolved_srdf_path and srdf_topic:
            srdf_content = _fetch_topic_content(srdf_topic, timeout)
            temp_srdf_path = _write_temp_description_file(srdf_content, ".srdf")
            resolved_srdf_path = temp_srdf_path
        try:
            model, data = load_model_from_description(
                urdf_path,
                resolved_srdf_path,
                generate_actuators=generate_actuators,
                attach_prefix=attach_prefix,
            )
        finally:
            os.unlink(urdf_path)
            if temp_srdf_path:
                os.unlink(temp_srdf_path)

        env = cls(parameters=ros_params, **launch_kw_args)
        env._load_python_model(model, data, filename=urdf_topic)
        return env

    def _prepare_parameters(self, config_files, parameters, plugin_config):
        if _is_ros1():
            config = {}
            for config_file in config_files:
                config = _deep_merge(config, _load_yaml_file(os.fspath(config_file)))
            config = _deep_merge(config, parameters)
            config = _deep_merge(config, _ros1_plugin_config(plugin_config))
            if config:
                import rosgraph

                master = rosgraph.Master("/mujoco_ros_python")
                for key, value in config.items():
                    normalized_key = key.strip("/")
                    master.setParam("/" + normalized_key, value)
                    master.setParam("/mujoco_server/" + normalized_key, value)
            return

        config = {}
        for config_file in config_files:
            config = _deep_merge(config, _load_yaml_file(os.fspath(config_file)))
        if parameters:
            config = _deep_merge(config, {"/mujoco_server": {"ros__parameters": parameters}})
        config = _deep_merge(config, _ros2_plugin_config(plugin_config))
        if not config:
            return

        import yaml

        fd, path = tempfile.mkstemp(prefix="mujoco_ros_python_", suffix=".yaml")
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream)
        self._temp_param_file = path
        if "--ros-args" not in sys.argv:
            sys.argv.extend(["--ros-args"])
        sys.argv.extend(["--params-file", path])

    def _model_from_string(self, model_or_path):
        import mujoco

        model_or_path = (
            os.fspath(model_or_path) if isinstance(model_or_path, os.PathLike) else model_or_path
        )
        model_path = Path(str(model_or_path))
        if model_path.suffix == ".xml" and _is_existing_file(model_path):
            return mujoco.MjModel.from_xml_path(str(model_path))
        if model_path.suffix == ".mjb" and _is_existing_file(model_path):
            return mujoco.MjModel.from_binary_path(str(model_path))
        return mujoco.MjModel.from_xml_string(str(model_or_path))

    def _load_python_model(self, model, data, filename=""):
        filename = os.fspath(filename) if isinstance(filename, os.PathLike) else filename
        self._current_model_source = filename
        return self._env._load(model, data, filename)

    def load_from_path(self, path):
        import mujoco

        path = os.fspath(path)
        model = self._model_from_string(path)
        data = mujoco.MjData(model)
        return self._load_python_model(model, data, str(path))

    def load_from_string(self, model_xml, filename=''):
        import mujoco

        filename = os.fspath(filename) if isinstance(filename, os.PathLike) else filename
        model = self._model_from_string(model_xml)
        data = mujoco.MjData(model)
        return self._load_python_model(model, data, filename)

    def start_physics_loop(self):
        self._env.start_physics_loop()

    def start_event_loop(self):
        self._env.start_event_loop()

    def load_model_from_string(self, model_or_path):
        model_or_path = (
            os.fspath(model_or_path) if isinstance(model_or_path, os.PathLike) else model_or_path
        )
        return self._env.load_model_from_string(model_or_path)

    def attach_viewer(self, active=True):
        warnings.warn(
            'MujocoEnv.attach_viewer() is deprecated; use '
            'mujoco_ros.viewer.launch() or launch_passive()',
            DeprecationWarning,
            stacklevel=2,
        )
        from . import viewer

        if active:
            return viewer.launch(self)
        return viewer.launch_passive(self, auto_sync=True)

    def step(self, num_steps=1, blocking=True):
        return self._env.step(num_steps, blocking)

    def reset(self):
        self._env.reset()

    def pause(self, admin_hash=""):
        return self.toggle_paused(True, admin_hash)

    def unpause(self, admin_hash=""):
        return self.toggle_paused(False, admin_hash)

    def toggle_paused(self, paused, admin_hash=""):
        return self._env.toggle_paused(paused, admin_hash)

    def set_rt_factor(self, rt_factor, admin_hash=""):
        return self._env.set_rt_factor(rt_factor, admin_hash)

    @property
    def render_backpressure_policy(self):
        return self._env.render_backpressure_policy

    @render_backpressure_policy.setter
    def render_backpressure_policy(self, value):
        self._env.render_backpressure_policy = str(value)

    def get_gravity(self):
        return self._env.get_gravity()

    def set_gravity(self, gravity, admin_hash=""):
        return self._env.set_gravity(gravity, admin_hash)

    @property
    def runtime_options(self):
        return self._env.runtime_options

    def apply_runtime_options(self, values):
        return self._env.apply_runtime_options(values)

    def _start_reload_service(self):
        if _is_ros1():
            import rospy
            from mujoco_ros_msgs.srv import Reload

            if not rospy.core.is_initialized():
                rospy.init_node("mujoco_ros_python_reload", anonymous=True, disable_signals=True)
            self._reload_service = rospy.Service(
                f"{self.handle_namespace}/reload", Reload, self._reload_cb_ros1
            )
            return

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from mujoco_ros_msgs.srv import Reload

        if not rclpy.ok():
            rclpy.init()
        self._reload_node = rclpy.create_node("mujoco_ros_python_reload")
        self._reload_service = self._reload_node.create_service(
            Reload, f"{self.handle_namespace}/reload", self._reload_cb_ros2
        )
        self._reload_executor = SingleThreadedExecutor()
        self._reload_executor.add_node(self._reload_node)
        self._reload_thread = threading.Thread(target=self._reload_executor.spin, daemon=True)
        self._reload_thread.start()

    def _reload_impl(self, model):
        try:
            source = model or self._current_model_source
            if source is None:
                return False, "No Python-owned model source is available for reload"
            source = os.fspath(source) if isinstance(source, os.PathLike) else source
            success = (
                self.load_from_path(source)
                if _is_existing_file(source)
                else self.load_from_string(source)
            )
            return bool(success), ""
        except Exception as exc:
            return False, str(exc)

    def _reload_cb_ros1(self, req):
        from mujoco_ros_msgs.srv import ReloadResponse

        success, status = self._reload_impl(req.model)
        return ReloadResponse(success=success, status_message=status)

    def _reload_cb_ros2(self, req, res):
        success, status = self._reload_impl(req.model)
        res.success = success
        res.status_message = status
        return res

    def shutdown(self):
        native_error = None
        if getattr(self, "_reload_executor", None) is not None:
            self._reload_executor.shutdown()
            self._reload_executor = None
        if getattr(self, "_reload_thread", None) is not None:
            self._reload_thread.join(timeout=1.0)
            self._reload_thread = None
        if getattr(self, '_reload_node', None) is not None:
            self._reload_node.destroy_node()
            self._reload_node = None
        if getattr(self, "_reload_service", None) is not None:
            if hasattr(self._reload_service, "shutdown"):
                self._reload_service.shutdown()
            self._reload_service = None

        if getattr(self, "_env", None) is not None:
            try:
                self._env.shutdown()
            except BaseException as exc:
                native_error = exc
            finally:
                self._env = None

        if getattr(self, "_ros_core", None) is not None:
            self._ros_core.shutdown()
            self._ros_core = None

        if getattr(self, "_temp_param_file", None) is not None:
            # sys.argv is process-global and rclpy.init() re-parses it on every
            # call; leaving our "--params-file <path>" pair behind breaks the
            # *next* MujocoEnv/rclpy.init() in this process once the file below
            # is deleted (RCLError: "Couldn't parse params file").
            pair = ["--params-file", self._temp_param_file]
            for i in range(len(sys.argv) - 1):
                if sys.argv[i] == pair[0] and sys.argv[i + 1] == pair[1]:
                    sys.argv.pop(i)
                    sys.argv.pop(i)
                    break
            try:
                os.unlink(self._temp_param_file)
            except OSError:
                pass
            self._temp_param_file = None

        if native_error is not None:
            raise native_error

    def wait_for_physics_join(self):
        self._env.wait_for_physics_join()

    def wait_for_events_join(self):
        self._env.wait_for_events_join()

    @property
    def model_valid(self):
        return self._env.model_valid

    @property
    def load_count(self):
        return self._env.load_count

    @property
    def operational_status(self):
        return self._env.operational_status

    def wait_for_operational_status_idle(self, timeout=5.0):
        if not self._env.wait_for_operational_status_idle(int(timeout * 1000)):
            raise RuntimeError("environment did not become idle before timeout")

    @property
    def settings(self):
        return self._settings

    @property
    def sim_state(self):
        return self._env.sim_state

    @property
    def sim_info(self):
        return self._env.sim_info

    @property
    def plugin_stats(self):
        return self._env.plugin_stats

    @property
    def plugins(self):
        return [bind_plugin(plugin) for plugin in self._env.plugins]

    @property
    def plugin_names(self):
        return self._env.plugin_names

    @property
    def model(self):
        return self._env.model

    @property
    def data(self):
        return self._env.data

    @property
    def filename(self):
        return self._env.filename

    @property
    def handle_namespace(self):
        return self._env.handle_namespace

    @property
    def is_running(self):
        return self._env.is_running

    def set_enableflag(self, bit: int, enable: bool):
        if isinstance(bit, mujoco._enums.mjtEnableBit):
            bit = bit.value
        field = _ENABLE_FLAG_FIELDS.get(int(bit))
        if field is None:
            raise ValueError(f'unsupported enable flag bit: {bit}')
        current = getattr(self.runtime_options, field)
        if current == enable:
            return
        self.apply_runtime_options({field: enable})

    def set_disableflag(self, bit: int, disable: bool):
        if isinstance(bit, mujoco._enums.mjtDisableBit):
            bit = bit.value
        field = _DISABLE_FLAG_FIELDS.get(int(bit))
        if field is None:
            raise ValueError(f'unsupported disable flag bit: {bit}')
        current = getattr(self.runtime_options, field)
        if current == disable:
            return
        self.apply_runtime_options({field: disable})

    def toggle_enableflag(self, bit: int):
        if isinstance(bit, mujoco._enums.mjtEnableBit):
            bit = bit.value
        field = _ENABLE_FLAG_FIELDS.get(int(bit))
        if field is None:
            raise ValueError(f'unsupported enable flag bit: {bit}')
        current = getattr(self.runtime_options, field)
        self.apply_runtime_options({field: not current})

    def toggle_disableflag(self, bit: int):
        if isinstance(bit, mujoco._enums.mjtDisableBit):
            bit = bit.value
        field = _DISABLE_FLAG_FIELDS.get(int(bit))
        if field is None:
            raise ValueError(f'unsupported disable flag bit: {bit}')
        current = getattr(self.runtime_options, field)
        self.apply_runtime_options({field: not current})

    @property
    def binding(self):
        return self._env

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.shutdown()

    def __del__(self):
        self.shutdown()
