import os
import signal
import subprocess
import time

try:
    from py_binding_tools import roscpp_init
    from py_binding_tools import roscpp_shutdown
except ImportError:
    roscpp_init = None
    roscpp_shutdown = None

_roscpp_initialized = False


def _is_ros1_available():
    try:
        import rosgraph  # noqa: F401
    except ImportError:
        return False
    return True


def is_ros_master_available():
    if not _is_ros1_available():
        return True

    import rosgraph

    try:
        rosgraph.Master("/mujoco_ros_python").getPid()
        return True
    except Exception:
        return False


def set_use_sim_time(use_sim_time=True):
    if not _is_ros1_available():
        return False

    import rosgraph

    rosgraph.Master("/mujoco_ros_python").setParam("/use_sim_time", bool(use_sim_time))
    return True


def ensure_ros_initialized(node_name="mujoco_server"):
    global _roscpp_initialized
    if not _is_ros1_available() or roscpp_init is None:
        return False

    if _roscpp_initialized:
        return True

    roscpp_init(node_name)
    _roscpp_initialized = True
    return True


def shutdown_ros():
    global _roscpp_initialized
    if _is_ros1_available() and roscpp_shutdown is not None:
        roscpp_shutdown()
        _roscpp_initialized = False


class RosCore:
    def __init__(self, managed=False, timeout=10.0, use_sim_time=True):
        self.managed = managed
        self.timeout = timeout
        self.use_sim_time = use_sim_time
        self._process = None
        self._started = False

    @property
    def owns_core(self):
        return self._started

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.shutdown()

    def start(self):
        if not _is_ros1_available():
            return self

        if is_ros_master_available():
            self._set_default_parameters()
            return self

        if not self.managed:
            raise RuntimeError(
                "No ROS 1 master is reachable. Start roscore, run under rostest/roslaunch, "
                "or construct MujocoEnv with manage_ros_core=True for simple standalone scripts."
            )

        self._process = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self._started = True

        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise RuntimeError("Managed roscore exited before it became reachable")
            if is_ros_master_available():
                self._set_default_parameters()
                return self
            time.sleep(0.1)

        self.shutdown()
        raise RuntimeError(
            f"Timed out after {self.timeout:.1f}s while waiting for managed roscore"
        )

    def _set_default_parameters(self):
        if self.use_sim_time is not None:
            set_use_sim_time(self.use_sim_time)

    def shutdown(self):
        if not self._started or self._process is None:
            return

        if self._process.poll() is None:
            os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            try:
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
                self._process.wait(timeout=5.0)

        self._process = None
        self._started = False
