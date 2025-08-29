import gc
import mujoco
import rospy
from pathlib import Path

from mujoco_ros.rendering import OffcamManager

from mujoco_ros_msgs.srv import Reload, ReloadResponse
import subprocess
import sys
import psutil
import signal
import yaml
import time

from typing import List, Dict

import numpy as np

try:
    from pymujoco_ros import _MujocoEnvWrapper, __mujoco_version__
except ImportError:
    print("pymujoco_ros not found")
    pass

import mujoco_ros.plugins

if len(mujoco_ros.plugins._entry_points_list):
    print(f"Available bindings for mujoco_ros plugins:")
    for plugin in mujoco_ros.plugins._entry_points_list:
        print(f"\t{plugin}")
elif len(mujoco_ros.plugins._entry_points_list) == 0:
    rospy.logwarn(
        f"No python bindings for mujoco_ros plugins found. When using a devel space, importing plugins from mujoco_ros.plugins will not work!"
    )

from py_binding_tools import roscpp_init, roscpp_shutdown

if mujoco.__version__ != __mujoco_version__:
    raise Exception(
        f"mujoco_ros ({__mujoco_version__}) and mujoco (python module: {mujoco.__version__}) version mismatch!"
    )


class ROSCoreHandler:
    def __init__(self, port: int = 11311):
        self.port = port

    def run(self):
        try:
            self.roscore_process = subprocess.Popen("roscore")
            self.roscore_pid = self.roscore_process.pid
        except Exception as e:
            sys.stderr.write("roscore could not be started")
            raise e

    def terminate(self):
        try:
            parent = psutil.Process(self.roscore_pid)
            children = parent.children(recursive=True)
            for process in children:
                process.send_signal(signal.SIGTERM)

            self.roscore_process.terminate()
            self.roscore_process.wait()
        except psutil.NoSuchProcess:
            pass

    def __enter__(self):
        self.run()

    def __exit__(self, exc_type, exc_value, traceback):
        self.terminate


class MujocoEnv:
    def __init__(self, xml_path: str = None, start_ros_core: bool = False, **kwargs):
        self.ros_core = None
        if start_ros_core:
            self.ros_core = ROSCoreHandler()
            self.ros_core.run()
        self.setup(xml_path, **kwargs)

    def setup(
        self,
        xml_path: str = None,
        unpause: bool = False,
        headless: bool = None,
        configs_to_load: List[str] = [],
        cam_buff_size: int = 1,
        **kwargs,
    ):
        rospy.init_node("pymujoco_ros")
        roscpp_init("mujoco_server")

        curr_params = rospy.get_param("/", default={})
        for config in configs_to_load:
            if config is None:
                continue
            print("Loading config from: {}".format(config))
            with open(config, "r") as stream:
                try:
                    config_dict = yaml.safe_load(stream)
                    curr_params.update(**config_dict)
                except yaml.YAMLError as exc:
                    print(exc)
                    continue

        rospy.set_param("/", curr_params)

        rospy.set_param("/use_sim_time", True)
        rospy.set_param("/mujoco_server/verbose", True)
        rospy.set_param("/mujoco_server/unpause", unpause)

        if headless is not None:
            rospy.set_param("/mujoco_server/headless", headless)

        if xml_path is None:
            xml_path = rospy.get_param("/mujoco_server/modelfile", None)
        if xml_path is None:
            raise ValueError(
                "No model path provided. Please provide a valid XML or MJB file path."
            )

        self._env = _MujocoEnvWrapper()
        super(MujocoEnv, self).__init__(**kwargs)

        model = self._model_from_string(xml_path)

        data = mujoco.MjData(model)
        self._env._load(model=model, data=data)

        print("\tmujoco version: {}".format(mujoco.__version__))
        print("\tmujoco_ros mujoco version: {}".format(__mujoco_version__))

        self.reload_service = rospy.Service(
            "/mujoco_server/reload", Reload, self._reload_cb
        )

        self.offcam_manager = None
        if self._env.settings.render_offscreen:
            while not self._env.settings.visual_init_request == 0:
                time.sleep(0.05)
            self.offcam_manager = OffcamManager(
                self._env._offscreen_context,
                model,
                cam_buff_size=cam_buff_size,
            )

    def pause(self):
        """Pause simulation"""
        self.settings.run = 0

    def unpause(self):
        """Unpause simulation"""
        self.settings.run = 1

    def step(self, num_steps: int = 1, blocking: bool = True):
        """Step simulation

        Parameters
        ----------
        num_steps : int, optional
            number of steps to forward simulation, by default 1.
        blocking : bool, optional
            whether this call should block, by default True.
        """
        self._env.step(num_steps, blocking)

    def _model_from_string(self, m_string):
        fpath = Path(m_string)
        if fpath.suffix == ".xml":
            if not fpath.is_file():
                raise Exception(f"File at path {fpath.absolute()} does not exist")
            m = mujoco.MjModel.from_xml_path(str(fpath.absolute()))
        elif fpath.suffix == ".mjb":
            if not fpath.is_file():
                raise Exception(f"File at path {fpath.absolute()} does not exist")
            m = mujoco.MjModel.from_binary_path(str(fpath.absolute()))
        else:
            m = mujoco.MjModel.from_xml_string(m_string)
        return m

    def attach_viewer(self, active: bool = False):
        if active:
            print("Attaching viewer")
            self._env.attach_viewer()
        else:
            print("NYI")

    @property
    def plugins(self):
        """Get all loaded plugins"""
        return self._env.get_plugins()

    @property
    def binding(self):
        return self._env

    @property
    def model(self):
        return self._env.model

    @property
    def data(self):
        return self._env.data

    @property
    def settings(self):
        return self._env.settings

    @property
    def plugins(self):
        return self._env.get_plugins()

    def set_enableflag(self, bit: int, enable: bool = True):
        if isinstance(bit, mujoco._enums.mjtEnableBit):
            bit = bit.value
        if enable:
            self._env.model.opt.enableflags |= bit
        else:
            self._env.model.opt.enableflags &= ~bit

    def set_disableflag(self, bit: int, disable: bool = True):
        if isinstance(bit, mujoco._enums.mjtDisableBit):
            bit = bit.value
        if disable:
            self._env.model.opt.disableflags |= bit
        else:
            self._env.model.opt.disableflags &= ~bit

    def toggle_enableflag(self, bit: int):
        self._env.model.opt.enableflags ^= bit

    def toggle_disableflag(self, bit: int):
        self._env.model.opt.disableflags ^= bit

    def _reload_cb(self, req):
        res = ReloadResponse()
        if len(req.model) > self._env.kMaxFilenameLength:
            err = f"Model string too long. Max length: {self._env.kMaxFilenameLength} (got {len(req.model)})"
            rospy.logerr(err)
            res.success = False
            res.status_message = err
            return True

        # If no new model is provided, reuse the current one
        # and create a new data from it
        if len(req.model) > 0:
            m = self._model_from_string(req.model)
        else:
            m = self._env.model

        d = mujoco.MjData(m)

        return self._env._load(m, d)

    def shutdown(self):
        self.reload_service.shutdown()
        self.settings.exit_request = 1
        self._env._wait_for_physics_join()
        self._env._wait_for_events_join()
        self._env = None
        gc.collect()
        if self.ros_core:
            roscpp_shutdown()
            self.ros_core.terminate()
