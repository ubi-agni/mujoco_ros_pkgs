# import time
from typing import Dict, Optional, Tuple

# import glfw
# import imageio as iio
import mujoco
import numpy as np

from functools import singledispatchmethod
from pymujoco_ros import _OffscreenCamera, _OffscreenCameraBuffer


# def _import_egl(width, height):
#     from mujoco.egl import GLContext
#     return GLContext(width, height)

# def _import_glfw(width, height):
#     from mujoco.glfw import GLContext

#     return GLContext(width, height)

# def _import_osmesa(width, height):
#     from mujoco.osmesa import GLContext
#     return GLContext(width, height)

# class MujocoOffscreenRenderer:
#     def __init__(
#             self,
#             model,
#             data,
#             max_geom
#     ):
#         self.model = model
#         self.data = data

#         self.viewport = mujoco.MjrRect()


def _reorder_ring_buffer(buf, idx):
    """Reorder a ring buffer given an index"""
    if idx == 1:
        return buf
    return np.concatenate([buf[idx:], buf[:idx]])


class RosCamWrapper:
    def __init__(self, camera: _OffscreenCamera, cam_buff_size: int = 1):
        self.camera = camera
        self.buffer = _OffscreenCameraBuffer(camera, cam_buff_size)
        self._rgb, self._depth, self._seg = self.buffer.getBufferHandles()

    @property
    def width(self):
        return self.camera.width

    @property
    def height(self):
        return self.camera.height

    @property
    def cam_id(self):
        return self.camera.cam_id

    @property
    def cam_name(self):
        return self.camera.cam_name

    @property
    def fps(self):
        return self.camera.pub_freq

    @fps.setter
    def fps(self, fps: float):
        self.camera.pub_freq = fps

    @property
    def pub_frequency(self):
        return 1.0 / self.camera.pub_freq

    def set_flag(self, flag_idx: int, enable: bool = True):
        """Set a visualization flag"""
        self.buffer._set_flag(flag_idx, enable)

    def toggle_flag(self, flag_idx: int):
        """Toggle a visualization flag"""
        self.buffer._toggle_flag(flag_idx)

    def get_buffered_frames(self):
        """Get buffered frames from the camera"""
        rgb, depth, seg = None, None, None
        with self.buffer:
            if self._rgb is not None:
                rgb = _reorder_ring_buffer(self._rgb, self.buffer._rgb_buf_idx)[
                    -self.buffer._rgb_frame_count :
                ]
            if self._depth is not None:
                depth = _reorder_ring_buffer(self._depth, self.buffer._depth_buf_idx)[
                    -self.buffer._depth_frame_count :
                ]
            if self._seg is not None:
                seg = _reorder_ring_buffer(self._seg, self.buffer._segment_buf_idx)[
                    -self.buffer._segment_frame_count :
                ]
            self.buffer.set_buffers_read()
        return rgb, depth, seg


class OffcamManager:
    def __init__(self, offscreen_context, model, cam_buff_size=1):
        self._cams = {}
        self.offscreen_context = offscreen_context
        for cam_id in range(model.ncam):
            cam = offscreen_context.camera(cam_id)
            if cam is None:
                continue
            wrapped_cam = RosCamWrapper(cam, cam_buff_size=cam_buff_size)
            self._cams[cam] = wrapped_cam

    @property
    def num_cams(self):
        return len(self._cams)

    def camera(self, cam_id=0) -> Optional[_OffscreenCamera]:
        return self.cam(cam_id)

    @singledispatchmethod
    def cam(self, cam_id: int = 0) -> Optional[RosCamWrapper]:
        """Get a camera wrapper by its ID"""
        return self._cams.get(self.offscreen_context.camera(cam_id), None)

    @cam.register(str)
    def _(self, cam_name: str) -> Optional[RosCamWrapper]:
        """Get a camera wrapper by its name"""
        return self._cams.get(self.offscreen_context.camera(cam_name), None)

    def buffer(
        self, cam_id: int = 0
    ) -> Tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        return self.get_buffered_frames(cam_id)

    def get_buffered_frames(
        self, cam_id: int = 0
    ) -> Tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        """Get buffered frames from a camera by its ID"""
        cam = self.cam(cam_id)
        if cam is None:
            return None, None, None
        return cam.get_buffered_frames()
