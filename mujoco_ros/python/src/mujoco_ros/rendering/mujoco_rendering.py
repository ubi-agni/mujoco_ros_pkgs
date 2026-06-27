from functools import singledispatchmethod

import numpy as np

from pymujoco_ros import _OffscreenCamera
from pymujoco_ros import _OffscreenCameraBuffer


def _reorder_ring_buffer(buffer, index):
    if index == 0:
        return buffer
    return np.concatenate([buffer[index:], buffer[:index]])


def _read_frames(buffer, index, count):
    if count == 0:
        return buffer[:0]
    return _reorder_ring_buffer(buffer, index)[-count:]


class RosCamWrapper:
    def __init__(self, camera: _OffscreenCamera, cam_buff_size: int = 1):
        self.camera = camera
        self.buffer = _OffscreenCameraBuffer(camera, cam_buff_size)
        self._rgb, self._depth, self._segment = self.buffer.getBufferHandles()

    @property
    def width(self):
        return self.camera.width

    @property
    def height(self):
        return self.camera.height

    @property
    def cam_id(self):
        return self.camera.id

    @property
    def cam_name(self):
        return self.camera.name

    @property
    def fps(self):
        return self.camera.pub_freq

    @fps.setter
    def fps(self, fps):
        self.camera.pub_freq = float(fps)

    def set_flag(self, flag_idx: int, enable: bool = True):
        self.buffer._set_flag(flag_idx, enable)

    def toggle_flag(self, flag_idx: int):
        self.buffer._toggle_flag(flag_idx)

    def get_buffered_frames(self):
        rgb, depth, segment = None, None, None
        with self.buffer:
            if self._rgb is not None:
                rgb = _read_frames(
                    self._rgb, self.buffer._rgb_buf_idx, self.buffer._rgb_frame_count
                )
            if self._depth is not None:
                depth = _read_frames(
                    self._depth, self.buffer._depth_buf_idx, self.buffer._depth_frame_count
                )
            if self._segment is not None:
                segment = _read_frames(
                    self._segment, self.buffer._segment_buf_idx, self.buffer._segment_frame_count
                )
            self.buffer.set_buffers_read()
        return rgb, depth, segment


class OffcamManager:
    def __init__(self, offscreen_context, model, cam_buff_size=1):
        self.offscreen_context = offscreen_context
        self._cams_by_id = {}
        self._cams_by_name = {}
        for cam_id in range(model.ncam):
            cam = offscreen_context.camera(cam_id)
            wrapped_cam = RosCamWrapper(cam, cam_buff_size=cam_buff_size)
            self._cams_by_id[cam_id] = wrapped_cam
            self._cams_by_name[cam.name] = wrapped_cam

    @property
    def num_cams(self):
        return len(self._cams_by_id)

    def camera(self, cam_id=0):
        return self.cam(cam_id)

    @singledispatchmethod
    def cam(self, cam_id: int = 0):
        return self._cams_by_id.get(cam_id)

    @cam.register(str)
    def _(self, cam_name: str):
        return self._cams_by_name.get(cam_name)

    def buffer(self, cam_id=0):
        return self.get_buffered_frames(cam_id)

    def get_buffered_frames(self, cam_id=0):
        cam = self.cam(cam_id)
        if cam is None:
            return None, None, None
        return cam.get_buffered_frames()
