from functools import singledispatchmethod

import numpy as np

from pymujoco_ros import _OffscreenCamera
from pymujoco_ros import _OffscreenCameraBuffer


class _FrameView(np.ndarray):
    pass


class _BorrowedFrame:
    def __init__(self, native):
        self._native = native

    def __enter__(self):
        view = self._native.__enter__().view(_FrameView)
        view.capture_id = self._native.capture_id
        view.frame_generation = self._native.frame_generation
        return view

    def __exit__(self, exc_type, exc_value, traceback):
        return self._native.__exit__(exc_type, exc_value, traceback)


class _BufferCompatibility:
    """Keep legacy ``cam.buffer`` object access while allowing calls for snapshots."""

    def __init__(self, camera):
        object.__setattr__(self, "_camera", camera)

    def __call__(self, last_n=None):
        return self._camera._buffer_snapshot(last_n=last_n)

    def __enter__(self):
        return self._camera._buffer.__enter__()

    def __exit__(self, exc_type, exc_value, traceback):
        return self._camera._buffer.__exit__(exc_type, exc_value, traceback)

    def __getattr__(self, name):
        return getattr(self._camera._buffer, name)

    def __setattr__(self, name, value):
        if name == "_camera":
            object.__setattr__(self, name, value)
            return
        setattr(self._camera._buffer, name, value)


class RosCamWrapper:
    def __init__(self, context, camera: _OffscreenCamera, cam_buff_size: int = 1):
        self._context = context
        self._camera_id = camera.id
        self._buffer = _OffscreenCameraBuffer(context, self._camera_id, cam_buff_size)
        self.buffer = _BufferCompatibility(self)

    @property
    def camera(self):
        return self._context.camera(self._camera_id)

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
        self._buffer._set_flag(flag_idx, enable)

    def toggle_flag(self, flag_idx: int):
        self._buffer._toggle_flag(flag_idx)

    def _has_plane(self, bit):
        return bool(int(self.camera.stream_type) & bit)

    def borrow_latest_rgb(self):
        return _BorrowedFrame(self._buffer.borrow_latest_rgb())

    def borrow_latest_depth(self):
        return _BorrowedFrame(self._buffer.borrow_latest_depth())

    def borrow_latest_segment(self):
        return _BorrowedFrame(self._buffer.borrow_latest_segment())

    def copy_rgb(self, count=1):
        return self._buffer.copy_rgb(int(count))

    def copy_depth(self, count=1):
        return self._buffer.copy_depth(int(count))

    def copy_segment(self, count=1):
        return self._buffer.copy_segment(int(count))

    def _buffer_snapshot(self, last_n=None):
        count = 1 if last_n is None else int(last_n)
        if count <= 0:
            raise ValueError("last_n must be positive")
        rgb = self._buffer.copy_rgb(count) if self._has_plane(1) else None
        depth = self._buffer.copy_depth(count) if self._has_plane(2) else None
        segment = self._buffer.copy_segment(count) if self._has_plane(4) else None
        return rgb, depth, segment

    def get_buffered_frames(self, last_n=None):
        return self._buffer_snapshot(last_n=last_n)

    def close(self):
        self._buffer.close()


class OffcamManager:
    def __init__(self, camera_publication_transport, model, cam_buff_size=1):
        del model
        self.camera_publication_transport = camera_publication_transport
        self._cam_buff_size = cam_buff_size
        self._cams_by_id = {}
        self._cams_by_name = {}
        self._closed = False
        self._refresh_cameras()

    def _refresh_cameras(self):
        if self._closed:
            return

        active_ids = set()
        cameras_by_name = {}
        for cam_id in range(self.camera_publication_transport.num_cams):
            cam = self.camera_publication_transport.camera(cam_id)
            active_ids.add(cam_id)
            wrapped_cam = self._cams_by_id.get(cam_id)
            if wrapped_cam is None:
                wrapped_cam = RosCamWrapper(
                    self.camera_publication_transport, cam, cam_buff_size=self._cam_buff_size
                )
                self._cams_by_id[cam_id] = wrapped_cam
            cameras_by_name[cam.name] = wrapped_cam

        for cam_id in set(self._cams_by_id) - active_ids:
            self._cams_by_id.pop(cam_id).close()
        self._cams_by_name = cameras_by_name

    @property
    def num_cams(self):
        self._refresh_cameras()
        return len(self._cams_by_id)

    def camera(self, cam_id=0):
        return self.cam(cam_id)

    @singledispatchmethod
    def cam(self, cam_id: int = 0):
        self._refresh_cameras()
        return self._cams_by_id.get(cam_id)

    @cam.register(str)
    def _(self, cam_name: str):
        self._refresh_cameras()
        return self._cams_by_name.get(cam_name)

    def buffer(self, cam_id=0, last_n=None):
        return self.get_buffered_frames(cam_id, last_n=last_n)

    def get_buffered_frames(self, cam_id=0, last_n=None):
        cam = self.cam(cam_id)
        if cam is None:
            return None, None, None
        return cam.get_buffered_frames(last_n=last_n)

    def close(self):
        if self._closed:
            return
        self._closed = True
        for cam in self._cams_by_id.values():
            cam.close()
        self._cams_by_id.clear()
        self._cams_by_name.clear()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
