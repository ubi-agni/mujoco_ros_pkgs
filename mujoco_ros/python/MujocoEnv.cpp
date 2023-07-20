#include "mujoco_ros_python.h"

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>

#include <pybind11/stl.h>

#if RENDER_BACKEND == GLFW_BACKEND
static std::string render_backend = "GLFW";
#elif RENDER_BACKEND == OSMESA_BACKEND
static std::string render_backend = "OSMesa";
#elif RENDER_BACKEND == EGL_BACKEND
static std::string render_backend = "EGL";
#else
static std::string render_backend = "NONE. No offscreen rendering available.";
#endif

namespace py = pybind11;
namespace mujoco_ros::python {
using namespace mujoco_ros;

class MujocoEnvWrapper : public MujocoEnv
{
private:
	py::object model_py_                 = py::none();
	py::object data_py_                  = py::none();
	bool viewer_running_                 = false;
	mujoco_ros::Viewer *attached_viewer_ = nullptr;

public:
	~MujocoEnvWrapper()
	{
		ROS_DEBUG("MujocoEnvWrapper destructor called");
		if (viewer_running_) {
			ROS_DEBUG("Closing attached viewer");
			attached_viewer_->exit_request.store(true);
		}
		while (viewer_running_) {
			// sleep until the viewer thread has finished
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		ROS_DEBUG("Attached viewer closed");
	}

	MujocoEnvWrapper(std::string admin_hash) : MujocoEnv(admin_hash, true)
	{
		// TODO(dleins): remove this
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
		ros::console::notifyLoggerLevelsChanged();

		startPhysicsLoop();
		startEventLoop();
	}

	bool Load(py::object m, py::object d)
	{
		// If model and data already exist we need to keep them alive
		// to prevent deleting the data that C++ is storing in the
		// shared_ptr (with no-op deleter) before the C++ reload happened
		py::object keep_alive_model;
		if (!model_py_.is_none()) {
			keep_alive_model = model_py_;
		}
		py::object keep_alive_data;
		if (!data_py_.is_none()) {
			keep_alive_data = data_py_;
		}

		model_py_ = m;
		data_py_  = d;

		std::uintptr_t m_raw = m.attr("_address").cast<std::uintptr_t>();
		std::uintptr_t d_raw = d.attr("_address").cast<std::uintptr_t>();

		mnew = reinterpret_cast<mjModel *>(m_raw);
		dnew = reinterpret_cast<mjData *>(d_raw);

		// Wait until model is loaded in cpp
		settings_.load_request.store(1);

		while (settings_.load_request.load() > 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		return sim_state_.model_valid;
	}

	bool AttachViewer()
	{
		ROS_WARN_STREAM("[attachViewer] BACKEND is: " << render_backend);
#if RENDER_BACKEND == GLFW_BACKEND
		if (viewer_running_) {
			ROS_WARN("Viewer already running. Ignoring request to start new viewer (passive NYI)");
			return false;
		}
		auto viewer_thread = std::thread([&]() {
			// bool is_passive = viewer_running_;
			auto adapter     = std::make_unique<mujoco_ros::GlfwAdapter>();
			auto viewer      = new mujoco_ros::Viewer(std::move(adapter), this, false);
			attached_viewer_ = viewer;
			// if (!is_passive) {
			// 	ROS_WARN("Starting active viewer render loop");
			// 	viewer_running_ = true;
			// }
			viewer_running_ = true;
			viewer->RenderLoop();
			viewer_running_ = false;
			// if (!is_passive) {
			// 	viewer_running_ = false;
			// 	ROS_WARN("Active viewer render loop exited");
			// }
			delete viewer;
		});
		viewer_thread.detach();
		return true;
#else
		ROS_ERROR("GLFW backend not available. Cannot launch viewer!");
		return false;
#endif
	}

	bool is_running()
	{
		return settings_.run.load() || settings_.env_steps_request.load() > 0;
	}

	const std::string getFilename() const
	{
		return std::string(filename_);
	}

	mujoco_ros::OffscreenRenderContext &getOffscreen()
	{
		return offscreen_;
	}

	py::list getPlugins()
	{
		py::list plugins;
		for (const auto &plugin : plugins_) {
			plugins.append(plugin.get());
		}
		return plugins;
	}

	py::object getModelPy()
	{
		return model_py_;
	}
	py::object getDataPy()
	{
		return data_py_;
	}

	// /**
	//  * Pass up to 3 empty numpy arrays to be filled with RGB, depth and segmentation data.
	//  */
	// py::tuple renderOffscreenCam(mujoco_ros::rendering::OffscreenCamera *cam)
	// {
	// 	// bool do_segment = offscreen_.cams[cam_id].stream_type_ & mujoco_ros::rendering::streamType::SEGMENTED;
	// 	// bool do_rgb     = offscreen_.cams[cam_id].stream_type_ & mujoco_ros::rendering::streamType::RGB;
	// 	// bool do_depth   = offscreen_.cams[cam_id].stream_type_ & mujoco_ros::rendering::streamType::DEPTH;
	// 	bool do_segment = cam->stream_type_ & mujoco_ros::rendering::streamType::SEGMENTED;
	// 	bool do_rgb     = cam->stream_type_ & mujoco_ros::rendering::streamType::RGB;
	// 	bool do_depth   = cam->stream_type_ & mujoco_ros::rendering::streamType::DEPTH;
	// 	mjv_updateSceneState(model_.get(), data_.get(), &cam->vopt_, &cam->scn_state_);

	// 	int width  = cam->width_;
	// 	int height = cam->height_;

	// 	std::cout << "[renderOffscreenCam] cam_id: " << cam->cam_id_ << " width: " << width << " height: " << height
	// 	          << " do_rgb: " << do_rgb << " do_depth: " << do_depth << " do_segment: " << do_segment << std::endl;
	// 	// lock render mutex
	// 	std::cout << "[renderOffscreenCam] Locking render mutex" << std::endl;
	// 	offscreen_.render_mutex.lock();
	// 	std::cout << "[renderOffscreenCam] Locked render mutex" << std::endl;

	// 	offscreen_.cam.fixedcamid = cam->cam_id_;
	// 	offscreen_.con.offWidth   = width;
	// 	offscreen_.con.offHeight  = height;
	// 	mjrRect viewport = mjr_maxViewport(&offscreen_.con);

	// 	py::object rgb = py::none();
	// 	py::object depth = py::none();
	// 	py::object seg = py::none();

	// 	if (do_rgb && do_segment) {
	// 		auto rgb_buf = new uint8_t[width * height * 3];
	// 		auto seg_buf = new uint8_t[width * height * 3];

	// 		offscreen_.scn.flags[mjRND_SEGMENT] = 0;
	// 		mjv_updateSceneFromState(&cam->scn_state_, &cam->vopt_, nullptr, &offscreen_.cam, mjCAT_ALL,
	// &offscreen_.scn); 		mjr_render(viewport, &offscreen_.scn, &offscreen_.con);

	// 		if (do_depth) {
	// 			auto depth_buf = new float[width * height];
	// 			mjr_readPixels(rgb_buf, depth_buf, viewport, &offscreen_.con);
	// 			depth = py::array(
	// 				width * height,
	// 				depth_buf,
	// 				py::capsule(depth_buf, [](void *p) {
	// 					delete[] reinterpret_cast<float *>(p);
	// 				})
	// 			);
	// 		} else {
	// 			mjr_readPixels(rgb_buf, nullptr, viewport, &offscreen_.con);
	// 		}

	// 		offscreen_.scn.flags[mjRND_SEGMENT] = 1;
	// 		mjv_updateSceneFromState(&cam->scn_state_, &cam->vopt_, nullptr, &offscreen_.cam, mjCAT_ALL,
	// &offscreen_.scn); 		mjr_render(viewport, &offscreen_.scn, &offscreen_.con); 		mjr_readPixels(seg_buf,
	// nullptr, viewport, &offscreen_.con);

	// 		rgb = py::array(
	// 			width * height * 3,
	// 			rgb_buf,
	// 			py::capsule(rgb_buf, [](void *p) {
	// 				delete[] reinterpret_cast<uint8_t *>(p);
	// 			})
	// 		);
	// 		seg = py::array(
	// 			width * height * 3,
	// 			seg_buf,
	// 			py::capsule(seg_buf, [](void *p) {
	// 				delete[] reinterpret_cast<uint8_t *>(p);
	// 			})
	// 		);

	// 	} else if (do_rgb) {
	// 		std::cout << "[renderOffscreenCam] Creating RGB buffer" << std::endl;
	// 		auto rgb_buf = new uint8_t[width * height * 3];
	// 		offscreen_.scn.flags[mjRND_SEGMENT] = 0;
	// 		std::cout << "[renderOffscreenCam] Rendering RGB" << std::endl;
	// 		mjv_updateSceneFromState(&cam->scn_state_, &cam->vopt_, nullptr, &offscreen_.cam, mjCAT_ALL,
	// &offscreen_.scn); 		mjr_render(viewport, &offscreen_.scn, &offscreen_.con); 		std::cout <<
	// "[renderOffscreenCam] Rendering RGB done" << std::endl;

	// 		if (do_depth) {
	// 			auto depth_buf = new float[width * height];
	// 			mjr_readPixels(rgb_buf, depth_buf, viewport, &offscreen_.con);
	// 			depth = py::array(
	// 				width * height,
	// 				depth_buf,
	// 				py::capsule(depth_buf, [](void *p) {
	// 					delete[] reinterpret_cast<float *>(p);
	// 				})
	// 			);
	// 		} else {
	// 			mjr_readPixels(rgb_buf, nullptr, viewport, &offscreen_.con);
	// 		}

	// 		std::cout << "[renderOffscreenCam] Reading pixels done" << std::endl;
	// 		rgb = py::array(
	// 			width * height * 3,
	// 			rgb_buf,
	// 			py::capsule(rgb_buf, [](void *p) {
	// 				delete[] reinterpret_cast<uint8_t *>(p);
	// 			})
	// 		);
	// 		std::cout << "[renderOffscreenCam] bound RGB buffer" << std::endl;
	// 	} else if (do_segment) {
	// 		auto seg_buf = new uint8_t[width * height * 3];

	// 		offscreen_.scn.flags[mjRND_SEGMENT] = 1;
	// 		mjv_updateSceneFromState(&cam->scn_state_, &cam->vopt_, nullptr, &offscreen_.cam, mjCAT_ALL,
	// &offscreen_.scn); 		mjr_render(viewport, &offscreen_.scn, &offscreen_.con);

	// 		if (do_depth) {
	// 			auto depth_buf = new float[width * height];
	// 			mjr_readPixels(seg_buf, depth_buf, viewport, &offscreen_.con);
	// 			depth = py::array(
	// 				width * height,
	// 				depth_buf,
	// 				py::capsule(depth_buf, [](void *p) {
	// 					delete[] reinterpret_cast<float *>(p);
	// 				})
	// 			);
	// 		} else {
	// 			mjr_readPixels(seg_buf, nullptr, viewport, &offscreen_.con);
	// 		}
	// 		seg = py::array(
	// 			width * height * 3,
	// 			seg_buf,
	// 			py::capsule(seg_buf, [](void *p) {
	// 				delete[] reinterpret_cast<uint8_t *>(p);
	// 			})
	// 		);
	// 	} else if (do_depth) {
	// 		auto depth_buf = new float[width * height];

	// 		mjv_updateSceneFromState(&cam->scn_state_, &cam->vopt_, nullptr, &offscreen_.cam, mjCAT_ALL,
	// &offscreen_.scn); 		mjr_render(viewport, &offscreen_.scn, &offscreen_.con); 		mjr_readPixels(nullptr,
	// depth_buf, viewport, &offscreen_.con);

	// 		depth = py::array(
	// 			width * height,
	// 			depth_buf,
	// 			py::capsule(depth_buf, [](void *p) {
	// 				delete[] reinterpret_cast<float *>(p);
	// 			})
	// 		);
	// 	}

	// 	std::cout << "[renderOffscreenCam] returning rgb, depth, seg" << std::endl;
	// 	return py::make_tuple(rgb, depth, seg);
	// }
};

/************************ Bindings ************************/
void InitMujocoEnvPy(py::module &m)
{
	auto mjenv_wrapper = py::class_<MujocoEnvWrapper, std::shared_ptr<MujocoEnvWrapper>>(m, "_MujocoEnvWrapper");

	// Constructor
	mjenv_wrapper.def(py::init([](std::optional<std::string> admin_hash) {
		                  return std::make_shared<MujocoEnvWrapper>(admin_hash.value_or(""));
	                  }),
	                  py::arg("admin_hash") = py::none());

	// Functions
	mjenv_wrapper.def("_load", &MujocoEnvWrapper::Load, py::arg("model"), py::arg("data"));
	mjenv_wrapper.def("_shutdown", [](MujocoEnvWrapper &self) { self.settings_.exit_request.store(1); });
	mjenv_wrapper.def("_reset", [](MujocoEnvWrapper &self) { self.settings_.reset_request.store(1); });
	mjenv_wrapper.def("_wait_for_events_join", &MujocoEnvWrapper::waitForEventsJoin);
	mjenv_wrapper.def("_wait_for_physics_join", &MujocoEnvWrapper::waitForPhysicsJoin);
	mjenv_wrapper.def("attach_viewer", &MujocoEnvWrapper::AttachViewer);
	mjenv_wrapper.def(
	    "togglePaused",
	    [](MujocoEnvWrapper &self, bool paused, std::optional<std::string> hash) {
		    return self.togglePaused(paused, hash.value_or(""));
	    },
	    py::arg("paused"), py::arg("hash") = py::none());
	mjenv_wrapper.def("step", &MujocoEnvWrapper::step, py::arg("num_steps") = 1, py::arg("blocking") = true);
	mjenv_wrapper.def("get_plugins", &MujocoEnvWrapper::getPlugins, py::return_value_policy::reference_internal);
	mjenv_wrapper.def("__repr__", [](const MujocoEnvWrapper &self) {
		return std::string("<MujocoEnvWrapper filename='") + self.getFilename() + "'>";
	});
	// .def("render", &MujocoEnvWrapper::renderOffscreenCam, py::arg("camera"),
	// py::return_value_policy::reference_internal)

	// Properties
	mjenv_wrapper.def_property_readonly("filename", &MujocoEnvWrapper::getFilename);
	mjenv_wrapper.def_property_readonly("model", &MujocoEnvWrapper::getModelPy,
	                                    py::return_value_policy::reference_internal);
	mjenv_wrapper.def_property_readonly("data", &MujocoEnvWrapper::getDataPy,
	                                    py::return_value_policy::reference_internal);
	mjenv_wrapper.def_readonly("settings", &MujocoEnvWrapper::settings_, py::return_value_policy::reference_internal);
	mjenv_wrapper.def_readonly("sim_state", &MujocoEnvWrapper::sim_state_, py::return_value_policy::reference_internal);
	mjenv_wrapper.def_property_readonly("_offscreen_context", &MujocoEnvWrapper::getOffscreen);

	mjenv_wrapper.attr("kMaxFilenameLength") = MujocoEnvWrapper::kMaxFilenameLength;
	mjenv_wrapper.attr("kErrorLength")       = MujocoEnvWrapper::kErrorLength;
}

} // namespace mujoco_ros::python
