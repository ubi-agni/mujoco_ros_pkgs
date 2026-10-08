#include <mujoco/mujoco.h>
#include <mujoco_ros/rendering/render_backend_interface.hpp>
#include <mujoco_ros/rendering/render_core.hpp>
#include <mujoco_ros/rendering/frame_boundary.hpp>

int main()
{
	using namespace mujoco_ros::rendering;
	if (mj_version() < 330) {
		return 1;
	}
	FrameLease lease;
	if (lease.valid()) {
		return 1;
	}
	auto backend = CreateDisabledRenderBackend();
	RenderCore core(std::move(backend));
	core.StopAcceptingSnapshots();
	core.Shutdown();
	return 0;
}
