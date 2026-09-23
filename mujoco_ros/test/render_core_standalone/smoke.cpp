#include <mujoco_ros/rendering/render_backend_interface.hpp>
#include <mujoco_ros/rendering/render_core.hpp>

int main()
{
	using namespace mujoco_ros::rendering;
	auto backend = CreateDisabledRenderBackend();
	RenderCore core(std::move(backend));
	core.StopAcceptingSnapshots();
	core.Shutdown();
	return 0;
}
