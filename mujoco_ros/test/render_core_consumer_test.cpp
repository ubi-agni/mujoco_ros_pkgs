#include <gtest/gtest.h>

#include <mujoco_ros/rendering/frame_boundary.hpp>

TEST(RenderCoreConsumer, FrameLeaseCanBeConstructedAndDestroyedThroughMujocoRos)
{
	// This deliberately links through mujoco_ros, rather than mujoco_ros_render_core
	// directly. The out-of-line destructor makes the public RenderCore dependency
	// observable at link time without any manual linker injection.
	mujoco_ros::rendering::FrameLease lease;
	EXPECT_FALSE(lease.valid());
}
