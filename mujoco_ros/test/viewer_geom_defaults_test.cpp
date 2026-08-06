#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <mujoco_ros/viewer.hpp>

TEST(ViewerGeomDefaults, HidesGroup2AfterDefaultOption)
{
	mjvOption opt;
	mjv_defaultOption(&opt);
	ASSERT_NE(opt.geomgroup[2], 0); // MuJoCo default enables 0/1/2
	mujoco_ros::ApplyInteractiveViewerGeomDefaults(&opt);
	EXPECT_EQ(opt.geomgroup[0], 1);
	EXPECT_EQ(opt.geomgroup[1], 1);
	EXPECT_EQ(opt.geomgroup[2], 0);
}
