/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <gtest/gtest.h>

#include <mujoco_ros/sensor_hub.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using namespace mujoco_ros;

namespace {

class FakeStream : public SensorStreamBase
{
public:
	FakeStream(std::string name, bool is_public) : name_(std::move(name)), public_(is_public) {}
	const std::string &name() const override { return name_; }
	bool is_public() const override { return public_; }

private:
	std::string name_;
	bool public_;
};

class FakeProvider : public ISensorStreamProvider
{
public:
	explicit FakeProvider(SensorProviderKind kind) : kind_(kind) {}

	void Add(const std::string &name, bool is_public = true) { streams_.emplace(name, FakeStream(name, is_public)); }
	SensorProviderKind kind() const override { return kind_; }
	SensorStreamBase *Find(const std::string &name) const override
	{
		auto it = streams_.find(name);
		if (it == streams_.end() || !it->second.is_public()) {
			return nullptr;
		}
		return const_cast<FakeStream *>(&it->second);
	}
	std::vector<std::string> PublicNames() const override
	{
		std::vector<std::string> names;
		for (const auto &[name, stream] : streams_) {
			if (stream.is_public()) {
				names.push_back(name);
			}
		}
		return names;
	}

private:
	SensorProviderKind kind_;
	std::map<std::string, FakeStream> streams_;
};

} // namespace

TEST(SensorHubTest, RegisterDuplicatePublicNameFailsLoud)
{
	SensorHub hub;
	FakeProvider first(SensorProviderKind::kNative);
	FakeProvider second(SensorProviderKind::kCustom);
	first.Add("scan");
	second.Add("scan");

	hub.RegisterProvider(first);
	EXPECT_THROW(hub.RegisterProvider(second), std::runtime_error);
	EXPECT_EQ(hub.Get("scan"), first.Find("scan"));
	EXPECT_EQ(hub.Custom().Get("scan"), nullptr);
}

TEST(SensorHubTest, NativeAndCustomViewsIsolateKinds)
{
	SensorHub hub;
	FakeProvider native(SensorProviderKind::kNative);
	FakeProvider custom(SensorProviderKind::kCustom);
	native.Add("accel");
	custom.Add("front_scan");
	hub.RegisterProvider(native);
	hub.RegisterProvider(custom);

	EXPECT_NE(hub.Native().Get("accel"), nullptr);
	EXPECT_EQ(hub.Native().Get("front_scan"), nullptr);
	EXPECT_NE(hub.Custom().Get("front_scan"), nullptr);
	EXPECT_EQ(hub.Custom().Get("accel"), nullptr);
	EXPECT_NE(hub.Get("accel"), nullptr);
	EXPECT_NE(hub.Get("front_scan"), nullptr);
	EXPECT_EQ(hub.Native().Names(), std::vector<std::string>{ "accel" });
	EXPECT_EQ(hub.Custom().Names(), std::vector<std::string>{ "front_scan" });
}

TEST(SensorHubTest, NonPublicStreamHiddenFromHub)
{
	SensorHub hub;
	FakeProvider provider(SensorProviderKind::kNative);
	provider.Add("visible");
	provider.Add("internal", false);
	hub.RegisterProvider(provider);

	EXPECT_NE(hub.Get("visible"), nullptr);
	EXPECT_EQ(hub.Get("internal"), nullptr);
	EXPECT_EQ(hub.Names(), std::vector<std::string>{ "visible" });
}

TEST(SensorHubTest, NonPublicNameDoesNotCollide)
{
	SensorHub hub;
	FakeProvider first(SensorProviderKind::kNative);
	FakeProvider second(SensorProviderKind::kCustom);
	first.Add("shared", false);
	second.Add("shared");
	hub.RegisterProvider(first);
	EXPECT_NO_THROW(hub.RegisterProvider(second));
}

TEST(SensorHubTest, EmptyHubWarnsOncePerGeneration)
{
	SensorHub hub;
	EXPECT_TRUE(hub.empty());
	EXPECT_EQ(hub.EmptyWarningCountForTest(), 0u);

	EXPECT_EQ(hub.Get("x"), nullptr);
	EXPECT_EQ(hub.EmptyWarningCountForTest(), 1u);
	hub.Names();
	EXPECT_EQ(hub.Native().Get("x"), nullptr);
	EXPECT_EQ(hub.EmptyWarningCountForTest(), 1u);

	hub.Clear();
	EXPECT_EQ(hub.Get("x"), nullptr);
	EXPECT_EQ(hub.EmptyWarningCountForTest(), 2u);
}

TEST(SensorHubTest, NonEmptyHubDoesNotWarn)
{
	SensorHub hub;
	FakeProvider provider(SensorProviderKind::kNative);
	provider.Add("a");
	hub.RegisterProvider(provider);
	EXPECT_EQ(hub.Get("missing"), nullptr);
	EXPECT_EQ(hub.EmptyWarningCountForTest(), 0u);
}

TEST(SensorHubTest, ClearDropsRegistrations)
{
	SensorHub hub;
	FakeProvider provider(SensorProviderKind::kNative);
	provider.Add("a");
	hub.RegisterProvider(provider);
	ASSERT_FALSE(hub.empty());

	hub.Clear();
	EXPECT_TRUE(hub.empty());
	EXPECT_EQ(hub.Get("a"), nullptr);
	// Same provider may re-register after a reload.
	EXPECT_NO_THROW(hub.RegisterProvider(provider));
}

TEST(SensorHubTest, UnregisterDropsOnlyThatProvider)
{
	SensorHub hub;
	FakeProvider a(SensorProviderKind::kNative);
	FakeProvider b(SensorProviderKind::kCustom);
	a.Add("a");
	b.Add("b");
	hub.RegisterProvider(a);
	hub.RegisterProvider(b);
	hub.UnregisterProvider(a);
	EXPECT_EQ(hub.Get("a"), nullptr);
	EXPECT_NE(hub.Get("b"), nullptr);
}
