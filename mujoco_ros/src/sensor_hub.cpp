/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <mujoco_ros/sensor_hub.hpp>

#include <mujoco_ros/logging.hpp>

#include <algorithm>
#include <stdexcept>

namespace mujoco_ros {

SensorStreamBase *SensorHubView::Get(const std::string &name) const
{
	return hub_.Find(name, &kind_);
}

std::vector<std::string> SensorHubView::Names() const
{
	return hub_.NamesOf(&kind_);
}

void SensorHub::RegisterProvider(ISensorStreamProvider &provider)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (std::find(providers_.begin(), providers_.end(), &provider) != providers_.end()) {
		throw std::runtime_error("sensor provider is already registered");
	}
	for (const auto &name : provider.PublicNames()) {
		for (const auto *existing : providers_) {
			if (existing->Find(name) != nullptr) {
				throw std::runtime_error("sensor name '" + name + "' is already registered by another provider");
			}
		}
	}
	providers_.push_back(&provider);
}

void SensorHub::UnregisterProvider(ISensorStreamProvider &provider)
{
	std::lock_guard<std::mutex> lock(mutex_);
	providers_.erase(std::remove(providers_.begin(), providers_.end(), &provider), providers_.end());
}

void SensorHub::Clear()
{
	std::lock_guard<std::mutex> lock(mutex_);
	providers_.clear();
	empty_warned_ = false;
}

bool SensorHub::empty() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return providers_.empty();
}

SensorStreamBase *SensorHub::Get(const std::string &name) const
{
	return Find(name, nullptr);
}

std::vector<std::string> SensorHub::Names() const
{
	return NamesOf(nullptr);
}

std::size_t SensorHub::EmptyWarningCountForTest() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return empty_warning_count_;
}

SensorStreamBase *SensorHub::Find(const std::string &name, const SensorProviderKind *kind) const
{
	std::lock_guard<std::mutex> lock(mutex_);
	WarnIfEmptyLocked();
	for (const auto *provider : providers_) {
		if (kind != nullptr && provider->kind() != *kind) {
			continue;
		}
		if (auto *stream = provider->Find(name)) {
			return stream;
		}
	}
	return nullptr;
}

std::vector<std::string> SensorHub::NamesOf(const SensorProviderKind *kind) const
{
	std::lock_guard<std::mutex> lock(mutex_);
	WarnIfEmptyLocked();
	std::vector<std::string> names;
	for (const auto *provider : providers_) {
		if (kind != nullptr && provider->kind() != *kind) {
			continue;
		}
		auto provided = provider->PublicNames();
		names.insert(names.end(), provided.begin(), provided.end());
	}
	return names;
}

void SensorHub::WarnIfEmptyLocked() const
{
	if (!providers_.empty() || empty_warned_) {
		return;
	}
	empty_warned_ = true;
	++empty_warning_count_;
	MJR_WARN("Sensor hub accessed but no plugin with sensor capability loaded");
}

} // namespace mujoco_ros
