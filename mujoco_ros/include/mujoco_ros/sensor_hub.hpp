/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

namespace mujoco_ros {

enum class SensorProviderKind
{
	kNative, ///< MuJoCo mjSENS_* sensors.
	kCustom, ///< Sensors without an MJCF counterpart (e.g. laser).
};

/**
 * Minimal handle the core can store and return without knowing sensor
 * implementations. The sensors package derives its stream interface from it.
 */
class SensorStreamBase
{
public:
	virtual ~SensorStreamBase() = default;

	virtual const std::string &name() const = 0;
	virtual bool is_public() const          = 0;
};

/**
 * Implemented by plugins that own sensor streams. The provider owns the
 * streams; the hub only aggregates lookups over registered providers.
 */
class ISensorStreamProvider
{
public:
	virtual ~ISensorStreamProvider() = default;

	virtual SensorProviderKind kind() const = 0;
	/// Must return nullptr for unknown and for non-public streams.
	virtual SensorStreamBase *Find(const std::string &name) const = 0;
	virtual std::vector<std::string> PublicNames() const          = 0;
};

class SensorHub;

/// Lookup restricted to one provider kind.
class SensorHubView
{
public:
	SensorHubView(const SensorHub &hub, SensorProviderKind kind) : hub_(hub), kind_(kind) {}

	/// nullptr if no public stream of that name exists in this kind.
	SensorStreamBase *Get(const std::string &name) const;
	std::vector<std::string> Names() const;

private:
	const SensorHub &hub_;
	SensorProviderKind kind_;
};

/**
 * Non-owning, env-lifetime aggregate of sensor providers. Never loads MJCF and
 * never creates streams. Registrations are dropped on every env (re)load so no
 * provider pointer outlives its plugin generation.
 */
class SensorHub
{
public:
	/// Throws std::runtime_error if a public stream name already exists in the hub.
	void RegisterProvider(ISensorStreamProvider &provider);
	void UnregisterProvider(ISensorStreamProvider &provider);
	/// Drops all registrations and re-arms the empty-hub warning.
	void Clear();
	bool empty() const;

	/// nullptr if no public stream of that name exists.
	SensorStreamBase *Get(const std::string &name) const;
	std::vector<std::string> Names() const;
	SensorHubView Native() const { return SensorHubView(*this, SensorProviderKind::kNative); }
	SensorHubView Custom() const { return SensorHubView(*this, SensorProviderKind::kCustom); }

	std::size_t EmptyWarningCountForTest() const;

private:
	friend class SensorHubView;
	SensorStreamBase *Find(const std::string &name, const SensorProviderKind *kind) const;
	std::vector<std::string> NamesOf(const SensorProviderKind *kind) const;
	/// Caller holds mutex_.
	void WarnIfEmptyLocked() const;

	mutable std::mutex mutex_;
	std::vector<ISensorStreamProvider *> providers_;
	mutable bool empty_warned_              = false;
	mutable std::size_t empty_warning_count_ = 0;
};

} // namespace mujoco_ros
