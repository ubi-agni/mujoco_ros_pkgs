#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include <mujoco_ros/extended_params.hpp>
#include <mujoco_ros/mesh_uri_prep.hpp>

namespace mujoco_ros {

// Output of ConvertDescription: a standalone robot mjSpec (no scene
// knowledge -- attachment failure) plus the raw, joint-keyed Extended Params
// (extension metadata). Caller owns `spec` and must mj_deleteSpec() it.
//
// `basename_to_dir` mirrors MeshPrepResult::basename_to_dir: the VFS used to
// parse `spec` is deleted before ConvertDescription returns (mj_parseXMLString
// only needs the VFS to *parse*), but mj_compile() also resolves mesh assets
// by basename lookup and needs its own VFS. Callers that intend to
// mj_compile() this spec (or a spec it gets attached into) must rebuild a VFS
// from this map -- see CompileWithMeshVfs() -- rather than passing nullptr,
// or basename-only mesh URIs will fail to resolve on disk.
struct ConversionResult
{
	mjSpec *spec;
	ExtendedParamsByJoint extended_params;
	std::map<std::string, std::string> basename_to_dir;
	std::vector<GlbVisualBinding> glb_visual_bindings;
};

// Ordered list of ros2_control command_interface names ("position"/"velocity"/"effort") requested
// for one joint, keyed by joint name. Order within the vector is document order (matters for
// deterministic actuator-creation order, not for correctness).
using JointCommandInterfaces = std::map<std::string, std::vector<std::string>>;

// Reads every <ros2_control><joint name="..."><command_interface name="..."/></joint> entry from
// raw URDF text (not the compiled mjSpec -- mj_parseXMLString's URDF importer does not preserve
// unknown tags like <ros2_control>). Generic to ros2_control's own vocabulary; agnostic to who
// authored the block. Throws std::runtime_error on a <joint> missing "name" or a
// <command_interface> missing "name" -- never silently skips a malformed entry.
JointCommandInterfaces ParseRos2ControlCommandInterfaces(const std::string &urdf_text);

// Reads every <joint><mimic joint="..."/> tag from raw URDF text, returning
// follower-joint-name -> driver-joint-name. Only used today to reject a mimic
// follower that <ros2_control> also claims (Decision 5); does not itself
// produce any MJCF <equality> -- full mimic porting is out of scope (see
// design.md). Throws std::runtime_error if a <mimic> or its parent <joint>
// has a missing, empty, or whitespace-only required "joint"/"name" attribute,
// a follower has multiple <mimic> children, or URDF repeats a follower name.
std::map<std::string, std::string> ParseMimicFollowers(const std::string &urdf_text);

// Builds a scratch mjVFS from `basename_to_dir`, compiles `spec` against it,
// deletes the VFS, and returns the resulting mjModel* (nullptr on failure,
// mirroring mj_compile's own contract -- use mjs_getError(spec) for details).
// An empty `basename_to_dir` is equivalent to mj_compile(spec, nullptr).
// Caller owns the returned mjModel* and must mj_deleteModel() it.
mjModel *CompileWithMeshVfs(mjSpec *spec, const std::map<std::string, std::string> &basename_to_dir);

// Post-parse pass: visual geoms (contype==0 && conaffinity==0) → group 1;
// all others → group 2. Called from ConvertDescription after parse.
void AssignRobotGeomGroups(mjSpec *robot_spec);

// Post-parse pass: add MuJoCo textures/materials for GLB visual meshes.
void ApplyGlbVisualMaterials(mjSpec *robot_spec, const std::vector<GlbVisualBinding> &bindings);

// Options controlling where and how a converted robot is composed into a
// world. Empty robot_root_body means "infer the robot's root body from the
// spec"; empty world_frame means "attach directly to worldbody". The default
// "spawn_frame" preserves the legacy policy: use it when present, otherwise
// fall back to worldbody.
struct ComposeOptions
{
	std::string prefix      = ""; // Decision 2: callers needing multi-robot namespacing pass one explicitly.
	std::string world_frame = "spawn_frame";
	std::string robot_root_body;
	const mjtNum *pos  = nullptr;
	const mjtNum *quat = nullptr;
};

// Derives a standalone robot mjSpec from URDF (+ optional SRDF
// disable_collisions, + optional Extended Params) in one call. Sets
// spec->strippath = 1 explicitly (binding decision 11 -- do not rely on any
// MuJoCo version's URDF-import default). Never attaches into a world; that
// is the caller's job via ComposeIntoWorld. Throws
// std::runtime_error on any parse or compile failure -- never silently
// drops data.
ConversionResult ConvertDescription(const std::string &urdf_path, const std::string &srdf_path,
                                    const MeshPrepOptions &mesh_options = {}, bool generate_actuators = false,
                                    ConverterExtensionContext *extension_context = nullptr);

// Attaches robot_spec's root body into world_spec at options.world_frame
// (frame first, then body), or directly to worldbody when options.world_frame
// is empty. Empty options.robot_root_body infers the robot's root body from
// the spec. options.prefix must be unique across every
// ComposeIntoWorld call against the same world_spec -- the caller is
// responsible for guaranteeing this (e.g. derive it from a unique
// robot-instance id), never trial-and-error retry a rejected prefix.
//
// Optional world-frame position/orientation to apply to the attached
// robot's root body after a successful attach. When null, the attached
// body keeps whatever local pos/quat it already has (typically the
// identity/origin the URDF parse produced, or wherever spawn_frame placed
// it). `quat` is a wxyz unit quaternion (MuJoCo convention).
//
// On success returns the attached element. On a prefix collision already
// present in world_spec (e.g. options.prefix+root_body exists), throws
// std::runtime_error BEFORE calling mjs_attach -- world_spec is left
// untouched. On a rejected mjs_attach (nullptr return -- attachment failure),
// throws std::runtime_error and world_spec is then in a partially-mutated,
// unrecoverable state: the caller MUST discard it and rebuild from scratch,
// never call ComposeIntoWorld or mj_deleteSpec again on that same pointer
// (even mjs_copy on a poisoned spec has been observed to segfault).
// Note: MuJoCo 3.3.5's mjs_attach does not return nullptr on duplicate
// prefixed names (collision surfaces only at mj_compile); the up-front
// prefix check is what makes that class of failure loud at compose time.
mjsElement *ComposeIntoWorld(mjSpec *world_spec, mjSpec *robot_spec, const ComposeOptions &options);

// Backward-compatible convenience wrapper: uses `prefix`, optional pose, the
// default "spawn_frame" world target, and inferred robot root body.
mjsElement *ComposeIntoWorld(mjSpec *world_spec, mjSpec *robot_spec, const std::string &prefix,
                             const mjtNum pos[3] = nullptr, const mjtNum quat[4] = nullptr);

// Loads mujoco_ros/assets/default_world.xml from the package share directory.
// Caller owns the returned mjSpec* and must mj_deleteSpec() it.
// Throws std::runtime_error if the share path or XML parse fails.
mjSpec *LoadDefaultWorldSpec();

// Implementation-detail helper shared by load_model_from_description and
// MujocoEnv::from_description (mujoco_env.cpp) -- not one of this module's
// two public interfaces. Converts, composes into a copy of world_spec (or
// LoadDefaultWorldSpec() when world_spec is nullptr), compiles, and saves
// the compiled result to a freshly created temp .mjb file. Caller owns the
// returned path and is responsible for removing it once done (e.g. via
// std::remove) -- this function does not clean up after itself. Throws
// std::runtime_error on any conversion, compose, or compile failure.
std::string SaveDescriptionToTempMjb(const std::string &urdf_path, const std::string &srdf_path, mjSpec *world_spec,
                                     const MeshPrepOptions &mesh_options = {});
std::string SaveDescriptionToTempMjb(const std::string &urdf_path, const std::string &srdf_path, mjSpec *world_spec,
                                     const MeshPrepOptions &mesh_options, bool generate_actuators,
                                     const std::string &attach_prefix,
                                     ConverterExtensionContext *extension_context = nullptr);

// Convert -> Compose into a copy of world_spec (or LoadDefaultWorldSpec() when
// world_spec is nullptr, so nullptr means the built-in default world) ->
// mj_compile -> mj_saveModel (temp .mjb) -> mj_loadModel. This produces the
// exact mjModel*/mjData* pair that MujocoEnv::from_description hands to
// LoadModelFromString's existing .mjb-file branch.
// Caller owns the returned mjModel*/mjData* (mj_deleteModel/mj_deleteData).
// Throws std::runtime_error on any conversion, compose, or compile failure.
std::pair<mjModel *, mjData *> load_model_from_description(const std::string &urdf_path, const std::string &srdf_path,
                                                           mjSpec *world_spec                  = nullptr,
                                                           const MeshPrepOptions &mesh_options = {});
std::pair<mjModel *, mjData *> load_model_from_description(const std::string &urdf_path, const std::string &srdf_path,
                                                           mjSpec *world_spec, const MeshPrepOptions &mesh_options,
                                                           bool generate_actuators, const std::string &attach_prefix,
                                                           ConverterExtensionContext *extension_context = nullptr);

} // namespace mujoco_ros
