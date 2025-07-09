import mujoco
import rospy
import numpy as np

from scipy.spatial.transform import Rotation

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle  # , Plane
from geometry_msgs.msg import Pose, Point


def _as_point(point: np.ndarray):
    """Convert a numpy array to a Point message."""
    return Point(x=point[0], y=point[1], z=point[2])


def _as_orientation(quat: np.ndarray):
    """Convert a numpy array to a Quaternion message."""
    return Pose().orientation.__class__(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


def _body_to_psi(
    body_name: str,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    use_convex_hull: bool = False,
) -> CollisionObject:
    """Get the position, size and orientation of a body in the model/data and convert it to a CollisionObject.

    Parameters
    ----------
    body_name : str
        Name of the body to get the position, size and orientation from.
    model : mujoco.MjModel
        MuJoCo model to get the body from.
    data : mujoco.MjData
        MuJoCo data to get the body from.

    Returns
    -------
    moveit_msgs.msg.CollisionObject
        PlanningScene CollisionObject with the position, size and orientation of the body.
    """

    try:
        body_model = model.body(body_name)
    except KeyError:
        rospy.logerr(f"Body '{body_name}' not found in model!")
        return None

    collision_obj = CollisionObject()
    collision_obj.header.frame_id = "world"
    collision_obj.id = body_name

    geom_ids = np.arange(body_model.geomadr, body_model.geomadr + body_model.geomnum)

    for geom_id in geom_ids:
        geom = model.geom(geom_id)

        if geom.contype == 0:
            # Skip geoms that are not used for collision detection
            continue

        geom_data = data.geom(geom_id)
        pose = Pose()
        pose.position = _as_point(geom_data.xpos)
        quat = Rotation.from_matrix(geom_data.xmat.reshape(3, 3)).as_quat()
        pose.orientation = _as_orientation(quat)

        if geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
            pass
            # plane = Plane()
            # plane.coef = geom.size
            # collision_obj.planes.append(plane)
        elif (
            geom.type == mujoco.mjtGeom.mjGEOM_MESH
            or geom.type == mujoco.mjtGeom.mjGEOM_SDF
        ):
            mesh_ids = geom.dataid
            for mesh_id in mesh_ids:
                mesh = Mesh()
                assert mesh_id >= 0, "Mesh data ID should be non-negative"
                if use_convex_hull:
                    # Convex hull
                    gadr = model.mesh_graphadr[mesh_id]
                    vertadr = model.mesh_vertadr[mesh_id]

                    numvert = model.mesh_graph[gadr]
                    numface = model.mesh_graph[gadr + 1]

                    # Global indices for vertices
                    off_vert_edgeadr = gadr + 2
                    off_vert_global_id = off_vert_edgeadr + numvert
                    off_edge_local_id = off_vert_global_id + numvert
                    off_face_global_id = off_edge_local_id + (numvert + 3 * numface)

                    vert_gids = model.mesh_graph[
                        off_vert_global_id : off_vert_global_id + numvert
                    ]
                    face_gids = model.mesh_graph[
                        off_face_global_id : off_face_global_id + 3 * numface
                    ]

                    mesh.vertices = [
                        _as_point(model.mesh_vert[vertadr + gid]) for gid in vert_gids
                    ]

                    gid2local = {gid: i for i, gid in enumerate(vert_gids)}

                    mesh.triangles = [
                        MeshTriangle(
                            vertex_indices=[
                                gid2local[face_gids[i]],
                                gid2local[face_gids[i + 1]],
                                gid2local[face_gids[i + 2]],
                            ]
                        )
                        for i in range(0, len(face_gids), 3)
                    ]

                else:
                    # Collect vertices and faces from the mesh
                    vert_start = model.mesh_vertadr[mesh_id]
                    face_start = model.mesh_faceadr[mesh_id]
                    # Full mesh
                    mesh.vertices = [
                        _as_point(v)
                        for v in model.mesh_vert[
                            vert_start : vert_start + model.mesh_vertnum[mesh_id]
                        ]
                    ]
                    mesh.triangles = [
                        MeshTriangle(vertex_indices=face)
                        for face in model.mesh_face[
                            face_start : face_start + model.mesh_facenum[mesh_id]
                        ]
                    ]

                collision_obj.meshes.append(mesh)
                collision_obj.mesh_poses.append(pose)

        elif geom.type == mujoco.mjtGeom.mjGEOM_BOX:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = (geom.size * 2).tolist()

            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(pose)
        elif geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [geom.size[0]]

            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(pose)

        elif geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [geom.size[1] * 2, geom.size[0]]

            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(pose)

        # SolidPrimitive does not support capsule, so we use a cylinder
        elif geom.type == mujoco.mjtGeom.mjGEOM_CAPSULE:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [geom.size[1] * 2, geom.size[0]]

            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(pose)
    return collision_obj
