Configuration
=============

MuJoCo ROS configuration is intentionally parameter-driven. Launch files load YAML into the ROS parameter server, the server reads those parameters during setup and reload, and plugins receive their own nested configuration through ``MujocoPlugins``.

Camera Configuration
--------------------

MuJoCo cameras can publish ROS image streams without loading the sensors plugin.
Configure each named camera under ``cam_config/<camera_name>``:

.. code-block:: yaml

   cam_config:
     workspace_cam:
       stream_type: 7
       frequency: 15
       width: 720
       height: 480
       use_segid: true
       topic: cameras/workspace_cam
       name_rgb: rgb
       name_depth: depth
       name_segment: segmented

``stream_type`` is a bit mask: RGB is ``1``, depth is ``2``, and segmentation is ``4``.
Add values to enable multiple streams; ``7`` enables all three.

The default topic layout is:

* ``cameras/<camera_name>/rgb/image_raw``
* ``cameras/<camera_name>/rgb/camera_info``
* ``cameras/<camera_name>/depth/image_raw``
* ``cameras/<camera_name>/depth/camera_info``
* ``cameras/<camera_name>/segmented/image_raw``
* ``cameras/<camera_name>/segmented/camera_info``

Relative ``topic`` values are resolved in the server namespace.
Absolute topics are used as provided. Rendering is lazy: an offscreen camera only renders the streams that currently have image or camera-info subscribers.

Initial Joint States
--------------------

Initial joint states are configured as maps from MuJoCo joint names to string values:

.. code-block:: yaml

   initial_joint_positions:
     joint_map:
       hinge_joint: "0.0"
       ball_joint: "1 0 0 0"
       free_joint: "0 0 1 1 0 0 0"

   initial_joint_velocities:
     joint_map:
       hinge_joint: "0.0"
       ball_joint: "0 0 0"
       free_joint: "0 0 0 0 0 0"

The values are applied when the model loads, resets, reloads, or when ``load_initial_joint_states`` is called.

Evaluation Mode
---------------

``eval_mode`` restricts operations that should not be available during evaluation.
Requests that change simulation state include an ``admin_hash`` field; in evaluation mode the request hash must match the server's configured hash.
The sensors plugin also suppresses ground-truth topics in evaluation mode.
