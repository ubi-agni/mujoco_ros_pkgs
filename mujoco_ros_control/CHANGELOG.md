# 3.0.0
## Changes
1. Updated `MujocoRosControlPlugin` according to changes in `MujocoPlugin` made in `mujoco\_ros` version 3.0.0.

Robot namespace is now fetched from the node handle instead of requiring an extra ros param.
`node_handle_` was removed, as the node handle pointer passed on init should be used.
