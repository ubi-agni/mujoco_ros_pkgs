find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/StateUint.msg"
    "msg/ScalarStamped.msg"
    "msg/BodyState.msg"
    "msg/GeomProperties.msg"
    "msg/GeomType.msg"
    "msg/SensorNoiseModel.msg"
    "msg/SolverParameters.msg"
    "msg/MocapState.msg"
    "msg/EqualityConstraintType.msg"
    "msg/EqualityConstraintParameters.msg"
    "msg/SimInfo.msg"
    "msg/PluginStats.msg"

    "srv/GetStateUint.srv"
    "srv/SetFloat.srv"
    "srv/SetPause.srv"
    "srv/SetBodyState.srv"
    "srv/GetBodyState.srv"
    "srv/SetGeomProperties.srv"
    "srv/GetGeomProperties.srv"
    "srv/SetEqualityConstraintParameters.srv"
    "srv/GetEqualityConstraintParameters.srv"
    "srv/ResetBodyQPos.srv"
    "srv/RegisterSensorNoiseModels.srv"
    "srv/SetGravity.srv"
    "srv/GetGravity.srv"
    "srv/Reload.srv"
    "srv/SetMocapState.srv"
    "srv/GetSimInfo.srv"
    "srv/GetPluginStats.srv"

    "action/Step.action"
    DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

ament_package()
