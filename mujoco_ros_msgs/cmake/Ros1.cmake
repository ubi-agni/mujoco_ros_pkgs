find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    actionlib_msgs
)

add_message_files(
  FILES
    StateUint.msg
    ros1/ScalarStamped.msg
    BodyState.msg
    GeomProperties.msg
    GeomType.msg
    SensorNoiseModel.msg
    SolverParameters.msg
    MocapState.msg
    EqualityConstraintParameters.msg
    EqualityConstraintType.msg
    SimInfo.msg
    PluginStats.msg
)

add_service_files(
  FILES
    GetStateUint.srv
    SetFloat.srv
    SetPause.srv
    SetBodyState.srv
    GetBodyState.srv
    SetGeomProperties.srv
    GetGeomProperties.srv
    SetEqualityConstraintParameters.srv
    GetEqualityConstraintParameters.srv
    ResetBodyQPos.srv
    RegisterSensorNoiseModels.srv
    SetGravity.srv
    GetGravity.srv
    Reload.srv
    SetMocapState.srv
    GetSimInfo.srv
    GetPluginStats.srv
)

add_action_files(
  FILES
    Step.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    actionlib_msgs
)
