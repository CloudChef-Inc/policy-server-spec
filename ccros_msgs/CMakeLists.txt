cmake_minimum_required(VERSION 3.8)
project(ccros_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointReference.msg"
  "msg/ArmGains.msg"
  "srv/SetControlMode.srv"
  "srv/SetArmGains.srv"
  "action/FollowJointTrajectoryGains.action"
  DEPENDENCIES std_msgs trajectory_msgs action_msgs
)

ament_package()
