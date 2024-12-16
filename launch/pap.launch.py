from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  ld = LaunchDescription()

  ld.add_action(
    Node(
      package="omx_moveit",
      executable="object_detection_aruco.py",
    )
  )

  ld.add_action(
    Node(
      package="omx_moveit",
      executable="robot_execute",
    )
  )

  ld.add_action(
    Node(
      package="omx_moveit",
      executable="pick_and_place",
    )
  )

  ld.add_action(
    Node(
      package="omx_moveit",
      executable="pickup",
    )
  )

  ld.add_action(
    Node(
      package="omx_moveit",
      executable="drop",
    )
  )

  return ld