# This message is part of the IHMC whole-body controller API.
# It is a lightweight version of EuclideanTrajectoryMessage designed for streaming.

FrameInformation frame_information
# Flag that tells the controller whether the use of a custom control frame is requested.
bool use_custom_control_frame false
# Pose of custom control frame expressed in the end-effector frame.
# This is the frame attached to the rigid body that the taskspace trajectory is defined for.
geometry_msgs/Pose control_frame_pose

# Define the desired 3D position to be reached.
geometry_msgs/Point position
# Define the desired 3D linear velocity to be reached.
geometry_msgs/Vector3 linear_velocity
