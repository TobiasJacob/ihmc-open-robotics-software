# This message is part of the KinematicsStreamingToolbox API.
# Allows to specify the messages the toolbox should stream to the controller.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Whether the pelvis should pinned or free to move.
# Default value is false.
bool lock_pelvis false
# Whether the chest should pinned or free to move.
# Default value is false.
bool lock_chest false

# Whether the left arm should be controller in joint-space.
# This is compatible with the hand task-space control.
# Default value is true.
bool enable_left_arm_jointspace true

# Whether the right arm should be controller in joint-space.
# This is compatible with the hand task-space control.
# Default value is true.
bool enable_right_arm_jointspace true

# Whether the neck should be controller in joint-space.
# Default value is true.
# Whether the neck should be controller in joint-space.
bool enable_neck_jointspace true

# Whether the left hand should be controlled in task-space.
# This is compatible with the arm task-space control.
# Default value is true.
bool enable_left_hand_taskspace true

# Whether the right hand should be controlled in task-space.
# This is compatible with the arm task-space control.
# Default value is true.
bool enable_right_hand_taskspace true

# Whether the chest orientation should be controlled.
# Default value is true.
bool enable_chest_taskspace true

# Whether the pelvis should be controlled in task-space.
# Default value is true.
bool enable_pelvis_taskspace true
