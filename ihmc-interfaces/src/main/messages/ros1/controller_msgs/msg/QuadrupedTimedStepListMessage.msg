# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Defines the list of steps to perform
controller_msgs/QuadrupedTimedStepMessage[] quadruped_step_list

# If true, the times are expressed in absolute time relative to the robot start
# Field default value True
bool is_expressed_in_absolute_time

# Properties for queueing footstep lists.
controller_msgs/QueueableMessage queueing_properties


