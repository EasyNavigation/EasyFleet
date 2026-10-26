# State of one order (one goal) of a capability, as kept in the robot's recent
# history and returned by <capability>/get_order. The state values are those of
# action_msgs/GoalStatus.
uint8 STATE_UNKNOWN=0
uint8 STATE_ACCEPTED=1
uint8 STATE_EXECUTING=2
uint8 STATE_CANCELING=3
uint8 STATE_SUCCEEDED=4
uint8 STATE_CANCELED=5
uint8 STATE_ABORTED=6

# The goal's order_id, or its UUID (32 hex digits) when that was empty.
string order_id
# The goal's UUID, 32 hex digits.
string goal_id
# Fully-qualified name of the action that ran it (e.g. "/robot1/navigation").
string action_name
uint8 state
# When the capability accepted it, and when it reached a terminal state
# (zero while it has not).
builtin_interfaces/Time accepted_stamp
builtin_interfaces/Time finished_stamp
