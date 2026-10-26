# Published on /robot_status at a steady rate by every robot process hosting
# capabilities (easyfleet::Deployment, or easyfleet_core::RobotStatusPublisher).
# A robot whose capabilities run in several processes publishes one per
# process: a mission manager merges those of the same robot.

# Namespace of the robot (e.g. "robot1"); empty without a namespace.
string robot
builtin_interfaces/Time stamp

# The capabilities this process hosts, as in their own heartbeats.
CapabilityStatus[] capabilities

# The robot's pose and battery, when this process is configured to report them.
bool has_pose
geometry_msgs/PoseWithCovarianceStamped pose
bool has_battery
sensor_msgs/BatteryState battery
