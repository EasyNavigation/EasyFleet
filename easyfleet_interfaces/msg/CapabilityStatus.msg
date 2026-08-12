# Published at a steady rate on /capabilities_status by every active
# capability. Part of the arch_mockup architecture contract: its presence,
# not its content, is the "this capability is alive" signal a control
# center relies on -- a capability that stops publishing this is considered
# inactive even if it once announced itself on /capabilities.

# Namespace this capability is running under (e.g. "robot1"). Empty if the
# capability was launched without a namespace.
string robot

# Short capability type (e.g. "navigation"). Matches the `capability` field
# of the corresponding CapabilityDescription.
string capability

# Fully-qualified name of the ROS 2 action this capability exposes (e.g.
# "/robot1/navigation"). Matches the `action_name` field of the
# corresponding CapabilityDescription; unique per running capability, so it
# is the right key to use when tracking "is this specific one still alive".
string action_name

# True while this capability has a goal currently executing, false while it
# is idle and ready to accept one. Lets an orchestrator tell "alive but
# busy" apart from "alive and free" without having to send a goal (or a
# preempting one) just to find out.
bool busy
