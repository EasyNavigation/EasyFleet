# Published on the reliable, transient-local /capabilities topic by every
# capability once activated. This is part of the EasyFleet architecture
# contract: any capability -- regardless of who implements it or in what
# language -- must publish one of these, with the identity fields below
# filled in truthfully, to be discoverable by a mission manager.

# Namespace this capability is running under (e.g. "robot1"). Empty if the
# capability was launched without a namespace.
string robot

# Short capability type, shared by every robot that offers it (e.g.
# "navigation"). Two robots can both publish capability "navigation" with
# different `robot`/`action_name` values.
string capability

# Fully-qualified name of the ROS 2 action this capability exposes (e.g.
# "/robot1/navigation"). This is what a client must actually connect to.
string action_name

# Free-form JSON with the rich, human/LLM-facing description: display_name,
# description, requirements, effects, parameters, preemption, notes, ...
# Its exact shape is a documented convention (see the EasyFleet README),
# not enforced by this message type, since it is expected to vary and grow
# per capability.
string description_json
