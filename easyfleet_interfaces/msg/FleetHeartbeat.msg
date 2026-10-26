# Published on /fleet_heartbeat at a steady rate by every mission manager
# (FleetSession). Its presence, not its content, tells each capability that it
# is linked to the fleet: one that hears none for longer than its link_timeout
# applies its on_link_lost policy.

# Who publishes it (e.g. the mission manager's node name), for diagnostics.
string source
