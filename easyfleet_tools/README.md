
# easyfleet_tools

ROS 2 monitoring tools for EasyFleet:
- **TUI**: `ros2 run easyfleet_tools tui`
- **CLI ros2cli**: `ros2 easyfleet fleet|describe|watch|status|logs`

Read-only: shows what a fleet is doing (robots, capabilities, goal status
and feedback, RViz status markers, logs) without ever launching, stopping
or preempting anything itself.

## Run
```bash
ros2 run easyfleet_tools tui
ros2 easyfleet fleet
ros2 easyfleet describe robot_1/navigation
ros2 easyfleet watch robot_1/navigation --duration 10
ros2 easyfleet status --duration 10
ros2 easyfleet logs --duration 10
```
