# MRS UAV trajectory loader

Package for loading trajectories from a CSV file into the control manager.
The loaded trajectory is tracked by the currently running tracker algorithm.
> Currently, the trajectory loading is only supported by the `MpcTracker`.

## Config Parameters
```yaml
trajectory:
  # The trajectory waypoints are defined in this frame.
  # The frame name is automatically prefixed with the UAV name at runtime
  # e.g. "local_origin" becomes "uav1/local_origin" for UAV named "uav1"
  # Common options:
  #   "local_origin"  - origin set at UAV spawn/takeoff position
  #   "gps_origin"    - origin tied to a fixed GPS coordinate
  #   "utm_origin"    - UTM coordinate frame
  #   "fcu"           - body frame of the UAV
  frame_id: local_origin

  # Time step between consecutive waypoints in seconds
  # Must be >= 0.01 s (the control loop dt for MpcTracker)
  # Typical values: 0.1 (10 Hz), 0.2 (5 Hz)
  dt: 0.1

  # Whether to loop the trajectory after reaching the last waypoint
  # true  - continuously repeat the trajectory
  # false - stop and hover at the last waypoint
  loop: false

  # Whether to use the heading values specified in the trajectory file
  # true  - UAV follows the heading column in the trajectory file
  # false - heading is left to the heading controller independently
  # Note: trajectory file must have a heading column if true
  use_heading: true

  # Whether to start flying the trajectory immediately after it is loaded
  # true  - begins tracking as soon as the trajectory is loaded
  # false - waits for a service call to /UAV_NAME/control_manager/start_trajectory_tracking
  fly_now: true

  # Static offset added to every waypoint in the trajectory
  # Format: [x, y, z, heading]
  # Units: meters for x/y/z, radians for heading
  # Useful for translating an entire trajectory without modifying the file
  # e.g. [1.0, 0.0, 0.5, 0.0] shifts all points 1m in x and 0.5m in z
  offset: [0.0, 0.0, 0.0, 0.0]
```

## Launch Parameters
- `uav_name`: Automatically taken from ```bash UAV_NAME``` environment variable.
- `use_sim_time`: Automatically taken from ```bash USE_SIM_TIME``` environment variable. Change it if using with ```bash ros2 bag play```.
- `debug`: Set to ```bash true``` if you need the node running inside a ```bash gdb``` session.
- `custom_config`: Provide a custom config file if you want to override the default parameters.
- `traj_file`: Provide a custom config file if you want to override the default parameters.

## Usage:
### Loading trajectory into Control Manager from CSV or SSV file

Loading is done by calling:
```bash
ros2 launch mrs_uav_trajectory_loader trajectory_loader.launch.py traj_file:=<path-to-file>
```
### Sending command "Go to start"

To command the UAV to go to the first point of the trajectory, you can manually issue the command
```bash
ros2 service call /$UAV_NAME/control_manager/goto_trajectory_start
```
### Sending command "Start tracking"

To start tracking the trajectory, you can manually issue the command
```bash
ros2 service call /$UAV_NAME/control_manager/start_trajectory_tracking
```
### Sending command "Stop tracking"

To stop tracking the trajectory, you can manually issue the command
```bash
ros2 service call /$UAV_NAME/control_manager/stop_trajectory_tracking
```
