# MRS UAV trajectory loader (multi-uav version)

Adjusted version of the `mrs_uav_trajectory_loader` package. It provides a centralized
option for loading multiple trajectories into multiple UAVs simultaneously.

The trajectories are loaded from config-specified CSV/TXT files into the control manager.
This task requires Zenoh communication between the UAV platforms, namely at least the ability to use `/control_manager/trajectory_reference`

The trajectories itself differ slightly, as they either require to have additional first column for timestamp, or have a header present.

The loaded trajectories are tracked by the respective running tracker algorithm on each platform.
> Currently, the trajectory loading is only supported by the `MpcTracker`.

## Config Parameters
```yaml
trajectory:
  # The trajectory waypoints are defined in this frame.
  # The frame name is automatically prefixed with the UAV name at runtime
  # e.g. "world_origin" becomes "uav1/world_origin" for UAV named "uav1"
  # Only world origin is currently available, as the current system
  # does not allow subscription of tf from other uavs
  frame_id: world_origin 

  # Time step between consecutive waypoints in seconds
  # Must be >= 0.01 s (the control loop dt for MpcTracker)
  # Typical values: 0.1 (10 Hz), 0.2 (5 Hz)
  # If trajectories contain timestamps, the sampling is checked against this value
  # Trajectory-specific sampling is not allowed, as trajectories can be dynamically sampled
  dt: 0.2

  # Static offset added to every waypoint in all trajectories
  # Format: [x, y, z, heading]
  # Units: meters for x/y/z, radians for heading
  # Useful for translating all trajectories without modifying their respective file
  # e.g. [1.0, 0.0, 0.5, 0.0] shifts all points 1m in x and 0.5m in z
  global_offset: [0.0, 0.0, 0.0, 0.0] # [x, y, z, heading] offset from the frame_id (shifts the origin point for the trajectories)

  # Minimal distance used for the simple collision avoidance
  # Denoted by radiues with units in meters
  # Note: Keep the value >= 2.0, unless you know what you are doing
  safety_margin: 2.0

  # Limits how far into the the future (in seconds) we check for collissions
  # Default value is 3600.0 seconds (1 hour) assuming UAVs is then more likely to run outout of
  # battery than collide with another UAV
  lookahead_time: 3600.0

  # What UAVs we have, and what arguments we want to use for them

  # Mandatory UAV Argument: filename
  # Path to specific trajectory file
  # The trajectory files can be categorized to two types, header and headerless
  # Headerless trajectory files are required to contain 5 colums for [t,x,y,z,heading]
  # Trajectory files with header has to contain [x,y,z], the rest is optinal (but might be required by other params)

  # Mandatory UAV Argument: local_offset
  # Format: [x, y, z, heading]
  # Units: meters for x/y/z, radians for heading
  # Additional offset addded to the global offset (useful when all trajectories are [0,0,0] based)
  # e.g. [-1.0, 2.0, 0.0, 0.0] additionally shifts points within given trajectory by -1m in x and 2m in y

  # Mandatory UAV Argument: loop
  # Whether to loop the trajectory after reaching the last waypoint
  # true  - continuously repeat the trajectory
  # false - stop and hover at the last waypoint

  # Mandatory UAV Argument: use_heading
  # Whether to use the heading values specified in the trajectory file
  # true  - UAV follows the heading column in the trajectory file
  # false - heading is left to the heading controller independently
  # Note: trajectory file must have a heading column if true

  # Mandatory UAV Argument: force_load
  # Whether to ignore collision check result with this trajectory
  # true  - Ignore collision check for this trajectory
  # false - Check for possible collisions 
  # Note: ALWAYS SET TO FALSE, unless you have understanding of trajectory loading/tracking pipeline

  uavs:
    uav1:
      filename: "trajectory/circle.txt"
      local_offset: [0.0, 0.0, 0.0, 0.0] # [x, y, z, heading]
      loop: true
      use_heading: true
      force_load: false
    uav2: 
      filename: "trajectory/circle.txt"
      local_offset: [0.0, 0.0, 3.0, 0.0] # [x, y, z, heading]
      loop: true
      use_heading: true
      force_load: false
    # uav3:
    #   filename: "trajectory/circle.txt"
    #   local_offset: [0.0, 0.0, 6.0, 0.0] # [x, y, z, heading]
    #   loop: true
    #   use_heading: true
    #   force_load: false
```

## Launch Parameters
- `uav_name`: Automatically taken from ```UAV_NAME``` environment variable. Can be blank, if the package is not run from UAV
- `use_sim_time`: Automatically taken from ```USE_SIM_TIME``` environment variable. Change it if using with ```ros2 bag play```.
- `debug`: Set to ```true``` if you need the node running inside a ```gdb``` session.
- `config`: Provide a custom config file if you want to override the default parameters.
- `service_config`: Provide a custom config file if you want to override the default parameters.

## Usage:
It is recommended to have the following tasks automated by a script.
### Loading trajectory into Control Manager from CSV or SSV file

Loading custom trajectories is done by calling:
```bash
ros2 launch mrs_multi_uav_trajectory_loader trajectory_loader.launch.py config:=custom.yaml
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


