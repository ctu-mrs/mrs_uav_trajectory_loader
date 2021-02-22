# UAV trajectory loader

| Build status | [![Build Status](https://github.com/ctu-mrs/trajectory_loader/workflows/Melodic/badge.svg)](https://github.com/ctu-mrs/trajectory_loader/actions) | [![Build Status](https://github.com/ctu-mrs/trajectory_loader/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/trajectory_loader/actions) |
|--------------|---------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|

Launchfiles and config files are only examples how to use this package. It is recommended to copy them into your own package and modify them there.

## Commanding single client machines

### Load trajectory into MPC tracker from CSV or SSV file

Loading is done by calling:
```bash
roslaunch trajectory_handler single_uav.launch path:=<folder> file:=<file>
```
where parameters *\<folder\>* and *\<file\>* are the absolute path and filename of the trajectory file.
Furthermore, these parameters of the trajectory can be set by modifying the launch file:
* `trajectory/offset`       - offset for the whole trajectory `[x, y, z, heading]`.
* `trajectory/delay`        - sleep before loading the trajectory (also used as delay for *goto start* and *start tracking*).
* `trajectory/use_heading`  - whether the heading controller should follow the heading from the trajectory (otherwise it will just point in one direction).
* `trajectory/fly_now`      - whether the trajectory should be followed immediately after its loading. 
* `trajectory/loop`         - whether the trajectory is infinite. Trajectory will be then looped (after the last point is reached, the UAV will be commanded to the first point again).

*Note*: If you intend to create your own launchfile, please note the `trajectory/dynamic_uav_name` parameter.
If set to `true`, the trajectory will be loaded using the filename `trajectory/filename`, which is agnostic with respect to the `uav_name` parameter (whatever you put in `trajectory/uavs/<uav_name>/filename` will be ignored).
This is useful when loading a trajectory for a single UAV and the name of the UAV should be loaded eg. from an environment variable, so that you don't have to specify it in the trajectories config file.

### Sending command "Go to start"

To command the UAV to go to the first point of the trajectory, you can manually issue the command
```bash
rosservice call /$UAV_NAME/control_manager/goto_trajectory_start
```
or use `trajectory_loader`:
```bash
roslaunch trajectory_handler single_uav.launch mode:=goto
```
Note that if the `trajectory/delay` parameter is specified, the *goto* command will be delayed. You can disable this by setting `trajectory/delay` to zero.

### Sending command "Start tracking"

To command the UAV start tracking the trajectory, you can manually issue the command
```bash
rosservice call /$UAV_NAME/control_manager/start_trajectory_tracking
```
or use `trajectory_loader`:
```bash
roslaunch trajectory_handler single_uav.launch mode:=track
```
Note that if the `trajectory/delay` parameter is specified, the *track* command will be delayed. You can disable this by setting `trajectory/delay` to zero.

### Sending command "Stop following"

To command the UAV to stop tracking the trajectory, you can manually issue the command
```bash
rosservice call /$UAV_NAME/control_manager/stop_trajectory_tracking
```
or use `trajectory_loader`:
```bash
roslaunch trajectory_handler single_uav.launch mode:=stop
```
Note that if the `trajectory/delay` parameter is specified, the *stop* command will be delayed. You can disable this by setting `trajectory/delay` to zero.

## Commanding multiple client machines

### Prerequisites

Setting which UAVs are targets and also which trajectories should be loaded has to be defined in a config file (see `/config/example_params.yaml` for an example).
The parameter `trajectory/uavs` sets the specific options for the different UAVs.
Each UAV can have a different trajectory offset, delay, looping etc. (parameters for UAV with name *\<uav_name\>* are specified under `trajectory/uavs/<uav_name>/...`).
These override the common parameters, set in the `trajectory/offset`, `trajectory/delay`, ... parameters.

*Note*: If you intend to create your own launchfile, please note the `trajectory/dynamic_uav_name` parameter.
If set to `false`, the trajectories will be loaded using the filenames, specified in `trajectory/uavs/<uav_name>/filename` (similarly for parameters of the respective trajectories).
UAV names will then be automatically deduced from what is filled in `trajectory/uavs/...`.
This is useful when loading a trajectories for a multiple UAVs with set names.

### For simulations via LAN
Follow to [how to set ros remote](https://mrs.felk.cvut.cz/gitlab/uav/uav_core/wikis/ros_remote) page.

### For real UAV
To have correctly set multimaster net using package `multimaster_fkie` or `nimbro_network`. (Usually it is already prepared).

### Load trajectories into MPC tracker from CSV or SSV files

Loading is done by calling
```bash
roslaunch trajectory_handler load.launch config:=<path_to_your_trajectories_config>
```
or create your own launchfile similar to `load.launch` and use that one.

### Sending command "Go to start"

Use the command
```bash
roslaunch trajectory_handler goto_start.launch config:=<path_to_your_trajectories_config>
```
or create your own launchfile similar to `goto_start.launch` and use that one.

### Sending command "Start tracking"

Use the command
```bash
roslaunch trajectory_handler start_tracking.launch config:=<path_to_your_trajectories_config>
```
or create your own launchfile similar to `start_tracking.launch` and use that one.

