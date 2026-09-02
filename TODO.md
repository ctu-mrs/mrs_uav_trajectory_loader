### TODO
1. Create new package (currently in python only, after testing to be rewritten into C++, which shoudl be quick)
2. Define uav names, trajectories, offsets, reference frames and samepling time in single yaml file (refer to ros1 version)
3. Trajectories will contain additional columns for timestamp, from timestmap we can obtain dt of the trajectory and check whether it agrees with the provided config, if not, throw error (remove possible user error in miscofiguration of the sampling)
4. Before sending the trajectories to the UAVs, check them first within a time (loop trajectories might be slightly more difficult to do, however, we do not need to check further than 1 hour into the future, make it a param in the config)
5. Apply offsets and upload them to all UAVs, check ROS1 version how to manage synchronious tracking (refer to start_trajectory_tracking launch file or something similar iirc)
6. Do not forget to update readme, to let the user know to use zenoh_config.jsoN5 to setup communication between the uavs. The launch file could be possibly called from personal pc or another UAV, so try to also resolve that


Optionally: Add a script that will take desired UAVs position, and use it as a global offset position (will change all offsets in all yaml files, pass heading as optinal argument, but then it will be identicall to local origin). I guess a ros2 run should suffice for it, or even a standalone python script
