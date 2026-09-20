#!/usr/bin/env python3

# Python imports
import os
import yaml
import numpy as np
import time
from dataclasses import dataclass

# ROS imports
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from radar_msgs.msg import *
from mrs_msgs.srv import TrajectoryReferenceSrv
from mrs_msgs.msg import Reference

class MissingHeadingError(Exception):
    pass

class HeaderError(Exception):
    pass

class SamplingError(Exception):
    pass

class TrajectoryCollissionError(Exception):
    pass

class TrajectoryLoadingError(Exception):
    pass

@dataclass
class UAVTrajectory:
    uav_name:str
    trajectory_file_path: str
    timestamps: np.ndarray
    trajectory: np.ndarray
    loop: bool
    use_heading: bool
    force_load: bool



class TrajectoryLoader(Node):

    def __init__(self):
        super().__init__('multi_uav_trajectory_loader_node')

        self.declare_parameter('uav_name','')
        self.declare_parameter('config','')
        self.declare_parameter('service_config', '')
        self.declare_parameter('config_dir_path', '')

        self.uav_name = self.get_parameter('uav_name').get_parameter_value().string_value
        self.config_path = self.get_parameter('config').get_parameter_value().string_value
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.config_dir_path = self.get_parameter('config_dir_path').get_parameter_value().string_value
        self.service_config = self.get_parameter('service_config').get_parameter_value().string_value

        try:
            self.uav_configs = self.parse_config()
        except Exception as e:
            self._logger.error('Failed to parse config.')
            self._logger.error(f'{e}')
            return

        self._logger.info("Config parsed")

        try:
            self.collission_check()
        except Exception as e:
            self._logger.error('Mutual trajectory collision check has failed')
            self._logger.error(f'{e}')
            return
        self._logger.info("Collission check passed")

        try:
            self.start_services()
        except Exception as e:
            self._logger.error('Failed to initialize service clients')
            self._logger.error('Perhaps check zenoh config?')
            self._logger.error(f'{e}')
            return
        self._logger.info("Service clients found")

        try:
            self.send_trajectories()
        except Exception as e:
            self._logger.error('An exception occured when trying to send the trajectories:')
            self._logger.error(f'{e}')
        

    def parse_config(self) -> list:
        def log_config(d, parent_key=''):
            for key, value in d.items():
                full_key = f"{parent_key}.{key}" if parent_key else key
                if isinstance(value, dict):
                    log_config(value, full_key)
                else:
                    self._logger.info(f"{full_key}: {value}")

        # Read global config

        with open(self.config_path, 'r') as f:
            data = yaml.safe_load(f)
            log_config(data)
            
            category            = data['trajectory']
            self.frame_id       = category['frame_id']
            self.dt             = category['dt']
            self.global_offset  = category['global_offset']
            self.safety_margin  = category['safety_margin']
            self.lookahead_time = category['lookahead_time']
            uavs                = category['uavs']

        # Read config and trajectory for each UAV
        uav_configs = []
        for uav in uavs:

            uav_name             = uav
            trajectory_file_path = os.path.join(self.config_dir_path,uavs[uav]['filename'])
            loop                 = uavs[uav]['loop']
            local_offset         = uavs[uav]['local_offset']
            use_heading          = uavs[uav]['use_heading']
            force_load           = uavs[uav]['force_load']

            # Read trajectory file, determine whether it has a header
            with open(trajectory_file_path, 'r') as f:
                header = f.readline().strip().replace(" ", "").split(',')
                has_header = any(c.isalpha() for c in header)

            data = np.loadtxt(trajectory_file_path, delimiter=',', skiprows=int(has_header))

            # Because numpy maps singular points as just vectory
            if data.ndim == 1:
                data = data[np.newaxis, :]

            n_points, n_cols = data.shape

            if not has_header:
                if n_cols != 5:
                    raise HeaderError(f"Headerless trajectory files require [t,x,y,z,heading] columns, but {uavs[uav]['filename']} is missing some")
                timestamps = data[:,0]
                dt = np.mean(np.diff(timestamps))
                if not np.isclose(dt,self.dt,atol=1e-2):
                    raise SamplingError(f"Trajectory {uavs[uav]['filename']} are sampled at {dt}, but condig requires {self.dt}")
                trajectory = data[:,1:]

            else:
                if len(header) != n_cols:
                    raise HeaderError(f"The number of items in header and number of columns of data in {uavs[uav]['filename']} does not agree")

                if 't' in header:
                    timestamps = data[:,header.index('t')]
                    dt = np.mean(np.diff(timestamps))
                    if np.isclose(dt,self.dt,atol=1e-2):
                        raise SamplingError(f"Trajectory {uavs[uav]['filename']} are sampled at {dt}, but condig requires {self.dt}")
                else:
                    timestamps = np.arange(n_points) * self.dt
                
                if use_heading:
                    if 'heading' not in header:
                        raise HeaderError(f"{uav} requires heading, but {uavs[uav]['filename']} does not have it")
                    else:
                        heading = data[:,header.index('heading')]

                else:
                    heading = np.zeros(n_points)

                x,y,z = data[:,header.index('x')], data[:,header.index('y')], data[:,header.index('z')]

                trajectory = np.vstack([x,y,z,heading]).T

            trajectory += self.global_offset
            trajectory += local_offset

            uav_configs.append(UAVTrajectory(uav_name=uav_name,
                                         trajectory_file_path=trajectory_file_path,
                                         timestamps=timestamps,
                                         trajectory=trajectory,
                                         loop=loop,
                                         use_heading=use_heading,
                                         force_load=force_load))

        return uav_configs


    def collission_check(self):
        total_points = int(self.lookahead_time / self.dt+1) # how many trajectory points we have in one hour

        trajectories = np.zeros((len(self.uav_configs),total_points,3))
        uav_names = []
        for i, uav_config in enumerate(self.uav_configs):
            uav_names.append(uav_config.uav_name)
            trajectory = uav_config.trajectory
            trajectory_points = trajectory.shape[0]
            if uav_config.loop:
                N = total_points // trajectory_points + 1
                trajectory = np.tile(trajectory, (N, 1))
                trajectory = trajectory[:total_points]
            else:
                N = total_points - trajectory_points
                trajectory = np.vstack([trajectory, np.tile(trajectory[-1],(N,1))])

            trajectories[i] = trajectory[:,:3]
            # Appends trajectory to a list, and then calculate distances with that list, get binary mask of true values, and use nonzero search to get indices of trajectories and how many pointsa re in collisssion at what times

        i, j = np.triu_indices(trajectories.shape[0], k=1)

        dist_sq = np.sum((trajectories[i] - trajectories[j])**2, axis=-1)

        collision = dist_sq <= self.safety_margin**2

        has_collision = (collision).any(axis=1)

        # collisions = [timestamps, uav_idx1, uav_idx2]
        collisions = np.column_stack([np.argmax(collision, axis=1)[has_collision] * self.dt,
                                       i[has_collision],
                                       j[has_collision]])
        
        for i, uav_config in enumerate(self.uav_configs):
            if uav_config.force_load:
                collisions = collisions[(collisions[:, 1] != i) & (collisions[:, 2] != i)]

        if len(collisions) > 0:
            exception_str = "The following collisions were detected:"
            for col in collisions:
                exception_str += f"\nTrajectories of {uav_names[int(col[1])]} and {uav_names[int(col[2])]} collide at {np.round(col[0],2)} seconds"
            raise TrajectoryCollissionError(
                exception_str
            )
        return
    


    def start_services(self):
        self.service_dict = {}
        with open(self.service_config, 'r') as f:
            data = yaml.safe_load(f)['services']

            check_rate = data['availability_timer']
            check_timeout = data['availability_timeout']

            uavs = data['uavs']

            if len(uavs) != len(self.uav_configs):
                raise NameError(f'Mimatch between the number of UAVS within configs')
            
            for uav in self.uav_configs:
                if uav.uav_name not in uavs:
                    raise NameError(f'The UAV names inside configs do not match.')

            for uav in uavs:
                new_service = self.create_client(TrajectoryReferenceSrv,f"/{uav}{uavs[uav]}")
                self.service_dict[uav] = new_service

            T = time.monotonic()
            has_service = [False] * len(self.service_dict)

            for i, (uav, client) in enumerate(self.service_dict.items()):
                # wait_for_service handles executor graph updates internally
                if client.wait_for_service(timeout_sec=check_timeout):
                    has_service[i] = True
                else:
                    self.get_logger().error(f"Service for {uav} timed out.")

            # while time.monotonic() < T + check_timeout:
            #     for i, serv in enumerate(self.service_dict):
            #         has_service[i] = self.service_dict[serv].service_is_ready()
            #     if all(has_service):
            #         break
            #     time.sleep(check_rate)

            if not all(has_service):
                exception_str = "Connection to following services failed:"
                for i, (val, uav) in enumerate(zip(has_service,self.service_dict)):
                    if val == False:
                        exception_str += f"\n{self.service_dict[uav].srv_name}"
                raise TimeoutError(exception_str)
        return


    def send_trajectories(self):
        self.trajectory_loaded = [False] * len(self.uav_configs)

        for i, uav_config in enumerate(self.uav_configs):
            client = self.service_dict[uav_config.uav_name]

            request = self.create_request(
                loop=uav_config.loop, 
                use_heading=uav_config.use_heading, 
                points=uav_config.trajectory
            )

            # Send asynchronously and spin until the future resolves
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"{uav_config.uav_name}: Trajectory loaded")
                    self.trajectory_loaded[i] = True
                else:
                    self.get_logger().error(f"{uav_config.uav_name}: Failed to load trajectory: {response.message}")
            else:
                self.get_logger().error(f"{uav_config.uav_name}: Service call failed with exception: {future.exception()}")

        if all(self.trajectory_loaded):
            self.get_logger().info("All trajectories were loaded successfully")
        return

    def create_request(self, loop, use_heading, points):
        def to_ref(point:np.ndarray):
            ref = Reference()
            ref.position.x = point[0]
            ref.position.y = point[1]
            ref.position.z = point[2]
            ref.heading = point[3]
            return ref
        
        req = TrajectoryReferenceSrv.Request()
        req.trajectory.header.stamp = self.get_clock().now().to_msg()
        req.trajectory.header.frame_id = self.frame_id
        req.trajectory.dt = self.dt
        req.trajectory.loop = loop
        req.trajectory.use_heading = use_heading
        req.trajectory.fly_now = False
        req.trajectory.points = [to_ref(x) for x in points]
        return req
    

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = TrajectoryLoader()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
