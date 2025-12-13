#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from cartographer_ros_msgs.srv import WriteState
import numpy as np
import yaml
import os
import cv2


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        
        self.declare_parameter('maps_directory', '/home/bench207/ros2_ws/maps')
        self.declare_parameter('map_name', 'my_map')
        self.declare_parameter('save_interval', 15.0)
        
        self.maps_dir = self.get_parameter('maps_directory').value
        self.map_name = self.get_parameter('map_name').value
        self.save_interval = self.get_parameter('save_interval').value
        
        os.makedirs(self.maps_dir, exist_ok=True)
        
        self.latest_map = None
        
        # Subscribe to map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Client to save Cartographer state
        self.write_state_client = self.create_client(WriteState, '/write_state')
        
        # Timer to auto-save
        self.create_timer(self.save_interval, self.save_all)
        
        # Service to trigger manual save
        self.save_service = self.create_service(Empty, '/save_map', self.save_service_callback)
        
        self.get_logger().info(f'Map saver started')
        self.get_logger().info(f'  Directory: {self.maps_dir}')
        self.get_logger().info(f'  Auto-save every {self.save_interval}s')
        self.get_logger().info(f'  Call /save_map service for manual save')
    
    def map_callback(self, msg):
        self.latest_map = msg
    
    def save_service_callback(self, request, response):
        self.get_logger().info('Manual save requested')
        self.save_all()
        return response
    
    def save_all(self):
        """Save map image, yaml, and pbstream."""
        self.save_map_image()
        self.save_pbstream()
    
    def save_map_image(self):
        """Save map as PGM + YAML."""
        if self.latest_map is None:
            self.get_logger().debug('No map received yet')
            return
        
        map_msg = self.latest_map
        
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        # Convert occupancy grid to image
        data = np.array(map_msg.data).reshape((height, width))
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 205   # Unknown = gray
        img[data == 0] = 254    # Free = white
        img[data == 100] = 0    # Occupied = black
        
        # Flip vertically (ROS uses bottom-left origin)
        img = cv2.flip(img, 0)
        
        # Save PGM
        pgm_path = os.path.join(self.maps_dir, f'{self.map_name}.pgm')
        cv2.imwrite(pgm_path, img)
        
        # Save YAML
        yaml_path = os.path.join(self.maps_dir, f'{self.map_name}.yaml')
        yaml_data = {
            'image': f'{self.map_name}.pgm',
            'resolution': float(resolution),
            'origin': [float(origin_x), float(origin_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f)
        
        self.get_logger().info(f'Map image saved: {pgm_path}')
    
    def save_pbstream(self):
        """Save Cartographer state as pbstream."""
        if not self.write_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Cartographer /write_state service not available')
            return
        
        pbstream_path = os.path.join(self.maps_dir, f'{self.map_name}.pbstream')
        
        request = WriteState.Request()
        request.filename = pbstream_path
        request.include_unfinished_submaps = True
        
        future = self.write_state_client.call_async(request)
        future.add_done_callback(self.pbstream_save_callback)
    
    def pbstream_save_callback(self, future):
        try:
            response = future.result()
            if response.status.code == 0:
                pbstream_path = os.path.join(self.maps_dir, f'{self.map_name}.pbstream')
                self.get_logger().info(f'Cartographer state saved: {pbstream_path}')
            else:
                self.get_logger().error(f'Failed to save pbstream: {response.status.message}')
        except Exception as e:
            self.get_logger().error(f'pbstream save error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Saving final state before exit...')
        node.save_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()