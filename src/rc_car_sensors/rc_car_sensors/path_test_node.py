#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
import tf2_ros
import json
import os
import math

from rc_car_sensors.path_planner import PathPlanner


class AutonomousNavNode(Node):
    def __init__(self):
        super().__init__('autonomous_nav_node')
        
        # Parameters
        self.declare_parameter('tags_file', '/home/bench207/ros2_ws/tag_positions.json')
        self.declare_parameter('map_yaml', '/home/bench207/ros2_ws/maps/my_map.yaml')
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('waypoint_tolerance', 0.2)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('obstacle_distance', 0.35)
        self.declare_parameter('start_delay', 10.0)
        self.declare_parameter('enable_motor_control', False)  # Disabled for testing
        self.declare_parameter('sounds_directory', '/home/bench207/ros2_ws/sounds')
        
        self.tags_file = self.get_parameter('tags_file').value
        self.map_yaml = self.get_parameter('map_yaml').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.start_delay = self.get_parameter('start_delay').value
        self.enable_motor_control = self.get_parameter('enable_motor_control').value
        self.sounds_dir = self.get_parameter('sounds_directory').value
        
        # Tag config for sounds and colors
        self.tag_config = {
            0: {'sound': 'sound_0.mp3', 'name': 'Start', 'color': (0.0, 1.0, 0.0)},
            1: {'sound': 'sound_1.mp3', 'name': 'Checkpoint 1', 'color': (0.0, 0.0, 1.0)},
            2: {'sound': 'sound_2.mp3', 'name': 'Checkpoint 2', 'color': (1.0, 1.0, 0.0)},
            3: {'sound': 'sound_3.mp3', 'name': 'Treasure', 'color': (1.0, 0.5, 0.0)},
            4: {'sound': 'sound_4.mp3', 'name': 'Finish', 'color': (1.0, 0.0, 0.0)},
        }
        
        # State
        self.tag_positions = []  # List of (x, y, name, tag_id)
        self.current_goal_index = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.active = False
        self.localized = False
        
        # Path following
        self.current_path = []
        self.current_waypoint_index = 0
        
        # Path planner
        self.planner = None
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/planned_path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/current_goal', 10)
        self.tag_marker_pub = self.create_publisher(MarkerArray, '/tag_markers', 10)
        
        # Timers
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.05, self.update_robot_pose)
        self.create_timer(0.5, self.publish_path_visualization)
        self.create_timer(1.0, self.publish_tag_markers)
        
        # Load tags immediately so they show in Foxglove
        self.load_tags()
        
        # Load planner
        self.load_planner()
        
        # Auto-start after delay
        self.get_logger().info(f'Autonomous navigation will start in {self.start_delay} seconds...')
        self.get_logger().info(f'Motor control: {"ENABLED" if self.enable_motor_control else "DISABLED (test mode)"}')
        self.create_timer(self.start_delay, self.auto_start, callback_group=None)
        
    def auto_start(self):
        """Called once after start_delay seconds."""
        if not self.active:
            self.start_navigation()
        # Cancel this timer after first call
        return
    
    def load_tags(self):
        """Load tag positions from JSON file."""
        if not os.path.exists(self.tags_file):
            self.get_logger().error(f'Tags file not found: {self.tags_file}')
            return False
        
        with open(self.tags_file, 'r') as f:
            tags_data = json.load(f)
        
        self.tag_positions = []
        for tag_id, data in sorted(tags_data.items(), key=lambda x: int(x[0])):
            tag_id_int = int(tag_id)
            config = self.tag_config.get(tag_id_int, {'name': f'Tag {tag_id}'})
            self.tag_positions.append((
                data['x'],
                data['y'],
                config['name'],
                tag_id_int
            ))
        
        self.get_logger().info(f'Loaded {len(self.tag_positions)} tags:')
        for i, (x, y, name, tag_id) in enumerate(self.tag_positions):
            self.get_logger().info(f'  {i}: {name} (ID {tag_id}) at ({x:.2f}, {y:.2f})')
        
        return True
    
    def load_planner(self):
        """Load the path planner with saved map."""
        if not os.path.exists(self.map_yaml):
            self.get_logger().error(f'Map file not found: {self.map_yaml}')
            return False
        
        try:
            self.planner = PathPlanner(self.map_yaml)
            self.get_logger().info(f'Path planner loaded with map: {self.map_yaml}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load planner: {e}')
            return False
    
    def publish_tag_markers(self):
        """Publish tag positions as markers for Foxglove."""
        marker_array = MarkerArray()
        
        for i, (x, y, name, tag_id) in enumerate(self.tag_positions):
            config = self.tag_config.get(tag_id, {
                'name': name,
                'color': (0.5, 0.5, 0.5)
            })
            
            # Cylinder marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'tags'
            marker.id = tag_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.3
            marker.color.r = float(config['color'][0])
            marker.color.g = float(config['color'][1])
            marker.color.b = float(config['color'][2])
            marker.color.a = 0.8
            marker.lifetime.sec = 0
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'tag_labels'
            text_marker.id = tag_id + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.4
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = name
            text_marker.lifetime.sec = 0
            marker_array.markers.append(text_marker)
            
            # Highlight current goal
            if self.active and i == self.current_goal_index:
                ring_marker = Marker()
                ring_marker.header.frame_id = 'map'
                ring_marker.header.stamp = self.get_clock().now().to_msg()
                ring_marker.ns = 'current_goal_ring'
                ring_marker.id = 200
                ring_marker.type = Marker.CYLINDER
                ring_marker.action = Marker.ADD
                ring_marker.pose.position.x = x
                ring_marker.pose.position.y = y
                ring_marker.pose.position.z = 0.02
                ring_marker.pose.orientation.w = 1.0
                ring_marker.scale.x = 0.5
                ring_marker.scale.y = 0.5
                ring_marker.scale.z = 0.02
                ring_marker.color.r = 1.0
                ring_marker.color.g = 1.0
                ring_marker.color.b = 0.0
                ring_marker.color.a = 0.5
                marker_array.markers.append(ring_marker)
        
        self.tag_marker_pub.publish(marker_array)
    
    def start_navigation(self):
        """Start autonomous navigation."""
        if not self.tag_positions:
            self.get_logger().error('No tags loaded!')
            return
        
        if not self.planner:
            self.get_logger().error('Path planner not loaded!')
            return
        
        if not self.localized:
            self.get_logger().warn('Robot may not be fully localized yet, proceeding anyway...')
        
        self.current_goal_index = 0
        self.active = True
        
        self.get_logger().info('Navigation started!')
        
        # Plan path to first tag
        self.plan_to_current_goal()
    
    def plan_to_current_goal(self):
        """Plan path from current position to current goal."""
        if self.current_goal_index >= len(self.tag_positions):
            return False
        
        goal_x, goal_y, goal_name, tag_id = self.tag_positions[self.current_goal_index]
        
        self.get_logger().info(f'Planning path to {goal_name} at ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info(f'Robot at ({self.robot_x:.2f}, {self.robot_y:.2f})')
        
        start = (self.robot_x, self.robot_y)
        goal = (goal_x, goal_y)
        
        self.current_path = self.planner.plan(start, goal)
        
        if self.current_path is None:
            self.get_logger().error(f'No path found to {goal_name}!')
            self.current_path = []
            return False
        
        self.current_waypoint_index = 0
        self.get_logger().info(f'Path planned with {len(self.current_path)} waypoints')
        
        return True
    
    def stop_navigation(self):
        """Stop navigation."""
        self.active = False
        self.stop_robot()
        self.get_logger().info('Navigation complete!')
    
    def update_robot_pose(self):
        """Get robot pose from TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            
            q = transform.transform.rotation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            
            if not self.localized:
                self.localized = True
                self.get_logger().info(f'Localized at ({self.robot_x:.2f}, {self.robot_y:.2f})')
                
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
    
    def play_sound(self, tag_id):
        """Play sound for a tag."""
        import subprocess
        config = self.tag_config.get(tag_id)
        if config and 'sound' in config:
            sound_file = os.path.join(self.sounds_dir, config['sound'])
            if os.path.exists(sound_file):
                try:
                    subprocess.Popen(['mpg123', '-q', sound_file])
                    self.get_logger().info(f'Playing sound for tag {tag_id}')
                except Exception as e:
                    self.get_logger().error(f'Error playing sound: {e}')
    
    def control_loop(self):
        """Main control loop."""
        if not self.active:
            return
        
        # Check if all goals done
        if self.current_goal_index >= len(self.tag_positions):
            self.get_logger().info('All tags visited! Navigation complete!')
            self.stop_navigation()
            return
        
        goal_x, goal_y, goal_name, tag_id = self.tag_positions[self.current_goal_index]
        
        # Publish goal for visualization
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        self.goal_pub.publish(goal_msg)
        
        # Check if reached goal
        dist_to_goal = math.sqrt((goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2)
        
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f'Reached {goal_name}! (distance: {dist_to_goal:.2f}m)')
            self.play_sound(tag_id)
            self.current_goal_index += 1
            self.stop_robot()
            
            # Plan to next goal
            if self.current_goal_index < len(self.tag_positions):
                next_x, next_y, next_name, next_id = self.tag_positions[self.current_goal_index]
                self.get_logger().info(f'Next target: {next_name}')
                self.plan_to_current_goal()
            return
        
        # Motor control (only if enabled)
        if self.enable_motor_control:
            self.do_motor_control(goal_x, goal_y, dist_to_goal)
    
    def do_motor_control(self, goal_x, goal_y, dist_to_goal):
        """Send motor commands (only when enable_motor_control is True)."""
        if not self.current_path:
            self.stop_robot()
            return
        
        # Get current waypoint
        if self.current_waypoint_index >= len(self.current_path):
            self.plan_to_current_goal()
            return
        
        wp_x, wp_y = self.current_path[self.current_waypoint_index]
        
        # Check if reached waypoint
        dist_to_wp = math.sqrt((wp_x - self.robot_x)**2 + (wp_y - self.robot_y)**2)
        if dist_to_wp < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            return
        
        # Calculate control
        angle_to_wp = math.atan2(wp_y - self.robot_y, wp_x - self.robot_x)
        heading_error = self.normalize_angle(angle_to_wp - self.robot_yaw)
        
        cmd = Twist()
        
        if abs(heading_error) > 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_angular_speed,
                               min(self.max_angular_speed, heading_error * 1.0))
        else:
            cmd.linear.x = min(self.max_linear_speed, dist_to_wp * 0.5)
            cmd.angular.z = max(-self.max_angular_speed,
                               min(self.max_angular_speed, heading_error * 1.5))
        
        self.cmd_vel_pub.publish(cmd)
    
    def publish_path_visualization(self):
        """Publish path as visualization markers."""
        marker_array = MarkerArray()
        
        # Clear old markers if no path
        if not self.current_path:
            clear_marker = Marker()
            clear_marker.header.frame_id = 'map'
            clear_marker.header.stamp = self.get_clock().now().to_msg()
            clear_marker.ns = 'path'
            clear_marker.id = 0
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)
            self.path_pub.publish(marker_array)
            return
        
        # Path line
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.lifetime.sec = 0
        
        # Add robot position as start
        p_start = Point()
        p_start.x = self.robot_x
        p_start.y = self.robot_y
        p_start.z = 0.05
        line_marker.points.append(p_start)
        
        # Add path waypoints
        for wp_x, wp_y in self.current_path:
            p = Point()
            p.x = wp_x
            p.y = wp_y
            p.z = 0.05
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # Waypoint spheres
        for i, (wp_x, wp_y) in enumerate(self.current_path):
            wp_marker = Marker()
            wp_marker.header.frame_id = 'map'
            wp_marker.header.stamp = self.get_clock().now().to_msg()
            wp_marker.ns = 'waypoints'
            wp_marker.id = i + 1
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = wp_x
            wp_marker.pose.position.y = wp_y
            wp_marker.pose.position.z = 0.05
            wp_marker.pose.orientation.w = 1.0
            wp_marker.scale.x = 0.08
            wp_marker.scale.y = 0.08
            wp_marker.scale.z = 0.08
            wp_marker.color.r = 0.0
            wp_marker.color.g = 0.8
            wp_marker.color.b = 0.0
            wp_marker.color.a = 1.0
            wp_marker.lifetime.sec = 0
            marker_array.markers.append(wp_marker)
        
        self.path_pub.publish(marker_array)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()