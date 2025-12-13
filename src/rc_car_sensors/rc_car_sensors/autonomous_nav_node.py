#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import json
import os
import math
import subprocess

from rc_car_sensors.path_planner import PathPlanner


class AutonomousNavNode(Node):
    def __init__(self):
        super().__init__('autonomous_nav_node')
        
        # Parameters
        self.declare_parameter('tags_file', '/home/bench207/ros2_ws/tag_positions.json')
        self.declare_parameter('map_yaml', '/home/bench207/ros2_ws/maps/my_map.yaml')
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('min_linear_speed', 0.25)
        self.declare_parameter('turn_slowdown_factor', 0.3)
        self.declare_parameter('obstacle_inflation', 0.2)  # NEW: inflate obstacles
        self.declare_parameter('start_delay', 10.0)
        self.declare_parameter('enable_motor_control', False)
        self.declare_parameter('sounds_directory', '/home/bench207/ros2_ws/sounds')
        
        self.tags_file = self.get_parameter('tags_file').value
        self.map_yaml = self.get_parameter('map_yaml').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.turn_slowdown_factor = self.get_parameter('turn_slowdown_factor').value
        self.obstacle_inflation = self.get_parameter('obstacle_inflation').value
        self.start_delay = self.get_parameter('start_delay').value
        self.enable_motor_control = self.get_parameter('enable_motor_control').value
        self.sounds_dir = self.get_parameter('sounds_directory').value
        
        # Tag config
        self.tag_config = {
            0: {'sound': 'sound_0.mp3', 'name': 'Start', 'color': (0.0, 1.0, 0.0)},
            1: {'sound': 'sound_1.mp3', 'name': 'Checkpoint 1', 'color': (0.0, 0.0, 1.0)},
            2: {'sound': 'sound_2.mp3', 'name': 'Checkpoint 2', 'color': (1.0, 1.0, 0.0)},
            3: {'sound': 'sound_3.mp3', 'name': 'Treasure', 'color': (1.0, 0.5, 0.0)},
            4: {'sound': 'sound_4.mp3', 'name': 'Finish', 'color': (1.0, 0.0, 0.0)},
        }
        
        # State
        self.tag_positions = []
        self.current_goal_index = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.active = False
        self.localized = False
        self.auto_started = False
        
        # Track which tags we've played sounds for
        self.played_sounds = set()
        
        # Path
        self.current_path = []
        self.current_waypoint_index = 0
        self.planner = None
        
        # Stuck detection
        self.last_position = (0.0, 0.0)
        self.stuck_counter = 0
        self.last_stuck_check = self.get_clock().now()
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/planned_path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/current_goal', 10)
        self.tag_marker_pub = self.create_publisher(MarkerArray, '/tag_markers', 10)
        
        # Init
        self.load_tags()
        self.load_planner()
        
        # Timers
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.05, self.update_robot_pose)
        self.create_timer(0.5, self.publish_path_visualization)
        self.create_timer(1.0, self.publish_tag_markers)
        
        self.get_logger().info(f'Motor control: {"ENABLED" if self.enable_motor_control else "DISABLED"}')
        self.get_logger().info(f'Goal tolerance: {self.goal_tolerance}m')
        self.get_logger().info(f'Obstacle inflation: {self.obstacle_inflation}m')
        self.get_logger().info(f'Navigation will auto-start in {self.start_delay} seconds...')
        self.auto_start_timer = self.create_timer(self.start_delay, self.auto_start)
    
    def auto_start(self):
        if not self.auto_started:
            self.auto_started = True
            self.auto_start_timer.cancel()
            self.start_navigation()
    
    def load_tags(self):
        self.get_logger().info(f'Loading tags from: {self.tags_file}')
        if not os.path.exists(self.tags_file):
            self.get_logger().error(f'Tags file not found')
            return False
        try:
            with open(self.tags_file, 'r') as f:
                tags_data = json.load(f)
            self.tag_positions = []
            for tag_id, data in sorted(tags_data.items(), key=lambda x: int(x[0])):
                tag_id_int = int(tag_id)
                config = self.tag_config.get(tag_id_int, {'name': f'Tag {tag_id}'})
                self.tag_positions.append((data['x'], data['y'], config['name'], tag_id_int))
            self.get_logger().info(f'Loaded {len(self.tag_positions)} tags')
            return True
        except Exception as e:
            self.get_logger().error(f'Error loading tags: {e}')
            return False
    
    def load_planner(self):
        self.get_logger().info(f'Loading planner from: {self.map_yaml}')
        if not os.path.exists(self.map_yaml):
            self.get_logger().error(f'Map file not found')
            return False
        try:
            self.planner = PathPlanner(self.map_yaml, inflation_radius=self.obstacle_inflation)
            self.get_logger().info(f'Path planner loaded with inflation={self.obstacle_inflation}m')
            return True
        except Exception as e:
            # Try without inflation parameter (backwards compatibility)
            try:
                self.planner = PathPlanner(self.map_yaml)
                self.get_logger().info('Path planner loaded (no inflation support)')
                return True
            except Exception as e2:
                self.get_logger().error(f'Error loading planner: {e2}')
                return False
    
    def publish_tag_markers(self):
        marker_array = MarkerArray()
        for i, (x, y, name, tag_id) in enumerate(self.tag_positions):
            config = self.tag_config.get(tag_id, {'name': name, 'color': (0.5, 0.5, 0.5)})
            
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
            marker.scale.x, marker.scale.y, marker.scale.z = 0.2, 0.2, 0.3
            marker.color.r, marker.color.g, marker.color.b = config['color']
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'tag_labels'
            text.id = tag_id + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.4
            text.scale.z = 0.15
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
            text.text = name
            marker_array.markers.append(text)
            
            if self.active and i == self.current_goal_index:
                ring = Marker()
                ring.header.frame_id = 'map'
                ring.header.stamp = self.get_clock().now().to_msg()
                ring.ns = 'current_goal_ring'
                ring.id = 200
                ring.type = Marker.CYLINDER
                ring.action = Marker.ADD
                ring.pose.position.x, ring.pose.position.y = x, y
                ring.pose.position.z = 0.02
                ring.scale.x, ring.scale.y, ring.scale.z = self.goal_tolerance * 2, self.goal_tolerance * 2, 0.02
                ring.color.r, ring.color.g, ring.color.b, ring.color.a = 1.0, 1.0, 0.0, 0.5
                marker_array.markers.append(ring)
        
        self.tag_marker_pub.publish(marker_array)
    
    def start_navigation(self):
        if not self.tag_positions or not self.planner:
            self.get_logger().error('Cannot start: missing tags or planner')
            return
        self.current_goal_index = 0
        self.active = True
        self.played_sounds = set()
        self.get_logger().info('=' * 50)
        self.get_logger().info('NAVIGATION STARTED')
        self.get_logger().info('=' * 50)
        self.plan_to_current_goal()
    
    def plan_to_current_goal(self):
        if self.current_goal_index >= len(self.tag_positions):
            return False
        
        goal_x, goal_y, goal_name, tag_id = self.tag_positions[self.current_goal_index]
        self.get_logger().info(f'Planning to {goal_name} at ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info(f'Robot at ({self.robot_x:.2f}, {self.robot_y:.2f})')
        
        start = (self.robot_x, self.robot_y)
        goal = (goal_x, goal_y)
        path = self.planner.plan(start, goal)
        
        if path is None or len(path) == 0:
            self.get_logger().error(f'No path found to {goal_name}!')
            self.current_path = [(goal_x, goal_y)]
            self.current_waypoint_index = 0
            return False
        
        # Validate path
        valid_path = []
        max_reasonable_dist = math.sqrt((goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2) * 2 + 1.0
        
        for wp_x, wp_y in path:
            dist_from_robot = math.sqrt((wp_x - self.robot_x)**2 + (wp_y - self.robot_y)**2)
            dist_from_goal = math.sqrt((wp_x - goal_x)**2 + (wp_y - goal_y)**2)
            
            if dist_from_robot < max_reasonable_dist and dist_from_goal < max_reasonable_dist:
                valid_path.append((wp_x, wp_y))
        
        if len(valid_path) == 0:
            self.get_logger().warn('Path validation failed, using direct line')
            valid_path = [(goal_x, goal_y)]
        
        self.current_path = valid_path
        self.current_waypoint_index = 0
        self.get_logger().info(f'Path found with {len(self.current_path)} waypoints')
        return True
    
    def update_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            q = trans.transform.rotation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            if not self.localized:
                self.localized = True
                self.get_logger().info(f'Localized at ({self.robot_x:.2f}, {self.robot_y:.2f})')
        except Exception:
            pass
    
    def play_sound(self, tag_id):
        # Only play sound once per tag
        if tag_id in self.played_sounds:
            return
        
        config = self.tag_config.get(tag_id)
        if config and 'sound' in config:
            sound_file = os.path.join(self.sounds_dir, config['sound'])
            if os.path.exists(sound_file):
                try:
                    subprocess.Popen(['mpg123', '-q', sound_file])
                    self.played_sounds.add(tag_id)
                    self.get_logger().info(f'Playing sound for tag {tag_id}')
                except:
                    pass
    
    def is_waypoint_behind(self, wp_x, wp_y):
        """Check if waypoint is behind the robot."""
        dx = wp_x - self.robot_x
        dy = wp_y - self.robot_y
        
        forward_x = math.cos(self.robot_yaw)
        forward_y = math.sin(self.robot_yaw)
        
        dot = dx * forward_x + dy * forward_y
        return dot < 0
    
    def control_loop(self):
        if not self.active:
            return
        
        if self.current_goal_index >= len(self.tag_positions):
            self.get_logger().info('ALL TAGS VISITED!')
            self.active = False
            self.stop_robot()
            return
        
        goal_x, goal_y, goal_name, tag_id = self.tag_positions[self.current_goal_index]
        
        # Publish current goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x, goal_msg.pose.position.y = goal_x, goal_y
        self.goal_pub.publish(goal_msg)
        
        # Check if reached goal
        dist = math.sqrt((goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2)
        if dist < self.goal_tolerance:
            self.get_logger().info(f'REACHED {goal_name}!')
            self.play_sound(tag_id)
            self.current_goal_index += 1
            self.stuck_counter = 0
            self.stop_robot()
            if self.current_goal_index < len(self.tag_positions):
                self.plan_to_current_goal()
            return
        
        # Stuck detection
        elapsed = (self.get_clock().now() - self.last_stuck_check).nanoseconds / 1e9
        if elapsed > 2.0:
            moved = math.sqrt(
                (self.robot_x - self.last_position[0])**2 + 
                (self.robot_y - self.last_position[1])**2
            )
            
            if moved < 0.1 and self.enable_motor_control:
                self.stuck_counter += 1
                self.get_logger().warn(f'Possibly stuck! Counter: {self.stuck_counter}')
                
                if self.stuck_counter >= 3:
                    self.get_logger().warn('Stuck detected! Replanning...')
                    self.stuck_counter = 0
                    self.plan_to_current_goal()
            else:
                self.stuck_counter = 0
            
            self.last_position = (self.robot_x, self.robot_y)
            self.last_stuck_check = self.get_clock().now()
        
        if self.current_waypoint_index >= len(self.current_path):
            self.get_logger().info('Waypoints exhausted, replanning...')
            self.plan_to_current_goal()
        
        if self.enable_motor_control:
            self.do_motor_control()
    
    def do_motor_control(self):
        if not self.current_path:
            self.stop_robot()
            return
        
        if self.current_waypoint_index >= len(self.current_path):
            self.get_logger().info('Waypoints exhausted, replanning...')
            self.plan_to_current_goal()
            return
        
        # Skip waypoints that are behind us or very close
        while self.current_waypoint_index < len(self.current_path):
            wp_x, wp_y = self.current_path[self.current_waypoint_index]
            dist = math.sqrt((wp_x - self.robot_x)**2 + (wp_y - self.robot_y)**2)
            
            if dist < self.waypoint_tolerance or (self.is_waypoint_behind(wp_x, wp_y) and dist < 1.0):
                self.current_waypoint_index += 1
            else:
                break
        
        if self.current_waypoint_index >= len(self.current_path):
            self.get_logger().info('All waypoints passed, replanning...')
            self.plan_to_current_goal()
            return
        
        wp_x, wp_y = self.current_path[self.current_waypoint_index]
        
        # Calculate heading error
        angle = math.atan2(wp_y - self.robot_y, wp_x - self.robot_x)
        heading_error = self.normalize_angle(angle - self.robot_yaw)
        
        # Skip waypoints requiring >90° turn
        if abs(heading_error) > math.pi / 2:
            self.get_logger().info(f'Waypoint requires >90° turn, skipping...')
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.current_path):
                self.plan_to_current_goal()
            return
        
        cmd = Twist()
        
        # Steering
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, heading_error * 1.5))
        
        # Speed
        turn_severity = min(1.0, abs(heading_error) / 1.0)
        speed_reduction = turn_severity * self.turn_slowdown_factor
        target_speed = self.max_linear_speed * (1.0 - speed_reduction)
        cmd.linear.x = max(self.min_linear_speed, target_speed)
        
        self.cmd_vel_pub.publish(cmd)
    
    def publish_path_visualization(self):
        marker_array = MarkerArray()
        
        if not self.current_path:
            clear = Marker()
            clear.header.frame_id = 'map'
            clear.ns = 'path'
            clear.action = Marker.DELETEALL
            marker_array.markers.append(clear)
            self.path_pub.publish(marker_array)
            return
        
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 1.0, 0.0, 1.0
        
        p = Point()
        p.x, p.y, p.z = self.robot_x, self.robot_y, 0.05
        line.points.append(p)
        
        for wp_x, wp_y in self.current_path[self.current_waypoint_index:]:
            p = Point()
            p.x, p.y, p.z = wp_x, wp_y, 0.05
            line.points.append(p)
        
        marker_array.markers.append(line)
        self.path_pub.publish(marker_array)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
    
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