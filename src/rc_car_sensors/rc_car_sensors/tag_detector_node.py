#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import subprocess
import os


class TagDetectorNode(Node):
    def __init__(self):
        super().__init__('tag_detector_node')
        
        # Parameters
        self.declare_parameter('sounds_directory', '/home/bench207/ros2_ws/sounds')
        self.sounds_dir = self.get_parameter('sounds_directory').value
        
        # Map tag IDs to sound files and display names
        self.tag_config = {
            0: {'sound': 'sound_0.mp3', 'name': 'Start', 'color': (0.0, 1.0, 0.0)},
            1: {'sound': 'sound_1.mp3', 'name': 'Checkpoint 1', 'color': (0.0, 0.0, 1.0)},
            2: {'sound': 'sound_2.mp3', 'name': 'Checkpoint 2', 'color': (1.0, 1.0, 0.0)},
            3: {'sound': 'sound_3.mp3', 'name': 'Treasure', 'color': (1.0, 0.5, 0.0)},
            4: {'sound': 'sound_4.mp3', 'name': 'Finish', 'color': (1.0, 0.0, 0.0)},
        }
        
        # Track detected tag positions in map frame
        self.detected_tags = {}
        
        # Cooldown for sounds
        self.last_played = {}
        self.cooldown_seconds = 3.0
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/tag_markers', 10)
        self.tag_poses_pub = self.create_publisher(PoseStamped, '/detected_tag_pose', 10)
        
        # Timer to publish markers
        self.create_timer(0.5, self.publish_markers)
        
        self.get_logger().info('Tag detector node started')
        self.get_logger().info(f'Sounds directory: {self.sounds_dir}')
    
    def detection_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        for detection in msg.detections:
            tag_id = detection.id
            
            # Get tag pose in camera frame
            pose_camera = PoseStamped()
            pose_camera.header = msg.header
            pose_camera.pose = detection.pose.pose.pose
            
            # Transform to map frame
            pose_map = self.transform_to_map(pose_camera)
            
            if pose_map is not None:
                self.detected_tags[tag_id] = pose_map
                self.tag_poses_pub.publish(pose_map)
                self.get_logger().info(
                    f'Tag {tag_id} at map position: '
                    f'x={pose_map.pose.position.x:.2f}, '
                    f'y={pose_map.pose.position.y:.2f}'
                )
            
            # Check cooldown for sound
            if tag_id in self.last_played:
                if current_time - self.last_played[tag_id] < self.cooldown_seconds:
                    continue
            
            # Play sound
            self.play_sound_for_tag(tag_id)
            self.last_played[tag_id] = current_time
    
    def transform_to_map(self, pose_camera):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                pose_camera.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose_camera, transform)
            pose_map.header.frame_id = 'map'
            return pose_map
        except Exception as e:
            self.get_logger().warn(f'Could not transform to map frame: {e}')
            return None
    
    def publish_markers(self):
        marker_array = MarkerArray()
        
        for tag_id, pose in self.detected_tags.items():
            config = self.tag_config.get(tag_id, {
                'name': f'Tag {tag_id}',
                'color': (0.5, 0.5, 0.5)
            })
            
            # Cylinder marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'apriltags'
            marker.id = tag_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.3
            marker.color.r = config['color'][0]
            marker.color.g = config['color'][1]
            marker.color.b = config['color'][2]
            marker.color.a = 0.8
            marker.lifetime.sec = 0
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'apriltag_labels'
            text_marker.id = tag_id + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = pose.pose
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = config['name']
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def play_sound_for_tag(self, tag_id):
        config = self.tag_config.get(tag_id)
        
        if config and 'sound' in config:
            sound_file = os.path.join(self.sounds_dir, config['sound'])
            if os.path.exists(sound_file):
                try:
                    # Use mpg123 to play sound (non-blocking)
                    subprocess.Popen(['mpg123', '-q', sound_file])
                    self.get_logger().info(f'Playing sound for {config["name"]}')
                except Exception as e:
                    self.get_logger().error(f'Error playing sound: {e}')
            else:
                self.get_logger().warn(f'Sound file not found: {sound_file}')
        else:
            self.get_logger().info(f'No config for tag ID: {tag_id}')


def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()