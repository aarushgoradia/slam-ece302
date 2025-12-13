#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cartographer_ros_msgs.msg import LandmarkList, LandmarkEntry
from cv_bridge import CvBridge
import tf2_ros
import subprocess
import numpy as np
import cv2
import apriltag
import os
import math
import json


class CameraApriltagNode(Node):
    def __init__(self):
        super().__init__('camera_apriltag_node')
        
        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('sounds_directory', '/home/bench207/ros2_ws/sounds')
        self.declare_parameter('tag_size', 0.1)
        self.declare_parameter('publish_tag_markers', True)
        self.declare_parameter('save_tags', True)
        self.declare_parameter('tags_file', '/home/bench207/ros2_ws/tag_positions.json')
        self.declare_parameter('publish_landmarks', False)
        self.declare_parameter('known_tags_file', '/home/bench207/ros2_ws/tag_positions.json')
        self.declare_parameter('play_sounds', True)  # NEW: enable/disable sounds
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.sounds_dir = self.get_parameter('sounds_directory').value
        self.tag_size = self.get_parameter('tag_size').value
        self.publish_tag_markers_enabled = self.get_parameter('publish_tag_markers').value
        self.save_tags_enabled = self.get_parameter('save_tags').value
        self.tags_file = self.get_parameter('tags_file').value
        self.publish_landmarks_enabled = self.get_parameter('publish_landmarks').value
        self.known_tags_file = self.get_parameter('known_tags_file').value
        self.play_sounds_enabled = self.get_parameter('play_sounds').value  # NEW
        
        self.bridge = CvBridge()
        
        # AprilTag detector
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
        
        # Tag config
        self.tag_config = {
            0: {'sound': 'sound_0.mp3', 'name': 'Start', 'color': (0.0, 1.0, 0.0)},
            1: {'sound': 'sound_1.mp3', 'name': 'Checkpoint 1', 'color': (0.0, 0.0, 1.0)},
            2: {'sound': 'sound_2.mp3', 'name': 'Checkpoint 2', 'color': (1.0, 1.0, 0.0)},
            3: {'sound': 'sound_3.mp3', 'name': 'Treasure', 'color': (1.0, 0.5, 0.0)},
            4: {'sound': 'sound_4.mp3', 'name': 'Finish', 'color': (1.0, 0.0, 0.0)},
        }
        
        # Known tag positions (for landmark publishing)
        self.known_tags = {}
        if self.publish_landmarks_enabled:
            self.load_known_tags()
        
        # Cooldown for sounds
        self.last_played = {}
        self.cooldown_seconds = 3.0
        
        # Tag observations for averaging
        self.tag_observations = {}
        self.detected_tags = {}
        self.max_observations = 10
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera intrinsics (approximate for Pi Camera v3)
        self.fx = self.width * 0.9
        self.fy = self.width * 0.9
        self.cx = self.width / 2
        self.cy = self.height / 2
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.image_annotated_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/tag_markers', 10)
        
        # Landmark publisher for Cartographer
        self.landmark_pub = self.create_publisher(LandmarkList, '/landmarks', 10)
        
        # Start camera
        self.process = subprocess.Popen([
            'rpicam-vid',
            '-t', '0',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.fps),
            '--codec', 'yuv420',
            '-o', '-'
        ], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        
        self.frame_size = self.width * self.height * 3 // 2
        
        # Timers
        self.create_timer(1.0 / self.fps, self.process_frame)
        
        if self.publish_tag_markers_enabled:
            self.create_timer(0.5, self.publish_markers)
        
        if self.save_tags_enabled:
            self.create_timer(10.0, self.save_tags_to_file)
        
        self.get_logger().info(f'Camera + AprilTag node started: {self.width}x{self.height} @ {self.fps}fps')
        self.get_logger().info(f'Publish tag markers: {self.publish_tag_markers_enabled}')
        self.get_logger().info(f'Save tags: {self.save_tags_enabled}')
        self.get_logger().info(f'Publish landmarks: {self.publish_landmarks_enabled}')
        self.get_logger().info(f'Play sounds: {self.play_sounds_enabled}')  # NEW
    
    def load_known_tags(self):
        """Load known tag positions for landmark publishing."""
        if not os.path.exists(self.known_tags_file):
            self.get_logger().warn(f'Known tags file not found: {self.known_tags_file}')
            return
        
        with open(self.known_tags_file, 'r') as f:
            data = json.load(f)
        
        for tag_id, info in data.items():
            self.known_tags[int(tag_id)] = {
                'x': info['x'],
                'y': info['y'],
                'name': info.get('name', f'Tag {tag_id}')
            }
        
        self.get_logger().info(f'Loaded {len(self.known_tags)} known tag positions for landmarks')
        for tag_id, info in self.known_tags.items():
            self.get_logger().info(f'  Tag {tag_id} ({info["name"]}): ({info["x"]:.2f}, {info["y"]:.2f})')
    
    def estimate_tag_distance(self, corners):
        """Estimate distance to tag based on apparent size."""
        diag1 = np.linalg.norm(corners[0] - corners[2])
        diag2 = np.linalg.norm(corners[1] - corners[3])
        apparent_size = (diag1 + diag2) / 2
        
        if apparent_size < 1:
            return float('inf')
        
        distance = (self.tag_size * self.fx) / apparent_size
        return distance
    
    def estimate_tag_angle(self, center):
        """Estimate horizontal angle to tag from camera center."""
        offset_x = center[0] - self.cx
        angle = math.atan2(offset_x, self.fx)
        return angle
    
    def get_tag_position_in_map(self, distance, angle):
        """Transform tag position from camera frame to map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            q = transform.transform.rotation
            robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            tag_x_robot = distance * math.cos(angle)
            tag_y_robot = distance * math.sin(angle)
            
            tag_x_map = robot_x + tag_x_robot * math.cos(robot_yaw) - tag_y_robot * math.sin(robot_yaw)
            tag_y_map = robot_y + tag_x_robot * math.sin(robot_yaw) + tag_y_robot * math.cos(robot_yaw)
            
            return tag_x_map, tag_y_map
            
        except Exception as e:
            return None, None
    
    def update_tag_position(self, tag_id, x, y):
        """Update tag position using averaging."""
        if tag_id not in self.tag_observations:
            self.tag_observations[tag_id] = []
        
        self.tag_observations[tag_id].append((x, y))
        
        if len(self.tag_observations[tag_id]) > self.max_observations:
            self.tag_observations[tag_id].pop(0)
        
        positions = self.tag_observations[tag_id]
        avg_x = sum(p[0] for p in positions) / len(positions)
        avg_y = sum(p[1] for p in positions) / len(positions)
        
        self.detected_tags[tag_id] = (avg_x, avg_y)
        return avg_x, avg_y
    
    def publish_landmark(self, tag_id, distance, angle):
        """Publish detected tag as Cartographer landmark."""
        if tag_id not in self.known_tags:
            return
        
        landmark_msg = LandmarkList()
        landmark_msg.header.stamp = self.get_clock().now().to_msg()
        landmark_msg.header.frame_id = 'base_link'
        
        entry = LandmarkEntry()
        entry.id = str(tag_id)
        
        entry.tracking_from_landmark_transform.position.x = distance * math.cos(angle)
        entry.tracking_from_landmark_transform.position.y = distance * math.sin(angle)
        entry.tracking_from_landmark_transform.position.z = 0.0
        
        entry.tracking_from_landmark_transform.orientation.w = 1.0
        entry.tracking_from_landmark_transform.orientation.x = 0.0
        entry.tracking_from_landmark_transform.orientation.y = 0.0
        entry.tracking_from_landmark_transform.orientation.z = 0.0
        
        entry.translation_weight = 1e7
        entry.rotation_weight = 0.0
        
        landmark_msg.landmarks.append(entry)
        self.landmark_pub.publish(landmark_msg)
        
        self.get_logger().debug(
            f'Published landmark {tag_id} at distance={distance:.2f}m, angle={math.degrees(angle):.1f}°'
        )
    
    def process_frame(self):
        try:
            raw_data = self.process.stdout.read(self.frame_size)
            if len(raw_data) != self.frame_size:
                return
            
            yuv = np.frombuffer(raw_data, dtype=np.uint8).reshape((self.height * 3 // 2, self.width))
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            
            detections = self.detector.detect(gray)
            
            current_time = self.get_clock().now().nanoseconds / 1e9
            annotated = bgr.copy()
            
            for det in detections:
                tag_id = det.tag_id
                corners = det.corners
                center = det.center
                
                # Draw on image
                corners_int = corners.astype(int)
                cv2.polylines(annotated, [corners_int], True, (0, 255, 0), 2)
                center_int = center.astype(int)
                cv2.circle(annotated, tuple(center_int), 5, (0, 0, 255), -1)
                
                config = self.tag_config.get(tag_id, {'name': f'Tag {tag_id}'})
                cv2.putText(annotated, config['name'], (center_int[0] - 20, center_int[1] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Estimate distance and angle
                distance = self.estimate_tag_distance(corners)
                angle = self.estimate_tag_angle(center)
                
                # Skip if too far or invalid
                if distance > 5.0 or distance <= 0:
                    continue
                
                # Display distance on image
                cv2.putText(annotated, f'{distance:.2f}m', (center_int[0] - 20, center_int[1] + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                # Publish landmark for Cartographer (autonomous mode)
                if self.publish_landmarks_enabled:
                    self.publish_landmark(tag_id, distance, angle)
                
                # Get position in map frame (for mapping mode)
                if self.save_tags_enabled or self.publish_tag_markers_enabled:
                    tag_x, tag_y = self.get_tag_position_in_map(distance, angle)
                    if tag_x is not None:
                        self.update_tag_position(tag_id, tag_x, tag_y)
                
                # Play sound with cooldown - ONLY IF ENABLED
                if self.play_sounds_enabled:
                    if tag_id not in self.last_played or (current_time - self.last_played[tag_id]) > self.cooldown_seconds:
                        self.play_sound_for_tag(tag_id)
                        self.last_played[tag_id] = current_time
            
            # Publish images
            msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.image_pub.publish(msg)
            
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_annotated_pub.publish(annotated_msg)
            
            # Camera info
            info_msg = CameraInfo()
            info_msg.header = msg.header
            info_msg.width = self.width
            info_msg.height = self.height
            info_msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
            self.info_pub.publish(info_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def play_sound_for_tag(self, tag_id):
        config = self.tag_config.get(tag_id)
        if config and 'sound' in config:
            sound_file = os.path.join(self.sounds_dir, config['sound'])
            if os.path.exists(sound_file):
                try:
                    subprocess.Popen(['mpg123', '-q', sound_file])
                except:
                    pass
    
    def publish_markers(self):
        if not self.publish_tag_markers_enabled:
            return
        
        marker_array = MarkerArray()
        
        for tag_id, (x, y) in self.detected_tags.items():
            config = self.tag_config.get(tag_id, {'name': f'Tag {tag_id}', 'color': (0.5, 0.5, 0.5)})
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'apriltags'
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
            marker_array.markers.append(marker)
            
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'apriltag_labels'
            text.id = tag_id + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = config['name']
            marker_array.markers.append(text)
        
        self.marker_pub.publish(marker_array)
    
    def save_tags_to_file(self):
        if not self.save_tags_enabled or not self.detected_tags:
            return
        
        tags_data = {}
        for tag_id, (x, y) in self.detected_tags.items():
            config = self.tag_config.get(tag_id, {'name': f'Tag {tag_id}'})
            tags_data[str(tag_id)] = {
                'x': float(x),
                'y': float(y),
                'name': config['name']
            }
        
        with open(self.tags_file, 'w') as f:
            json.dump(tags_data, f, indent=2)
        
        self.get_logger().info(f'Saved {len(tags_data)} tags to {self.tags_file}')
    
    def destroy_node(self):
        self.process.terminate()
        if self.save_tags_enabled:
            self.save_tags_to_file()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraApriltagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()