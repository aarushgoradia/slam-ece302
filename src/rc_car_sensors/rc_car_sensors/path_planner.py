#!/usr/bin/env python3
"""
A* Path Planner for occupancy grid maps.
"""
import numpy as np
import heapq
import cv2
import yaml


class PathPlanner:
    def __init__(self, map_yaml_path, inflation_radius=0.2):
        """
        Load map from yaml file.
        
        Args:
            map_yaml_path: Path to map YAML file
            inflation_radius: Robot safety margin in meters (default 0.2m = 20cm)
        """
        with open(map_yaml_path, 'r') as f:
            map_config = yaml.safe_load(f)
        
        # Load map image
        map_dir = '/'.join(map_yaml_path.split('/')[:-1])
        map_image_path = f"{map_dir}/{map_config['image']}"
        self.map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        
        if self.map_image is None:
            raise FileNotFoundError(f"Could not load map image: {map_image_path}")
        
        # Flip back to ROS convention (origin at bottom-left)
        self.map_image = cv2.flip(self.map_image, 0)
        
        self.resolution = map_config['resolution']  # meters per pixel
        self.origin_x = map_config['origin'][0]
        self.origin_y = map_config['origin'][1]
        
        self.height, self.width = self.map_image.shape
        
        # Create binary occupancy grid
        # Occupied if pixel value < 250 (black = obstacle, gray = unknown)
        self.occupancy = np.zeros_like(self.map_image, dtype=np.uint8)
        self.occupancy[self.map_image < 250] = 1  # 1 = obstacle
        self.occupancy[self.map_image >= 250] = 0  # 0 = free
        
        # Convert inflation radius from meters to pixels
        inflation_pixels = int(inflation_radius / self.resolution)
        inflation_pixels = max(1, inflation_pixels)  # At least 1 pixel
        
        print(f"Map resolution: {self.resolution}m/pixel")
        print(f"Inflation: {inflation_radius}m = {inflation_pixels} pixels")
        
        # Inflate obstacles for robot safety margin
        self.inflate_obstacles(inflation_radius=inflation_pixels)
    
    def inflate_obstacles(self, inflation_radius):
        """Expand obstacles by a safety margin."""
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (2 * inflation_radius + 1, 2 * inflation_radius + 1)
        )
        self.occupancy = cv2.dilate(self.occupancy, kernel)
    
    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates (meters) to grid coordinates (pixels)."""
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates (pixels) to world coordinates (meters)."""
        world_x = grid_x * self.resolution + self.origin_x
        world_y = grid_y * self.resolution + self.origin_y
        return world_x, world_y
    
    def is_valid(self, x, y):
        """Check if a grid cell is valid (in bounds and not occupied)."""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        return self.occupancy[y, x] == 0
    
    def heuristic(self, a, b):
        """Euclidean distance heuristic for A*."""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, x, y):
        """Get valid 8-connected neighbors."""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.is_valid(nx, ny):
                    # Diagonal movement costs more
                    cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                    neighbors.append((nx, ny, cost))
        return neighbors
    
    def plan(self, start_world, goal_world):
        """
        Plan a path from start to goal using A*.
        
        Args:
            start_world: (x, y) in meters
            goal_world: (x, y) in meters
        
        Returns:
            List of (x, y) waypoints in meters, or None if no path found
        """
        start = self.world_to_grid(start_world[0], start_world[1])
        goal = self.world_to_grid(goal_world[0], goal_world[1])
        
        # Check if start and goal are valid
        if not self.is_valid(start[0], start[1]):
            print(f"Start position {start} is invalid (occupied or out of bounds)")
            start = self.find_nearest_valid(start)
            if start is None:
                return None
            print(f"Using nearest valid start: {start}")
        
        if not self.is_valid(goal[0], goal[1]):
            print(f"Goal position {goal} is invalid (occupied or out of bounds)")
            goal = self.find_nearest_valid(goal)
            if goal is None:
                return None
            print(f"Using nearest valid goal: {goal}")
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        visited = set()
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            visited.add(current)
            
            # Goal reached
            if current == goal:
                path = self.reconstruct_path(came_from, current)
                return self.simplify_path(path)
            
            for nx, ny, cost in self.get_neighbors(current[0], current[1]):
                neighbor = (nx, ny)
                
                if neighbor in visited:
                    continue
                
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        print("No path found!")
        return None
    
    def find_nearest_valid(self, pos, max_search=20):
        """Find the nearest valid cell to a position."""
        for radius in range(1, max_search):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:
                        nx, ny = pos[0] + dx, pos[1] + dy
                        if self.is_valid(nx, ny):
                            return (nx, ny)
        return None
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from A* result."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        
        # Convert to world coordinates
        world_path = [self.grid_to_world(p[0], p[1]) for p in path]
        return world_path
    
    def simplify_path(self, path, tolerance=0.1):
        """
        Simplify path by removing unnecessary waypoints.
        Uses Ramer-Douglas-Peucker algorithm.
        """
        if len(path) <= 2:
            return path
        
        points = np.array(path)
        
        start = points[0]
        end = points[-1]
        
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)
        
        if line_len == 0:
            return [tuple(start), tuple(end)]
        
        line_unit = line_vec / line_len
        
        max_dist = 0
        max_idx = 0
        
        for i in range(1, len(points) - 1):
            point_vec = points[i] - start
            proj_len = np.dot(point_vec, line_unit)
            proj_point = start + proj_len * line_unit
            dist = np.linalg.norm(points[i] - proj_point)
            
            if dist > max_dist:
                max_dist = dist
                max_idx = i
        
        if max_dist > tolerance:
            left = self.simplify_path(path[:max_idx + 1], tolerance)
            right = self.simplify_path(path[max_idx:], tolerance)
            return left[:-1] + right
        else:
            return [tuple(start), tuple(end)]