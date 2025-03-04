import numpy as np
import heapq

class LandingDecisionPipeline:

    def __init__(self, landing_threshold=0.7):
        self.landing_threshold = landing_threshold

    def process_image(self, image):
        
        # Taking in a grayscale image and assuming that a higher mean brightness implies a safer landing zone
        # Can later add in a deep learning approach for more accurate landing decision

        safe_area_score = np.mean(image) / 255.0
        return safe_area_score

    def process_sensor_data(self, sensor_data, wind_weight, obstacle_weight):

        # Setting default values
      
        wind_speed = sensor_data.get("wind_speed", 0)
        altitude = sensor_data.get("altitude", 100)  
        obstacle_distance = sensor_data.get("obstacle_distance", 100)  

        wind_score = max(0, 1 - wind_speed / 10)
        obstacle_score = min(obstacle_distance / 50, 1)
        sensor_score = wind_weight * wind_score + obstacle_weight * obstacle_score

        return sensor_score

    def decide_landing(self, image, sensor_data):
  
        image_score = self.process_image(image)

        # Taking in sample weight scores
        # Would have to do cross validation later to figure out the most appropriate weights

        sensor_score = self.process_sensor_data(sensor_data, wind_weight = 0.2, obstacle_weight = 0.3)
        overall_score = 0.6 * image_score + 0.4 * sensor_score

        # Need to test out the landing threshold values 
        # Can do a detailed analysis with different values later on
        # Overall score can also be made much more accurate with the correct weights
        print(f"Image score: {image_score:.2f}, Sensor score: {sensor_score:.2f}, Overall score: {overall_score:.2f}")
        return overall_score >= self.landing_threshold


class PathPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        neighbors = []
        for i in directions:
            neighbor = (node[0] + i[0], node[1] + i[1])
            if 0 <= neighbor[0] < self.rows and 0 <= neighbor[1] < self.cols:
                if self.grid[neighbor[0]][neighbor[1]] == 0: 
                    neighbors.append(neighbor)
        return neighbors

    def a_star_search(self, start, goal):

        # Using a heapq approach to implement A* algorithm 

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            current_priority, current = heapq.heappop(open_set)

            if current == goal:
                break

            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1  
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

        return self.reconstruct_path(came_from, start, goal)

    def reconstruct_path(self, came_from, start, goal):
    
        # This function would only make sense with real test images from the RealSense Camera
        if goal not in came_from:
            return [] 

        current = goal
        path = [current]
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()

        # Overall implementation with Cross validation can only be done with test data 
      
        return path
