import matplotlib.pyplot as plt
import random
import math
import numpy as np

def generate_rrt_star_diagram(num_nodes=30, rewire_radius=3.0):
    random.seed(42)
    
    start = (1, 1)
    goal = (9, 9)
    goal_radius = 0.5  # Consider the goal reached if within this radius

    nodes = [start]
    edges = {}  # Dictionary: child -> parent
    costs = {start: 0}  
    
    plt.figure(figsize=(10, 8))
    
    for i in range(num_nodes):
        
        if random.random() < 0.05:  
            x_rand, y_rand = goal
        else:
            x_rand = random.uniform(0, 10)
            y_rand = random.uniform(0, 10)
        rand_point = (x_rand, y_rand)
        
        # 2. Find nearest existing node
        nearest_node = min(nodes, key=lambda n: distance(n, rand_point))
        
       
        step_size = 1.0
        direction = (x_rand - nearest_node[0], y_rand - nearest_node[1])
        dist = math.sqrt(direction[0]**2 + direction[1]**2)
        
        if dist > step_size:
            # Normalize and scale by step size
            direction = (direction[0]/dist*step_size, direction[1]/dist*step_size)
        
        new_node = (nearest_node[0] + direction[0], nearest_node[1] + direction[1])
        
        # 4. Find nearby nodes for rewiring consideration
        nearby_nodes = [n for n in nodes if distance(n, new_node) <= rewire_radius]
        
        # 5. Choose parent that results in lowest cost path from start
        best_parent = nearest_node
        best_cost = costs[nearest_node] + distance(nearest_node, new_node)
        
        for node in nearby_nodes:
            potential_cost = costs[node] + distance(node, new_node)
            if potential_cost < best_cost:
                best_cost = potential_cost
                best_parent = node
        
        nodes.append(new_node)
        edges[new_node] = best_parent
        costs[new_node] = best_cost
        
        for node in nearby_nodes:
            if node == best_parent:
                continue
                
            potential_cost = costs[new_node] + distance(new_node, node)
            if potential_cost < costs[node]:
                old_parent = edges.get(node)
                edges[node] = new_node
                costs[node] = potential_cost
    
    nearest_to_goal = min(nodes, key=lambda n: distance(n, goal))
    if distance(nearest_to_goal, goal) <= goal_radius:
        # Goal is reachable
        nodes.append(goal)
        edges[goal] = nearest_to_goal
        costs[goal] = costs[nearest_to_goal] + distance(nearest_to_goal, goal)
        
        path = [goal]
        current = goal
        while current != start:
            current = edges[current]
            path.append(current)
        path.reverse()
    else:
        path = []
    
    for child, parent in edges.items():
        plt.plot([parent[0], child[0]], [parent[1], child[1]], 'gray', linewidth=0.7, alpha=0.5)
    
    if path:
        for i in range(len(path)-1):
            plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], 
                    'g-', linewidth=2.5, alpha=0.8)
    
    for node in nodes:
        if node == start or node == goal:
            continue  
        plt.plot(node[0], node[1], 'bo', markersize=4, alpha=0.7)
        
    plt.plot(start[0], start[1], 'ro', markersize=10)
    plt.plot(goal[0], goal[1], 'go', markersize=10)
    
    example_node = nodes[len(nodes)//2]  
    circle = plt.Circle(example_node, rewire_radius, color='r', fill=False, alpha=0.3, linestyle='--')
    plt.gca().add_patch(circle)
    plt.text(example_node[0], example_node[1]-0.5, f"Rewiring\nRadius", 
             ha='center', va='top', fontsize=9)
    
    plt.text(start[0]+0.2, start[1]+0.2, "Start", fontsize=12)
    plt.text(goal[0]+0.2, goal[1]+0.2, "Goal", fontsize=12)
    
    plt.plot([], [], 'bo', markersize=4, label='Tree Nodes')
    plt.plot([], [], 'gray', linewidth=0.7, label='Tree Connections')
    plt.plot([], [], 'g-', linewidth=2.5, label='Optimal Path')
    plt.plot([], [], 'r--', label='Rewiring Radius')
    
    plt.grid(True, alpha=0.3)
    plt.xlim(-1, 11)
    plt.ylim(-1, 11)
    plt.title("RRT* Algorithm Illustration", fontsize=14)
    plt.legend(loc='lower right')
    
    plt.tight_layout()
    plt.show()

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
