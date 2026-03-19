import heapq
import random

GRID_SIZE = 70

def generate_grid(density):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if random.random() < density:
                grid[i][j] = 1  
    
    return grid

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    heapq.heappush(open_list, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    
    directions = [(1,0), (-1,0), (0,1), (0,-1)]
    
    while open_list:
        _, current = heapq.heappop(open_list)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for d in directions:
            neighbor = (current[0] + d[0], current[1] + d[1])
            
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
                
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    came_from[neighbor] = current
    
    return None

# Run simulation
density_levels = [0.1, 0.2, 0.3]  # low, medium, high
grid = generate_grid(density_levels[1])

start = (0, 0)
goal = (69, 69)

path = astar(grid, start, goal)

if path:
    print("Path found! Length:", len(path))
else:
    print("No path found")

if path:
    path_length = len(path)
    nodes_expanded = len(path)  # approx
    
    print("Measures of Effectiveness:")
    print("Path Length:", path_length)
    print("Nodes Expanded:", nodes_expanded)
    print("Obstacle Density:", density_levels[1])
