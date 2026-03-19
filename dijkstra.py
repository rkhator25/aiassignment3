import heapq

# Graph of cities (example — you can expand this)
graph = {
    "Delhi": {"Jaipur": 280, "Lucknow": 550, "Chandigarh": 250},
    "Jaipur": {"Delhi": 280, "Udaipur": 400, "Ahmedabad": 670},
    "Lucknow": {"Delhi": 550, "Varanasi": 320},
    "Chandigarh": {"Delhi": 250, "Shimla": 120},
    "Udaipur": {"Jaipur": 400, "Ahmedabad": 260},
    "Ahmedabad": {"Udaipur": 260, "Mumbai": 530},
    "Mumbai": {"Ahmedabad": 530},
    "Varanasi": {"Lucknow": 320},
    "Shimla": {"Chandigarh": 120}
}

def dijkstra(graph, start):
    pq = []
    heapq.heappush(pq, (0, start))
    
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    
    while pq:
        current_dist, current_node = heapq.heappop(pq)
        
        for neighbor, weight in graph[current_node].items():
            distance = current_dist + weight
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))
    
    return distances

# Run
start_city = "Delhi"
result = dijkstra(graph, start_city)

print("Shortest distances from", start_city)
for city, dist in result.items():
    print(city, ":", dist, "km")
