import numpy as np

def successive_shortest_path(graph, capacity, cost, demand, source, sink, desired_flow):
    # Initialize flow
    flow = np.zeros_like(capacity)

    while True:
        # Find the shortest path with maximum flow using label-correcting algorithm
        path, path_flow = find_shortest_path(graph, capacity, cost, demand, flow, source, sink)

        # If no path found, break
        if path is None:
            break

        # Augment flow along the found path
        augment_flow(flow, path, path_flow)

        # Check if desired flow is reached
        if np.sum(flow) >= desired_flow:
            break

    return flow

def find_shortest_path(graph, capacity, cost, demand, flow, source, sink):
    # Implement label-correcting algorithm to find the shortest path
    # Return the path and the minimum flow along the path

    visited = set()
    queue = [[source]]

    while queue:
        path = queue.pop(0)
        current_node = path[-1]

        if current_node == sink:
            return path, min_capacity(path, capacity)

        for neighbor in range(len(graph[current_node])):
            if graph[current_node][neighbor] == 1 and neighbor not in visited:
                residual_capacity = capacity[current_node][neighbor] - flow[current_node][neighbor]

                if residual_capacity > 0:
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append(new_path)

                    visited.add(neighbor)

    return None, None

def augment_flow(flow, path, path_flow):
    # Update the flow matrix along the given path
    # Subtract path_flow from forward edges and add to backward edges
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        flow[u][v] += path_flow
        flow[v][u] -= path_flow

def min_capacity(path, capacity):
    # Find the minimum capacity along the given path
    min_cap = float('inf')

    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        min_cap = min(min_cap, capacity[u][v])

    return min_cap

# Define the grid network
nodes = 10
graph = np.zeros((nodes * nodes, nodes * nodes), dtype=int)

# Create the grid connections
for i in range(nodes):
    for j in range(nodes):
        node_num = i * nodes + j

        # Connect to the right neighbor
        if j < nodes - 1:
            graph[node_num][node_num + 1] = 1

        # Connect to the bottom neighbor
        if i < nodes - 1:
            graph[node_num][node_num + nodes] = 1

# Set capacities for the edges (you can modify this based on your specific network)
capacity = np.zeros((nodes * nodes, nodes * nodes), dtype=int)

for i in range(nodes):
    for j in range(nodes):
        node_num = i * nodes + j

        # Set capacity to the right neighbor
        if j < nodes - 1:
            capacity[node_num][node_num + 1] = 5

        # Set capacity to the bottom neighbor
        if i < nodes - 1:
            capacity[node_num][node_num + nodes] = 5

# Set costs for the edges (you can modify this based on your specific network)
cost = np.zeros((nodes * nodes, nodes * nodes), dtype=int)

for i in range(nodes):
    for j in range(nodes):
        node_num = i * nodes + j

        # Set cost to the right neighbor
        if j < nodes - 1:
            cost[node_num][node_num + 1] = 2

        # Set cost to the bottom neighbor
        if i < nodes - 1:
            cost[node_num][node_num + nodes] = 2

demand = np.zeros(nodes * nodes, dtype=int)

source = 0
sink = nodes * nodes - 1
desired_flow = 50

result_flow = successive_shortest_path(graph, capacity, cost, demand, source, sink, desired_flow)

# Visualize the optimal flow
print("Optimal Flow example:")
for i in range(nodes * nodes):
    for j in range(nodes * nodes):
        if result_flow[i][j] > 0:
            print(f"{i + 1} → {j + 1}: {result_flow[i][j]} units")
