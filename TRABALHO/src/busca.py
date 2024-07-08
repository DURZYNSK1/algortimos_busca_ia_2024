import heapq
from collections import deque

# DFS
def dfs(graph, start, goal):
    visita = set()
    stack = [(start, [start])]
    nodes_analyzed = 0

    while stack:
        current, path = stack.pop()
        nodes_analyzed += 1

        if current == goal:
            path_length = sum(graph[path[i]][path[i + 1]] for i in range(len(path) - 1))
            return nodes_analyzed, path_length, path

        if current not in visita:
            visita.add(current)
            for neighbor in graph[current]:
                if neighbor not in visita:
                    stack.append((neighbor, path + [neighbor]))

    return nodes_analyzed, float('inf'), []

# BFS
def bfs(graph, start, goal):
    visited = set()
    queue = deque([(start, [start])])
    nodes_analyzed = 0

    while queue:
        current, path = queue.popleft()
        nodes_analyzed += 1

        if current == goal:
            path_length = sum(graph[path[i]][path[i + 1]] for i in range(len(path) - 1))
            return nodes_analyzed, path_length, path

        if current not in visited:
            visited.add(current)
            for neighbor in graph[current]:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return nodes_analyzed, float('inf'), []

# Branch and Bound
def branch_and_bound(graph, start, goal):
    frontier = [(0, start, [start])]
    nodes_analyzed = 0

    while frontier:
        cost, current, path = heapq.heappop(frontier)
        nodes_analyzed += 1

        if current == goal:
            return nodes_analyzed, cost, path

        for neighbor, weight in graph[current].items():
            if neighbor not in path:
                heapq.heappush(frontier, (cost + weight, neighbor, path + [neighbor]))

    return nodes_analyzed, float('inf'), []

# A*
def heuristic(a, b):
    return abs(a - b)

def a_star(graph, start, goal):
    frontier = [(0, start, [start])]
    nodes_analyzed = 0
    cost_so_far = {start: 0}

    while frontier:
        _, current, path = heapq.heappop(frontier)
        nodes_analyzed += 1

        if current == goal:
            return nodes_analyzed, cost_so_far[current], path

        for neighbor, weight in graph[current].items():
            new_cost = cost_so_far[current] + weight
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(frontier, (priority, neighbor, path + [neighbor]))

    return nodes_analyzed, float('inf'), []

# Dijkstra
def dijkstra(graph, start, goal):
    frontier = [(0, start, [start])]
    nodes_analyzed = 0
    cost_so_far = {start: 0}

    while frontier:
        current_cost, current, path = heapq.heappop(frontier)
        nodes_analyzed += 1

        if current == goal:
            return nodes_analyzed, current_cost, path

        for neighbor, weight in graph[current].items():
            new_cost = current_cost + weight
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                heapq.heappush(frontier, (new_cost, neighbor, path + [neighbor]))

    return nodes_analyzed, float('inf'), []

# Leitura do grafo
def read_graph(filename):
    graph = {}
    with open(filename, 'r') as file:
        nodes = int(file.readline().strip())
        edges = int(file.readline().strip())

        for _ in range(edges):
            node_from, node_to, weight = map(int, file.readline().strip().split())
            if node_from not in graph:
                graph[node_from] = {}
            if node_to not in graph:
                graph[node_to] = {}
            graph[node_from][node_to] = weight
            graph[node_to][node_from] = weight

    return graph

