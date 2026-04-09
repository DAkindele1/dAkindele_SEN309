# ==========================
# Drone Delivery Navigation
# ==========================

import heapq
from collections import deque

# --------- Graph Definition ---------
graph = {
    'A': {'B': 2, 'C': 5, 'D': 1},
    'B': {'A': 2, 'D': 2, 'E': 3},
    'C': {'A': 5, 'D': 2, 'F': 3},
    'D': {'A': 1, 'B': 2, 'C': 2, 'E': 1, 'F': 4},
    'E': {'B': 3, 'D': 1, 'G': 2},
    'F': {'C': 3, 'D': 4, 'G': 1},
    'G': {'E': 2, 'F': 1, 'H': 3},
    'H': {'G': 3}
}

heuristic = {
    'A': 7,
    'B': 6,
    'C': 6,
    'D': 4,
    'E': 2,
    'F': 2,
    'G': 1,
    'H': 0
}

start = 'A'
goal = 'H'

# ===================================
# Depth-First Search (DFS)
# ===================================
def dfs(graph, start, goal):
    visited = set()
    path = []

    def dfs_helper(node):
        visited.add(node)
        path.append(node)

        if node == goal:
            return True

        for neighbor in graph[node]:
            if neighbor not in visited:
                if dfs_helper(neighbor):
                    return True

        path.pop()
        return False

    dfs_helper(start)
    return path

# ===================================
# Breadth-First Search (BFS)
# ===================================
def bfs(graph, start, goal):
    visited = set()
    queue = deque()
    queue.append((start, [start]))

    visited.add(start)

    while queue:
        node, path = queue.popleft()

        if node == goal:
            return path

        for neighbor in graph[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor]))

    return []

# ===================================
# Uniform Cost Search (UCS)
# ===================================
def ucs(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start, [start]))
    explored = set()

    while frontier:
        cost, node, path = heapq.heappop(frontier)

        if node == goal:
            return path

        if node in explored:
            continue

        explored.add(node)

        for neighbor, edge_cost in graph[node].items():
            if neighbor not in explored:
                heapq.heappush(frontier, (cost + edge_cost, neighbor, path + [neighbor]))

    return []

# ===================================
# A* Search
# ===================================
def a_star(graph, start, goal, heuristic):
    frontier = []
    heapq.heappush(frontier, (heuristic[start], 0, start, [start]))
    explored = set()

    while frontier:
        f_cost, g_cost, node, path = heapq.heappop(frontier)

        if node == goal:
            return path

        if node in explored:
            continue

        explored.add(node)

        for neighbor, edge_cost in graph[node].items():
            if neighbor not in explored:
                new_g_cost = g_cost + edge_cost
                new_f_cost = new_g_cost + heuristic[neighbor]
                heapq.heappush(frontier, (new_f_cost, new_g_cost, neighbor, path + [neighbor]))

    return []

# ===================================
# Run and Compare
# ===================================
if __name__ == "__main__":
    print("DFS Path:", dfs(graph, start, goal))
    print("BFS Path:", bfs(graph, start, goal))
    print("UCS Path:", ucs(graph, start, goal))
    print("A* Path:", a_star(graph, start, goal, heuristic))