import heapq
import time

# Manhattan Distance
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Parse posisi
def find_points(grid):
    start, goal = None, None
    for i, row in enumerate(grid):
        for j, cell in enumerate(row):
            if cell == 'R':
                start = (i, j)
            elif cell == 'C':
                goal = (i, j)
    return start, goal

# Validasi lokasi
def is_valid(grid, pos):
    i, j = pos
    return 0 <= i < len(grid) and 0 <= j < len(grid[0]) and grid[i][j] != '#'

# Cost berdasarkan jenis jalan
def move_cost(grid, pos):
    cell = grid[pos[0]][pos[1]]
    if cell == 'T':
        return 5  # traffic lebih mahal
    return 1  # normal road

# A* implementation
def a_star(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    explored = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        explored += 1

        if current == goal:
            return reconstruct_path(came_from, current), explored

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if is_valid(grid, neighbor):
                tentative_g = g_score[current] + move_cost(grid, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + manhattan(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

    return None, explored

# Greedy Best First Search (GBFS)
def gbfs(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (manhattan(start, goal), start))
    came_from = {}
    visited = set()
    explored = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        explored += 1

        if current == goal:
            return reconstruct_path(came_from, current), explored

        visited.add(current)

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if is_valid(grid, neighbor) and neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(open_set, (manhattan(neighbor, goal), neighbor))
                visited.add(neighbor)

    return None, explored

# Bangun ulang path
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Visualisasi grid
def visualize(grid, path):
    grid_copy = [list(row) for row in grid]
    for x, y in path:
        if grid_copy[x][y] not in ('R', 'C'):
            grid_copy[x][y] = '*'
    for row in grid_copy:
        print("".join(row))

# Perbandingan
def compare(grid):
    start, goal = find_points(grid)

    print(" Running A*...")
    t0 = time.time()
    path_a, nodes_a = a_star(grid, start, goal)
    t1 = time.time()

    print("\n Running GBFS...")
    t2 = time.time()
    path_g, nodes_g = gbfs(grid, start, goal)
    t3 = time.time()

    print("\n A* Path:")
    if path_a:
        visualize(grid, path_a)
        print(f"Nodes Explored: {nodes_a}")
        print(f"Time: {(t1 - t0) * 1000:.3f} ms")
    else:
        print("No path found.")

    print("\n GBFS Path:")
    if path_g:
        visualize(grid, path_g)
        print(f"Nodes Explored: {nodes_g}")
        print(f"Time: {(t3 - t2) * 1000:.3f} ms")
    else:
        print("No path found.")

# Contoh grid
grid = [
    ['R', '.', '.', '.', '#', '.', '.', 'C'],
    ['.', 'T', '#', '.', 'T', '.', '#', '.'],
    ['.', '.', '.', '.', '.', '.', '.', '.'],
    ['#', '.', '#', '.', '#', 'T', '.', '.']
]

# Run
if __name__ == "__main__":
    compare(grid)
