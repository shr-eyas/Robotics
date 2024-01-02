import numpy as np
import matplotlib.pyplot as plt

def grassfire(grid):

    distance_map = np.full_like(grid, fill_value=20000)
    queue = []

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] == 1:
                distance_map[i, j] = 0
                queue.append((i,j))

   
    while queue:
        ci, cj = queue.pop(0)

        for (ni, nj) in [(ci+1, cj), (ci-1, cj), (ci, cj+1), (ci, cj-1)]:
            if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1] and distance_map[ni, nj] == 20000:
                distance_map[ni, nj] = distance_map[ci, cj] + 1
                queue.append((ni,nj))
    
    return distance_map


# Example usage:
if __name__ == "__main__":
    grid = np.array([
        [1, 1],
        [0, 0]
    ])

    distance_map = grassfire(grid)

    print(distance_map)

