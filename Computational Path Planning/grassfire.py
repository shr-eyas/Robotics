import numpy as np
import matplotlib.pyplot as plt

def grassfire(grid):

    UNEXPLORED = 2000

    distance_map = np.copy(grid)
    queue = []

    sx, sy = np.where(grid == 2) 
    ex, ey = np.where(grid == 3) 
 
    distance_map[sx, sy] = 0
    distance_map[ex, ey] = UNEXPLORED 

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if (i, j) != (sx, sy) and (i, j) != (ex, ey):
                if grid[i, j] == 0:
                    distance_map[i, j] = UNEXPLORED
                    
    for ni, nj in [(sx+1, sy), (sx-1, sy), (sx, sy+1), (sx, sy-1)]:
        if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1] and distance_map[ni, nj] == UNEXPLORED:
            distance_map[ni, nj] = distance_map[sx, sy] + 1
            queue.append((ni,nj))
    
    while queue:
        ci, cj = queue.pop(0)
        for ni, nj in [(ci+1, cj), (ci-1, cj), (ci, cj+1), (ci, cj-1)]:
            if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1] and distance_map[ni, nj] == UNEXPLORED:
                distance_map[ni, nj] = distance_map[ci, cj] + 1
                queue.append((ni,nj))
                
    return distance_map


if __name__ == "__main__":

    # 0 is empty cell
    # -1 is obstacle
    # 2 is start point
    # 3 is end point

    grid = np.array([
        [-1, -1, 0, 2],
        [-1, -1, 0, -1], 
        [-1, 0, 0, -1],
        [3, 0, -1, -1] 
    ])

    # grid in the course
    # grid = np.array([
    #     [0, 3, 0, 0, -1, -1],
    #     [0, 0, 0, 0, 0, -1], 
    #     [0, -1, -1, 0, 0, 0],
    #     [0, 0, -1, -1, -1, 0],
    #     [0, 0, -1, 0, 0, 0],
    #     [-1, 0, 0, 0, 2, 0],
    #     [-1, -1, 0, 0, 0, 0] 
    # ])

    distance_map = grassfire(grid)
    print(distance_map)
