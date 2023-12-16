import time
import numpy as np
import matplotlib.pyplot as plt
from rrt import RRT  # Your RRT implementation
from a_star import AStarPlanner  # Your A* implementation

def generate_obstacles(grid_max_x, grid_max_y):
    ox, oy = [], []

    # Basic border walls
    for i in range(0, grid_max_x):
        ox.append(i)
        oy.append(0.0)
    for i in range(0, grid_max_y):
        ox.append(grid_max_x - 1)
        oy.append(i)
    for i in range(0, grid_max_x):
        ox.append(i)
        oy.append(grid_max_y - 1)
    for i in range(0, grid_max_y):
        ox.append(0.0)
        oy.append(i)

    # Random obstacles
    num_obstacles = np.random.randint(10, 20)
    for _ in range(num_obstacles):
        shape_type = np.random.choice(['block', 'line'])
        x = np.random.randint(10, grid_max_x - 10)
        y = np.random.randint(10, grid_max_y - 10)

        if shape_type == 'block':
            max_block_size_x = min(grid_max_x - x, 20)
            max_block_size_y = min(grid_max_y - y, 20)
            block_size = np.random.randint(5, min(max_block_size_x, max_block_size_y))
            for bx in range(x, x + block_size):
                for by in range(y, y + block_size):
                    ox.append(bx)
                    oy.append(by)

        elif shape_type == 'line':
            if np.random.rand() > 0.5:  # horizontal line
                length = min(np.random.randint(10, 50), grid_max_x - x)
                for i in range(length):
                    ox.append(x + i)
                    oy.append(y)
            else:  # vertical line
                length = min(np.random.randint(10, 50), grid_max_y - y)
                for i in range(length):
                    ox.append(x)
                    oy.append(y + i)

    return ox, oy

def generate_random_point(min_value, max_value):
    return np.random.randint(min_value, max_value), np.random.randint(min_value, max_value)

def is_distance_sufficient(sx, sy, gx, gy, min_distance):
    return np.hypot(gx - sx, gy - sy) >= min_distance



def main():
    num_environments = 1  # Number of environments to generate
    grid_max_x, grid_max_y = 100, 100  # Size of the grid area
    min_distance = 50.0  # Minimum distance between start and goal
    robot_radius = 1.0  # Robot radius for RRT

    for i in range(num_environments):
        ox, oy = generate_obstacles(grid_max_x, grid_max_y)

        # Generate random start and goal positions with minimum distance
        while True:
            sx, sy = generate_random_point(1, grid_max_x - 1)
            gx, gy = generate_random_point(1, grid_max_y - 1)
            if is_distance_sufficient(sx, sy, gx, gy, min_distance):
                break


        print(f"Start Goal: ({sx}, {sy})")
        print(f"End Goal: ({gx}, {gy})")

        # RRT Planning
        start_time = time.time()
        rrt = RRT(
            start=[sx, sy],
            goal=[gx, gy],
            rand_area=[0, grid_max_x],
            obstacle_list=list(zip(ox, oy, [robot_radius] * len(ox))),  # Convert to format required by RRT
            robot_radius=robot_radius, 
            max_iter=10000
        )
        rrt_path = rrt.planning(animation=False)
        end_time = time.time()
        rrt_planning_time = end_time - start_time
        
        print(f"The rrt path: {rrt_path}")
        print(f"The rrt planning time: {rrt_planning_time}")


        # A* Planning
        grid_size = .5  # Grid resolution
        start_time = time.time()
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        astar_path = a_star.planning(sx, sy, gx, gy)
        end_time = time.time()
        astar_planning_time = end_time - start_time

        print(f"The A* path: {astar_path}")
        print(f"The A* planning time: {astar_planning_time}")

        # Plotting
        plt.figure()
        plt.plot(ox, oy, ".k")
        if rrt_path:
            plt.plot([x for (x, y) in rrt_path], [y for (x, y) in rrt_path], '-r', label='RRT Path')
        if astar_path:
            plt.plot(astar_path[0], astar_path[1], '-b', label='A* Path')
        plt.plot(sx, sy, "og", label='Start')
        plt.plot(gx, gy, "xb", label='Goal')
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.title(f"RRT vs A* - Environment {i+1}")
        plt.pause(1)

    plt.show()

if __name__ == '__main__':
    main()

# def main():
#     num_environments = 5  # Number of different environments to generate
#     grid_max_x, grid_max_y = 100, 100  # Size of the grid area
#     robot_radius = 1.0  # [m]
#     min_distance = 50.0  # Minimum distance between start and goal

#     for i in range(num_environments):
#         ox, oy = generate_obstacles(grid_max_x, grid_max_y)

#         # Generate random start and goal positions with minimum distance
#         while True:
#             sx, sy = generate_random_point(1, grid_max_x - 1)
#             gx, gy = generate_random_point(1, grid_max_y - 1)
#             if (sx, sy) not in zip(ox, oy) and (gx, gy) not in zip(ox, oy) and is_distance_sufficient(sx, sy, gx, gy, min_distance):
#                 break

#         grid_size = 2.0  # Grid resolution
#         a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
#         rx, ry = a_star.planning(sx, sy, gx, gy)

#         if True:  # Set to 'False' to disable animation
#             plt.figure()
#             plt.plot(ox, oy, ".k")
#             plt.plot(sx, sy, "og")
#             plt.plot(gx, gy, "xb")
#             plt.plot(rx, ry, "-r")
#             plt.grid(True)
#             plt.axis("equal")
#             plt.title(f"Environment {i+1}")
#             plt.pause(1)  # Pause to display each environment
    
#     plt.show()
        
#         # time.sleep(5)

# if __name__ == '__main__':
#     main()
