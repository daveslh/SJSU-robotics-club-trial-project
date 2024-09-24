import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np
import pygame
import math

class cell:
    def __init__(self, x, y, width, height, grid):
        self.x = x
        self.y = y
        self.width = width  # Constant width
        self.height = height  # Constant height
        self.obstruct = np.random.choice([True, False])
        if self.obstruct:
            self.color = (0, 0, 0)
        else:
            self.color = (255, 255, 255)
        self.start = False
        self.goal = False
        self.parent = None
        self.neighbors = []
        if not (self.obstruct):
            self.set_neighbors(grid, self)

    def set_neighbors(self, grid, cell):
        rows = grid.rows
        cols = grid.cols

        if self.x > 0:
            self.neighbors.append(grid.cells[self.y][self.x - 1])
        if self.x < (cols - 1):
            self.neighbors.append(grid.cells[self.y][self.x + 1])
        if self.y > 0:
            self.neighbors.append(grid.cells[self.y - 1][self.x])
        if self.y < (rows - 1):
            self.neighbors.append(grid.cells[self.y + 1][self.x])

    def get_neighbors(self):
        return self.neighbors

    def draw(self, win, offset_x, offset_y, is_start=False, is_end=False, is_path=False):
        if is_start:
            color = (0, 255, 0)
        elif is_end:
            color = (255, 0, 0)
        elif is_path:
            color = (255, 0, 0)
        else:
            color = self.color
        x_pos = self.x * self.width + offset_x
        y_pos = self.y * self.height + offset_y
        pygame.draw.rect(win, color, (x_pos, y_pos, self.width, self.height), 1)

    def get_distance(self, grid):
            distance = math.sqrt((grid.goal_cell.x - self.x) ** 2 + (grid.goal_cell.y - self.y) ** 2)
            return distance

class grid:
    def __init__(self, rows, cols, width, height, path_node):
        self.rows = rows
        self.cols = cols
        self.width = width
        self.height = height
        self.block_size = width // cols
        self.cells = []
        self.create_cells()

    def create_cells(self):
        for i in range(self.rows):
            for j in range(self.cols):
                new_cell = cell(j, i, self.block_size, self.block_size, self)
                self.cells.append(new_cell)

    def draw(self, win):
        offset_x = (self.width - (self.cols * self.block_size)) // 2
        offset_y = (self.height - (self.rows * self.block_size)) // 2
        for cell in self.cells:
            cell.draw(win, offset_x, offset_y)

class path_node(Node):
    def __init__(self):
        super().__init__('path_node')
        self.map_subscriber = self.create_subscription(
            Int32MultiArray,
            'map',
            self.create_map,
            10)
        self.map_subscriber  # prevent unused variable warning
        self.start_subscriber = self.create_subscription(
            Int32MultiArray,
            'start',
            self.create_start,
            10)
        self.start_subscriber  # prevent unused variable warning
        self.goal_subscriber = self.create_subscription(
            Int32MultiArray,
            'goal',
            self.create_goal,
            10)
        self.goal_subscriber  # prevent unused variable warning
        self.new_map = None
        self.goal_cell = None
        self.start_cell = None

    def create_map(self, msg):
        x_y_dim = msg.data
        width = 800
        height = 800
        self.new_map = grid(x_y_dim[1], x_y_dim[0], width, height, self)
        return self.new_map
        
    def create_goal(self, msg):
        if self.new_map is not None:
            x_y_goal = msg.data
            self.goal_cell = self.new_map[x_y_goal[1] - 1][x_y_goal[0] - 1]
            if self.goal_cell.obstruct == True:
                self.error_publisher = self.create_publisher(Int32MultiArray, 'error', 10)
                self.error_message = Int32MultiArray
                self.error_message.data = 1
                self.error_publisher.publish(self.error_message)
            else:
                self.goal_cell.goal = True

    def create_start(self, msg,):
        if self.new_map is not None:
            x_y_start = msg.data
            self.start_cell = self.new_map[x_y_start[1] - 1][x_y_start[0] - 1]
            if self.start_cell.obstruct == True:
                self.error_publisher = self.create_publisher(Int32MultiArray, 'error', 10)
                self.error_message = Int32MultiArray
                self.error_message.data = 1
                self.error_publisher.publish(self.error_message)
            else:
                self.start_cell.start = True

    def find_path(self):
        open_set = set()
        closed_set = set()
        open_set.add(self.start_cell)
        self.start_cell.cumulative_cost = 0
        self.start_cell.heuristic = self.start_cell.get_distance(self.new_map)
        self.start_cell.total_cost = self.start_cell.cumulative_cost + self.start_cell.heuristic
        path = []

        while open_set:
            current_node = min(open_set, key=lambda x: x.total_cost)
            if current_node == self.goal_cell:
                while current_node is not None:
                    path.append(current_node)
                    current_node = current_node.parent
                path.reverse()
                return path

            open_set.remove(current_node)
            closed_set.add(current_node)

            for neighbor in current_node.get_neighbors():
                if neighbor not in closed_set:
                    new_cumulative = current_node.cumulative_cost + 1
                    if neighbor not in open_set or new_cumulative < neighbor.cumulative_cost:
                        neighbor.parent = current_node
                        neighbor.cumulative_cost = new_cumulative
                        neighbor.heuristic = neighbor.get_distance(self.new_map)
                        neighbor.total_cost = neighbor.cumulative_cost + neighbor.heuristic
                        open_set.add(neighbor)

        # Print the path if it exists
        if path:
            print("Path found:")
            for cell in path:
                print(f"cell: ({cell.x}, {cell.y})")
        else:
            print("No path found")
        return None


def main(args=None):
    rclpy.init(args=args)
    # pygame.init()
    # win = pygame.display.set_mode((800, 800))
    # pygame.display.set_caption("Map")
    new_path_node = path_node()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(new_path_node)

            # for event in pygame.event.get():
            #     if event.type == pygame.QUIT:
            #         break

            # win.fill((255, 255, 255))
            # if new_path_node.new_map is not None:
            #     new_path_node.new_map.draw(win)
            # pygame.display.update()

    except KeyboardInterrupt:
        pass
    # pygame.quit()

if __name__ == '__main__':
    main()
