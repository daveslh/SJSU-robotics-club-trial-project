import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class map_node(Node):

    def __init__(self, x, y):
        super().__init__('map_node')
        self.map_publisher = self.create_publisher(Int32MultiArray, 'map', 10)
        self.x_dim = x
        self.y_dim = y
        self.get_logger().info('Publisher node started.')

    def publish(self):
        msg = Int32MultiArray()
        msg.data = [self.x_dim, self.y_dim]
        self.map_publisher.publish(msg)
        self.get_logger().info(f'Publishing: x = {self.x_dim} y = {self.y_dim}')


def main(args=None):
    rclpy.init(args=args)
    x_input = int(input("Enter x dimensions of map: "))
    y_input = int(input("Enter y dimensions of map: "))
    new_map_node = map_node(x_input, y_input)
    new_map_node.publish()
    try:
        rclpy.spin(new_map_node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
