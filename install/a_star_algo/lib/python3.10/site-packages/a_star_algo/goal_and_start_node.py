import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class goal_and_start_node(Node):
    def __init__(self):
        super().__init__('goal_and_start_node')
        self.start_publisher = self.create_publisher(Int32MultiArray, 'start', 10)
        self.goal_publisher = self.create_publisher(Int32MultiArray, 'goal', 10)
        self.map_subscriber = self.create_subscription(
            Int32MultiArray,
            'error',
            self.error,
            10)
        self.map_subscriber  # prevent unused variable warning
        self.x_start = None
        self.y_start = None
        self.get_user_input()

    def get_user_input(self):
        self.x_start = int(input("Enter the x index of starting point: "))
        self.y_start = int(input("Enter the y index of starting point: "))

    def publish_start(self):
        start_msg = Int32MultiArray()
        start_msg.data = [self.x_start, self.y_start]
        self.start_publisher.publish(start_msg)
        
    def publish_goal(self):
        goal_msg = Int32MultiArray()
        goal_msg.data = [self.x_goal, self.y_goal]
        self.goal_publisher.publish(goal_msg)
    
    def error(self):
        self.get_logger().error("Error: starting point is an obstacle. Try another starting point.")
        self.get_user_input()


def main(args=None):
    rclpy.init(args=args)
    new_goal_and_start_node = goal_and_start_node()
    try:
        rclpy.spin(new_goal_and_start_node)
    except KeyboardInterrupt:
        pass
    new_goal_and_start_node.publish_start()

if __name__ == '__main__':
    main()
