import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # typisches Topic
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: JointState):
        self.get_logger().info('--- Joint State ---')
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 'n/a'
            vel = msg.velocity[i] if i < len(msg.velocity) else 'n/a'
            eff = msg.effort[i] if i < len(msg.effort) else 'n/a'
            self.get_logger().info(f'{name}: pos={pos}, vel={vel}, effort={eff}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
