import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from waveshare_servo_driver.servo_control import ServoControl

class JointStateSubscriber(Node):
    def __init__(self,config ):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # typical topic for joint states
            self.listener_callback,
            10
        )
        self.sc = ServoControl(config)

    def listener_callback(self, msg: JointState):
        self.get_logger().info('--- Joint State ---')
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 'n/a'
            vel = msg.velocity[i] if i < len(msg.velocity) else 'n/a'
            eff = msg.effort[i] if i < len(msg.effort) else 'n/a'
            self.get_logger().info(f'{name}: pos={pos}, vel={vel}, effort={eff}')
            self.sc.process_msg(name,pos,vel,eff)
        self.sc.move_positions()


def main(args=None):
    rclpy.init(args=args)

    # Get the path to the package's share directory using ROS2
    package_name = 'waveshare_servo_driver'  # Replace with your actual package name
    package_share_directory = get_package_share_directory(package_name)

    # Construct the path to the config file
    config_file_path = os.path.join(package_share_directory, 'config', 'config.yaml')

    # Print the config file path for debugging
    print(f"Config File Path: {config_file_path}")
    print("--------------------")
    
    # Open and load the YAML config file
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)

    board_front = config['board_front']
    board_back = config['board_back']
    
    # Initialize and spin the node
    node = JointStateSubscriber(config)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
