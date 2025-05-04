import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from waveshare_servo_driver.servo_control import ServoControl

class JointTrajectorySubscriber(Node):
    def __init__(self,config ):
        super().__init__('joint_trajectory_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_commands',  # typical topic for joint states
            self.listener_callback,
            10
        )
        self.sc = ServoControl(config)

    def listener_callback(self, msg: JointTrajectory):
        self.get_logger().info('--- Joint Commands ---')
        for leg_index, point in enumerate(msg.points):
            hip_angle, shoulder_angle ,ankle_angle = point.positions

            # self.get_logger().info(f'{i}: pos={pos}, vel={vel}, effort={eff}')
            self.sc.process_msg(leg_index,hip_angle,shoulder_angle,ankle_angle)
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
    node = JointTrajectorySubscriber(config)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
