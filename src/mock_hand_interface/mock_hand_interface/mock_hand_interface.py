import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

import numpy as np

class MockHandInterfaceNode(Node):

    def __init__(self):
        super().__init__('mock_hand_interface_node')
        timer_rate = 100 # Hz
        self.qos_profile = QoSProfile(depth=10)
        self.state_subscriber = self.create_subscription(Int32, 'hand/state', self.state_callback, self.qos_profile)
        self.joint_state_publisher = self.create_publisher(JointState, 'hand/joint_states', self.qos_profile)
        self.timer = self.create_timer(1/timer_rate, self.timer_callback)
        self.broadcaster = TransformBroadcaster(self, self.qos_profile)
        self.get_logger().info('MockHandInterface has been started')

        self.finger_aa_idx = [
            1, 5, 9, 13, 17
        ]

        self.finger_fe_idx = [
            [2, 3, 4],
            [6, 7, 8],
            [10, 11, 12],
            [14, 15, 16],
            [18, 19, 20],
        ]

        self.thumb_flexion_joint_idx = [
            2, 3, 4
        ]
        self.finger_flexion_joint_idx = [
            6, 7, 8,
            10, 11, 12,
            14, 15, 16,
            18, 19, 20
        ]

        self.initialize_joints()
        self.publish_joint_states()
        self.current_state = 0

        self.states = {
            'OPEN': [-0.25*np.pi, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'CLOSE': [
                0.0, -0.125*np.pi, 
                0.0, -0.5*np.pi, 
                0.0, -0.5*np.pi, 
                0.0, -0.5*np.pi, 
                0.0, -0.5*np.pi
            ],
        }

    def state_callback(self, msg):
        self.current_state = msg.data
        if self.current_state == 1:
            self.joint_positions = self.transform_joint_state(self.states['CLOSE'])
        else:
            self.joint_positions = self.transform_joint_state(self.states['OPEN'])
        # Print the current state
        self.get_logger().info(f'Current state: {self.current_state}')

    def timer_callback(self):
        # angle = -1.309 * np.abs(np.cos(self.get_clock().now().nanoseconds / 1e9))
        # for i in self.thumb_flexion_joint_idx:
        #     self.joint_positions[i] = 0.25 * angle
        # for i in self.finger_flexion_joint_idx:
        #     self.joint_positions[i] = angle
        self.publish_joint_states()

    def initialize_joints(self):
        self.joint_names = [
            'wrist_joint',
            'thumb_aa_joint',
            'thumb_metacarpal_joint',
            'thumb_proximal_joint',
            'thumb_distal_joint',
            'index_metacarpal_joint',
            'index_proximal_joint',
            'index_middle_joint',
            'index_distal_joint',
            'middle_metacarpal_joint',
            'middle_proximal_joint',
            'middle_middle_joint',
            'middle_distal_joint',
            'ring_metacarpal_joint',
            'ring_proximal_joint',
            'ring_middle_joint',
            'ring_distal_joint',
            'little_metacarpal_joint',
            'little_proximal_joint',
            'little_middle_joint',
            'little_distal_joint'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)
    
    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions
        joint_state.velocity = self.joint_velocities
        joint_state.effort = self.joint_efforts
        self.joint_state_publisher.publish(joint_state)

    def transform_joint_state(self, desired_joint_state):
        joint_position = [0.0] * len(self.joint_names)
        for id, (finger_aa_id, finger_fe_idx) in enumerate(zip(self.finger_aa_idx, self.finger_fe_idx)):
            joint_position[finger_aa_id] = desired_joint_state[id * 2]
            for finger_fe_id in finger_fe_idx:
                joint_position[finger_fe_id] = desired_joint_state[id * 2 + 1]
        self.get_logger().info(f'Joint position: {joint_position}')
        return joint_position

def main(args=None):
    rclpy.init(args=args)
    mock_hand_interface_node = MockHandInterfaceNode()
    rclpy.spin(mock_hand_interface_node)
    mock_hand_interface_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()