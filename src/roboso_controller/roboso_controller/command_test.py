import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import random

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(2.0, self.move_arm)
        self.sent = False  # only send once

        # Define joint limits (from your URDF)
        self.joint_limits = [
            (-0.5, 0.5),   # joint 1
            (-0.5, 0.5),   # joint 2
            (-0.5, 0.5),    # joint 3
            (-0.5, 0.5),   # joint 4
            (-0.5, 0.5)    # joint 5
        ]

    def move_arm(self):
        if self.sent:
            return
        self.sent = True

        traj = JointTrajectory()
        traj.joint_names = ['1', '2', '3', '4', '5']

        num_points = 12  # number of trajectory points
        time_step = 2    # seconds between points
        current_time = 2

        for _ in range(num_points):
            point = JointTrajectoryPoint()
            # Random position within joint limits
            point.positions = [random.uniform(lim[0], lim[1]) for lim in self.joint_limits]
            point.time_from_start = Duration(sec=current_time)
            traj.points.append(point)
            current_time += time_step

        self.publisher_.publish(traj)
        self.get_logger().info(f'Sent {num_points}-point random trajectory!')

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
