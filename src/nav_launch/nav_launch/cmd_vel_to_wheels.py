import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_launch.msg import MotorSpeedCommand


class CmdVelToWheels(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wheels')
        self.declare_parameter('wheel_base', 0.3)
        self._pub = self.create_publisher(MotorSpeedCommand, '/wheel_cmd', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cb, 10)

    def _cb(self, msg: TwistStamped):
        wheel_base = self.get_parameter('wheel_base').value
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z
        out = MotorSpeedCommand()
        out.header = msg.header
        out.left = linear - (angular * wheel_base / 2.0)
        out.right = linear + (angular * wheel_base / 2.0)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheels()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
