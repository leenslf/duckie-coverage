#!/usr/bin/env python3
"""
Spawns a green box in Gazebo and sweeps it north-south across the coverage field
so the robot must detect and recover from a moving obstacle in real time.

Run in a separate terminal while demo_coverage is active:
  ros2 run moborobo_robot dynamic_obstacle_mover

Parameters (all optional, set via --ros-args -p name:=value):
  x          Fixed east-west position of the obstacle (default 4.45, field centre)
  y_north    Northern sweep limit (default -5.5, between rows 2-3; keeps obstacle
             away from spawn y=-3.5 and row 1 at y=-4.4 to avoid contact impulse)
  y_south    Southern sweep limit (default -8.5, just inside row 8)
  speed      Sweep speed in m/s (default 0.2)
  model_name Gazebo model name (default 'moving_obstacle')

Ctrl-C cleanly deletes the model from Gazebo before exiting.
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState, DeleteEntity
from geometry_msgs.msg import Pose, Twist

# 1.5 m long (E-W) × 0.5 m deep (N-S) × 1.2 m tall — clearly LiDAR-visible, green
_SDF = """\
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="moving_obstacle">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>1.5 0.5 1.2</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>1.5 0.5 1.2</size></box></geometry>
        <material>
          <ambient>0.1 0.6 0.1 1</ambient>
          <diffuse>0.1 0.6 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class DynamicObstacleMover(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_mover')

        self.declare_parameter('x',          4.45)
        self.declare_parameter('y_north',   -5.5)
        self.declare_parameter('y_south',   -8.5)
        self.declare_parameter('speed',      0.2)
        self.declare_parameter('model_name', 'moving_obstacle')

        self._name    = self.get_parameter('model_name').value
        self._x       = self.get_parameter('x').value
        self._y_north = self.get_parameter('y_north').value
        self._y_south = self.get_parameter('y_south').value
        self._speed   = self.get_parameter('speed').value

        self._y         = float(self._y_south)
        self._direction = 1.0   # start by moving north (toward robot)

        self._spawn_cli  = self.create_client(SpawnEntity,    '/spawn_entity')
        self._state_cli  = self.create_client(SetEntityState, '/set_entity_state')
        self._delete_cli = self.create_client(DeleteEntity,   '/delete_entity')

        self.get_logger().info('Waiting for Gazebo services...')
        self._spawn_cli.wait_for_service(timeout_sec=30.0)
        self._state_cli.wait_for_service(timeout_sec=30.0)

        self._spawn()
        self._timer = self.create_timer(0.1, self._tick)  # 10 Hz

    # ------------------------------------------------------------------

    def _spawn(self):
        req = SpawnEntity.Request()
        req.name         = self._name
        req.xml          = _SDF
        req.initial_pose = self._pose(self._x, self._y_south, 0.6)
        future = self._spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result and result.success:
            self.get_logger().info(
                f'Spawned "{self._name}" at ({self._x:.2f}, {self._y_south:.2f}). '
                f'Sweeping north to y={self._y_north:.1f} at {self._speed} m/s.')
        else:
            msg = result.status_message if result else 'timeout'
            self.get_logger().error(f'Spawn failed: {msg}')

    def _tick(self):
        dt = 0.1
        self._y += self._direction * self._speed * dt

        if self._y <= self._y_south:
            self._y       = self._y_south
            self._direction = 1.0   # bounce: now heading north
        elif self._y >= self._y_north:
            self._y       = self._y_north
            self._direction = -1.0  # bounce: now heading south

        req                       = SetEntityState.Request()
        req.state.name            = self._name
        req.state.pose            = self._pose(self._x, self._y, 0.6)
        req.state.twist           = Twist()
        req.state.reference_frame = 'world'
        self._state_cli.call_async(req)

    def _delete(self):
        req      = DeleteEntity.Request()
        req.name = self._name
        future   = self._delete_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Deleted "{self._name}"')

    @staticmethod
    def _pose(x, y, z) -> Pose:
        p = Pose()
        p.position.x    = float(x)
        p.position.y    = float(y)
        p.position.z    = float(z)
        p.orientation.w = 1.0
        return p

    def destroy_node(self):
        self._delete()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
