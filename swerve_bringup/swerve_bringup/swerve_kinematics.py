#!/usr/bin/env python3
"""Swerve inverse kinematics for the simulated base.

/cmd_vel in; steering positions and wheel velocities out to the two
ros2_control group controllers.

This replaces swerve_steering_controller/SwerveSteeringController, the
third-party ROS 1 controller the original used. ros2_controllers has no
equivalent for four independently steered modules, so the kinematics live here.

Module geometry, joint order and limits are read from parameters that carry the
same values the ROS 1 ros_controllers.yaml specified.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def normalize_angle(angle):
    """Wrap to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SwerveKinematics(Node):

    def __init__(self):
        super().__init__('swerve_kinematics')

        self.declare_parameter('module_positions',
                               [-0.3, -0.3, -0.3, 0.3, 0.3, -0.3, 0.3, 0.3])
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('steer_limit', 3.14)
        self.declare_parameter('max_wheel_speed', 5.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        p = self.get_parameter
        flat = p('module_positions').value
        self.modules = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.wheel_radius = p('wheel_radius').value
        self.steer_limit = p('steer_limit').value
        self.max_wheel_speed = p('max_wheel_speed').value
        self.cmd_vel_timeout = p('cmd_vel_timeout').value

        self.steer_pub = self.create_publisher(
            Float64MultiArray, 'steering_position_controller/commands', 10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, 'wheel_velocity_controller/commands', 10)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.last_twist = Twist()
        self.last_cmd_time = self.get_clock().now()
        # Hold the previous steering angles so a module can pick the closer of
        # the two equivalent headings rather than swinging half a turn.
        self.last_angles = [0.0] * len(self.modules)

        self.create_timer(1.0 / p('publish_rate').value, self.publish_commands)

        self.get_logger().info(
            f'swerve kinematics up for {len(self.modules)} modules')

    def on_cmd_vel(self, msg):
        self.last_twist = msg
        self.last_cmd_time = self.get_clock().now()

    def solve(self, vx, vy, omega):
        """Return (steer_angles, wheel_speeds) for the module set."""
        angles = []
        speeds = []

        for i, (rx, ry) in enumerate(self.modules):
            # v_module = v_chassis + omega x r
            vx_m = vx - omega * ry
            vy_m = vy + omega * rx

            speed = math.hypot(vx_m, vy_m)

            if speed < 1e-6:
                # No demand: hold the current heading rather than snapping to 0,
                # which would make the modules twitch whenever the stick centres.
                angle = self.last_angles[i]
            else:
                angle = math.atan2(vy_m, vx_m)

            # A module pointed 180 degrees away driving backwards is the same
            # motion, and often a much shorter steer. Take whichever is closer.
            delta = normalize_angle(angle - self.last_angles[i])
            if abs(delta) > math.pi / 2:
                angle = normalize_angle(angle + math.pi)
                speed = -speed

            # Respect the steering joint limits from the URDF.
            angle = max(-self.steer_limit, min(self.steer_limit, angle))

            angles.append(angle)
            speeds.append(speed)

        # Desaturate so the chassis still travels in the commanded direction.
        peak = max(abs(s) for s in speeds) if speeds else 0.0
        if self.max_wheel_speed > 0.0 and peak > 0.0:
            # Convert ground speed to wheel angular velocity for the controller.
            angular = [s / self.wheel_radius for s in speeds]
            peak_angular = max(abs(a) for a in angular)
            if peak_angular > self.max_wheel_speed:
                scale = self.max_wheel_speed / peak_angular
                angular = [a * scale for a in angular]
            speeds = angular
        else:
            speeds = [s / self.wheel_radius for s in speeds]

        return angles, speeds

    def publish_commands(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            # Stop the wheels but leave the steering where it is.
            self.wheel_pub.publish(
                Float64MultiArray(data=[0.0] * len(self.modules)))
            self.steer_pub.publish(Float64MultiArray(data=self.last_angles))
            return

        angles, speeds = self.solve(
            self.last_twist.linear.x,
            self.last_twist.linear.y,
            self.last_twist.angular.z)

        self.last_angles = angles

        self.steer_pub.publish(Float64MultiArray(data=angles))
        self.wheel_pub.publish(Float64MultiArray(data=speeds))


def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
