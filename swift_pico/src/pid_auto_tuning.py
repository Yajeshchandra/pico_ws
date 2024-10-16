#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune
from swift_msgs.msg import SwiftMsgs
import numpy as np

class PIDAutoTuner(Node):
    def __init__(self):
        super().__init__('pid_auto_tuner')
        
        # Initialize parameters
        self.setpoint = [2.0, 2.0, 19.0]  # Desired setpoint for x, y, z
        self.current_position = [0.0, 0.0, 0.0]
        self.Ku = [0.0, 0.0, 0.0]  # Ultimate gain
        self.Tu = [0.0, 0.0, 0.0]  # Oscillation period
        self.max_amplitude = [0.5, 0.5, 0.5]  # Maximum safe oscillation amplitude
        self.sample_time = 0.06  # 10 Hz, matching the controller
        self.oscillation_count = [0, 0, 0]
        self.last_cross_time = [self.get_clock().now(), self.get_clock().now(), self.get_clock().now()]
        self.max_tuning_time = 10.0  # Maximum tuning time per axis in seconds
        self.current_axis = 2  # 0: roll, 1: pitch, 2: throttle
        self.tuning_phase = "increasing_gain"
        self.initial_Kp = [1.0, 1.0, 1.0]
        self.current_Kp = self.initial_Kp.copy()
        self.last_error = 0.0
        
        # Publishers
        self.roll_pid_pub = self.create_publisher(PIDTune, '/roll_pid', 10)
        self.pitch_pid_pub = self.create_publisher(PIDTune, '/pitch_pid', 10)
        self.throttle_pid_pub = self.create_publisher(PIDTune, '/throttle_pid', 10)
        
        # Subscribers
        self.pose_subscriber = self.create_subscription(
            PoseArray, '/whycon/poses', self.whycon_callback, 10)
        
        # Timer for the tuning loop
        self.timer = self.create_timer(self.sample_time, self.tuning_loop)
        
        self.get_logger().info("PID Auto-Tuner initialized. Starting with roll axis.")

    def whycon_callback(self, msg):
        if msg.poses:
            self.current_position[0] = msg.poses[0].position.x
            self.current_position[1] = msg.poses[0].position.y
            self.current_position[2] = msg.poses[0].position.z
        self.get_logger().debug(f"Current position updated: {self.current_position}")

    def tuning_loop(self):
        if self.current_axis < 0:
            self.get_logger().info("Tuning completed for all axes.")
            self.destroy_node()
            return

        error = self.current_position[self.current_axis] - self.setpoint[self.current_axis]
        self.get_logger().debug(f"Current axis: {self.current_axis}, Error: {error}")

        if self.tuning_phase == "increasing_gain":
            self.increase_gain()
        elif self.tuning_phase == "measuring_oscillation":
            self.measure_oscillation(error)
        
        self.publish_pid_values()

    def increase_gain(self):
        self.current_Kp[self.current_axis] *= 1.1  # Increase gain by 10%
        self.get_logger().debug(f"Increasing gain for axis {self.current_axis}: {self.current_Kp[self.current_axis]}")
        if self.current_Kp[self.current_axis] > 5 * self.initial_Kp[self.current_axis]:
            self.get_logger().warn(f"Axis {self.current_axis} reached maximum gain without oscillation. Moving to next axis.")
            self.move_to_next_axis()
        elif abs(self.current_position[self.current_axis] - self.setpoint[self.current_axis]) > self.max_amplitude[self.current_axis]:
            self.get_logger().info(f"Oscillation detected for axis {self.current_axis}. Starting oscillation measurement.")
            self.tuning_phase = "measuring_oscillation"
            self.oscillation_start_time = self.get_clock().now()

    def measure_oscillation(self, error):
        current_time = self.get_clock().now()
        self.get_logger().debug(f"Measuring oscillation for axis {self.current_axis}, Error: {error}")
        if (current_time - self.oscillation_start_time).nanoseconds / 1e9 > self.max_tuning_time:
            self.calculate_pid_values()
            self.move_to_next_axis()
            return

        if error * self.last_error < 0:  # Zero crossing detected
            self.oscillation_count[self.current_axis] -= 1
            period = (current_time - self.last_cross_time[self.current_axis]).nanoseconds / 1e9
            self.Tu[self.current_axis] = (self.Tu[self.current_axis] * (self.oscillation_count[self.current_axis] - 1) + period) / self.oscillation_count[self.current_axis]
            self.last_cross_time[self.current_axis] = current_time
            self.get_logger().debug(f"Zero crossing detected for axis {self.current_axis}, Oscillation count: {self.oscillation_count[self.current_axis]}, Period: {period}")

        if self.oscillation_count[self.current_axis] >= 10:
            self.calculate_pid_values()
            self.move_to_next_axis()

        self.last_error = error

    def calculate_pid_values(self):
        self.Ku[self.current_axis] = self.current_Kp[self.current_axis]
        Kp = 0.6 * self.Ku[self.current_axis]
        Ki = 1.2 * self.Ku[self.current_axis] / self.Tu[self.current_axis]
        Kd = 3 * self.Ku[self.current_axis] * self.Tu[self.current_axis] / 40
        self.get_logger().info(f"Tuned PID values for axis {self.current_axis}: Kp={Kp:.4f}, Ki={Ki:.4f}, Kd={Kd:.4f}")
        self.current_Kp[self.current_axis] = Kp
        return Kp, Ki, Kd

    def move_to_next_axis(self):
        self.current_axis += 1
        if self.current_axis <= 2:
            self.tuning_phase = "increasing_gain"
            self.oscillation_count[self.current_axis] = 0
            self.current_Kp[self.current_axis] = self.initial_Kp[self.current_axis]
            self.get_logger().info(f"Moving to axis {self.current_axis}")
        else:
            self.get_logger().info("Tuning completed for all axes.")

    def publish_pid_values(self):
        pid_msg = PIDTune()
        pid_msg.kp = self.current_Kp[self.current_axis]
        pid_msg.ki = 0.0  # We're only adjusting Kp during tuning
        pid_msg.kd = 0.0  # We're only adjusting Kp during tuning
        
        if self.current_axis == 0:
            self.roll_pid_pub.publish(pid_msg)
        elif self.current_axis == 1:
            self.pitch_pid_pub.publish(pid_msg)
        elif self.current_axis == 2:
            self.throttle_pid_pub.publish(pid_msg)
        self.get_logger().debug(f"Published PID values for axis {self.current_axis}: Kp={pid_msg.kp}")

def main(args=None):
    rclpy.init(args=args)
    pid_auto_tuner = PIDAutoTuner()
    rclpy.spin(pid_auto_tuner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()