#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from swift_msgs.msg import SwiftMsgs
# import rclpy.clock as Clock

class SwiftPicoController(Node):
    def __init__(self):
        super().__init__('pico_controller')

        # Initialize PID constants for [roll, pitch, throttle]
        self.Kp = [0.0, 0.0, 0.0]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]

        # Setpoints and drone position
        self.setpoint = [2.0, 2.0, 19.0]  # Desired setpoint for x, y, z
        self.drone_position = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.error_rate = [0.0, 0.0, 0.0]

        # PID setpoint bias and derivative filter
        self.b = 0.8
        self.alpha = 0.1
        self.prev_derivative = [0.0, 0.0, 0.0]
        
        # Min/max control signal values for [roll, pitch, throttle]
        self.min_values = [1000, 1000, 1000]
        self.max_values = [2000, 2000, 2000]

        # PID update frequency
        self.sample_time = 0.06
        # self.last_time = Clock().now()

        # Initialize SwiftMsgs command message
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1000  # Assuming default value

        # ROS publishers and subscribers
        self.pose_subscriber = self.create_subscription(
            PoseArray, '/whycon/poses', self.whycon_callback, 10)
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        # Subscribing to PID tuning topics
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 10)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 10)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 10)

        # Timer to run PID function periodically
        self.timer = self.create_timer(self.sample_time, self.pid)

        # Arm the drone
        self.arm()

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)
        self.get_logger().info("Disarming the drone.")

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1500 
        self.command_pub.publish(self.cmd)
        self.get_logger().info("Arming the drone.")

    def whycon_callback(self, msg):
        if msg.poses:
            self.drone_position[0] = msg.poses[0].position.x
            self.drone_position[1] = msg.poses[0].position.y
            self.drone_position[2] = msg.poses[0].position.z
            self.get_logger().debug(f"Updated drone position: {self.drone_position}")
        else:
            self.get_logger().warn("No poses received from Whycon.")

    def pid(self):
        # current_time = Clock().now()
        # dt = current_time - self.last_time
        dt = self.sample_time
        if dt >= self.sample_time:
            error = [self.drone_position[i] - self.setpoint[i] for i in range(3)]
            # error = [self.drone_position[i] - self.b * self.setpoint[i] for i in range(3)]
            self.get_logger().debug(f"Current errors: {error}")

            # Calculate PID output
            output = [0.0, 0.0, 0.0]
            for i in range(3):
                # Integral term with windup guarding
                self.error_sum[i] += error[i] * dt
                # self.error_sum[i] = max(min(self.error_sum[i], 1000), -1000)  # Adjust windup limits as needed

                # Derivative term
                self.error_rate[i] = (error[i] - self.prev_error[i]) / dt

                # PID calculation
                proportional = self.Kp[i] * error[i]
                integral = self.Ki[i] * self.error_sum[i]
                derivative = self.Kd[i] * (error[i] - self.prev_error[i]) / dt
                # derivative = self.Kd[i] * self.error_rate[i]
                # derivative = self.alpha * derivative + (1 - self.alpha) * self.prev_derivative[i]
                # self.prev_derivative[i] = derivative

                output[i] = proportional + integral + derivative

                # Limit the output and center around 1500
                output[i] = max(min(output[i] + 1500, self.max_values[i]), self.min_values[i])

                # Update previous error
                self.prev_error[i] = error[i]

                self.get_logger().debug(
                    f"PID[{i}] - P: {proportional:.2f}, I: {integral:.2f}, D: {derivative:.2f}, Output: {output[i]:.2f}"
                )

            # Assign computed output to the respective channels
            self.cmd.rc_roll = int(output[0])
            self.cmd.rc_pitch = int(output[1])
            self.cmd.rc_throttle = int(output[2])

            # Publish command and errors
            self.command_pub.publish(self.cmd)
            self.publish_pid_error(error)

            # self.last_time = current_time

    def publish_pid_error(self, error):
        pid_error = PIDError()
        pid_error.roll_error = error[0]
        pid_error.pitch_error = error[1]
        pid_error.throttle_error = error[2]
        self.pid_error_pub.publish(pid_error)
        self.get_logger().debug(f"Published PID Errors: Roll={error[0]:.2f}, Pitch={error[1]:.2f}, Throttle={error[2]:.2f}")

    # Callback functions to update PID constants
    def altitude_set_pid(self, alt):
        # Without factors
        # self.Kp[2] = alt.kp
        # self.Ki[2] = alt.ki
        # self.Kd[2] = alt.kd

        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.0008
        self.Kd[2] = alt.kd * 0.6
        self.get_logger().info(f"Updated Altitude PID: Kp={self.Kp[2]:.4f}, Ki={self.Ki[2]:.6f}, Kd={self.Kd[2]:.4f}")

    def pitch_set_pid(self, pitch):
        
        # Without factors
        # self.Kp[1] = pitch.kp
        # self.Ki[1] = pitch.ki
        # self.Kd[1] = pitch.kd
        
        self.Kp[1] = pitch.kp * 0.05
        self.Ki[1] = pitch.ki * 0.0008
        self.Kd[1] = pitch.kd * 0.5
        self.get_logger().info(f"Updated Pitch PID: Kp={self.Kp[1]:.4f}, Ki={self.Ki[1]:.6f}, Kd={self.Kd[1]:.4f}")

    def roll_set_pid(self, roll):
        
        # Without factors
        # self.Kp[0] = roll.kp
        # self.Ki[0] = roll.ki
        # self.Kd[0] = roll.kd
        
        self.Kp[0] = roll.kp * 0.001
        self.Ki[0] = roll.ki * 0.0001
        self.Kd[0] = roll.kd * 0.01
        self.get_logger().info(f"Updated Roll PID: Kp={self.Kp[0]:.4f}, Ki={self.Ki[0]:.6f}, Kd={self.Kd[0]:.4f}")

def main(args=None):
    rclpy.init(args=args)
    swift_pico_controller = SwiftPicoController()
    rclpy.spin(swift_pico_controller)
    swift_pico_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
