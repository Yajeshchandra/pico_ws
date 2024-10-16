#!/usr/bin/env python3

# '''
# This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
# This node publishes and subsribes the following topics:

# 		PUBLICATIONS			SUBSCRIPTIONS
# 		/drone_command			/whycon/poses
# 		/pid_error			/throttle_pid
# 						/pitch_pid
# 						/roll_pid
					
# Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
# CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
# '''

# # Importing the required libraries

# from swift_msgs.msg import SwiftMsgs
# from geometry_msgs.msg import PoseArray
# from pid_msg.msg import PIDTune, PIDError
# import rclpy
# from rclpy.node import Node
# import time



# class Swift_Pico(Node):
# 	def __init__(self):
# 		super().__init__('pico_controller')  # initializing ros node with name pico_controller

# 		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
# 		# [x,y,z]
# 		self.drone_position = [0.0, 0.0, 0.0]  

# 		# [x_setpoint, y_setpoint, z_setpoint]
# 		self.setpoint = [2, 2, 19]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

# 		# Declaring a cmd of message type swift_msgs and initializing values
# 		self.cmd = SwiftMsgs()
# 		self.cmd.rc_roll = 1500
# 		self.cmd.rc_pitch = 1500
# 		self.cmd.rc_yaw = 1500
# 		self.cmd.rc_throttle = 1500

# 		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
# 		#after tuning and computing corresponding PID parameters, change the parameters

# 		self.Kp = [0, 0, 0]
# 		self.Ki = [0, 0, 0]
# 		self.Kd = [0, 0, 0]

# 		#-----------------------Add other required variables for pid here ----------------------------------------------

# 		self.prev_error = [0,0,0]
# 		self.max_values = [2000,2000,2000]
# 		self.min_values = [1000,1000,1000]
# 		self.error_sum = [0,0,0]
# 		self.error_rate = [0,0,0]
# 		self.pid_error = PIDError()



# 		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
# 		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
# 		#																	You can change the upper limit and lower limit accordingly. 
# 		#----------------------------------------------------------------------------------------------------------

# 		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
# 		self.sample_time = 0.060  # in seconds
# 		self.last_time = time.time()

# 		# Publishing /drone_command, /pid_error
# 		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
# 		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

# 		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
	

# 		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
# 		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
# 		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
# 		# Subscribing to /throttle_pid, /pitch_pid, /roll_pid
# 		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
# 		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

# 		#------------------------Add other ROS Subscribers here-----------------------------------------------------
	

# 		self.arm()  # ARMING THE DRONE

# 		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)


# 	def disarm(self):
# 		self.cmd.rc_roll = 1000
# 		self.cmd.rc_yaw = 1000
# 		self.cmd.rc_pitch = 1000
# 		self.cmd.rc_throttle = 1000
# 		self.cmd.rc_aux4 = 1000
# 		self.command_pub.publish(self.cmd)
		

# 	def arm(self):
# 		self.disarm()
# 		self.cmd.rc_roll = 1500
# 		self.cmd.rc_yaw = 1500
# 		self.cmd.rc_pitch = 1500
# 		self.cmd.rc_throttle = 1500
# 		self.cmd.rc_aux4 = 2000
# 		self.command_pub.publish(self.cmd)  # Publishing /drone_command


# 	# Whycon callback function
# 	# The function gets executed each time when /whycon node publishes /whycon/poses 
# 	def whycon_callback(self, msg):
# 		self.drone_position[0] = msg.poses[0].position.x 
# 		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
# 		self.drone_position[1] = msg.poses[0].position.y
# 		self.drone_position[2] = msg.poses[0].position.z
# 		#----------------------------------------------------------------------------------------------------------------------
# 		self.pid()  # call the pid function
# 		self.pid_error_pub.publish(self.pid_error)  # Publishing the pid errors


	
# 		#---------------------------------------------------------------------------------------------------------------


# 	# Callback function for /throttle_pid
# 	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
# 	def altitude_set_pid(self, alt):
# 		# Tuning the PID constants for throttle (altitude control)
# 		self.Kp[2] = alt.kp * 0.03  # Modify these scale factors as per your tuning requirements
# 		self.Ki[2] = alt.ki * 0.0001
# 		self.Kd[2] = alt.kd * 0.6

# 	# Callback function for /pitch_pid
# 	# This function gets executed each time when /drone_pid_tuner publishes /pitch_pid
# 	def pitch_set_pid(self, pitch):
# 		# Tuning the PID constants for pitch control
# 		self.Kp[1] = pitch.kp * 0.03  # Scale factor for pitch control
# 		self.Ki[1] = pitch.ki * 0.008
# 		self.Kd[1] = pitch.kd * 0.6

# 	# Callback function for /roll_pid
# 	# This function gets executed each time when /drone_pid_tuner publishes /roll_pid
# 	def roll_set_pid(self, roll):
# 		# Tuning the PID constants for roll control
# 		self.Kp[0] = roll.kp * 0.03  # Scale factor for roll control
# 		self.Ki[0] = roll.ki * 0.008
# 		self.Kd[0] = roll.kd * 0.6


# 	#----------------------------------------------------------------------------------------------------------------------


# 	# def pid(self):

		

# 	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

# 	# Steps:
# 	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
# 	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
# 	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
# 	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
# 	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
# 	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
# 	#																														self.cmd.rcPitch = self.max_values[1]
# 	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
# 	#	8. Add error_sum
# 	def pid(self):
# 		current_time = time.time()
# 		dt = current_time - self.last_time

# 		if dt >= self.sample_time:
# 			error = [0, 0, 0]
# 			output = [0, 0, 0]

# 			for i in range(3):
# 				error[i] = self.drone_position[i] - self.setpoint[i] 
# 				self.error_sum[i] += error[i] * dt
# 				self.error_rate[i] = (error[i] - self.prev_error[i]) / dt

# 				# PID computation
# 				proportional = self.Kp[i] * error[i]
# 				integral = self.Ki[i] * self.error_sum[i]
# 				derivative = self.Kd[i] * self.error_rate[i]

# 				output[i] = proportional + integral + derivative

# 				# Limit the output to max and min values
# 				output[i] = max(min(output[i] + 1500, self.max_values[i]), self.min_values[i])
# 				# Update previous error
# 				self.prev_error[i] = error[i]

# 			# Assign the computed output to the respective channels
# 			self.cmd.rc_roll = int(output[0])
# 			self.cmd.rc_pitch = int(output[1])
# 			self.cmd.rc_throttle = int(output[2])

# 			# Publishing the command and PID errors
# 			self.command_pub.publish(self.cmd)
# 			self.pid_error = PIDError()
# 			self.pid_error.roll_error = error[0]
# 			self.pid_error.pitch_error = error[1]
# 			self.pid_error.throttle_error = error[2]

# 			self.pid_error_pub.publish(self.pid_error)
# 			self.last_time = current_time
# 			#------------------------------------------------------------------------------------------------------------------------
# 			self.command_pub.publish(self.cmd)
# 			# calculate throttle error, pitch error and roll error, then publish it accordingly
# 			self.pid_error_pub.publish(self.pid_error)



# def main(args=None):
# 		rclpy.init(args=args)
# 		swift_pico = Swift_Pico()
# 		rclpy.spin(swift_pico)
# 		swift_pico.destroy_node()
# 		rclpy.shutdown()


# if __name__ == '__main__':
# 		main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

import time

class SwiftPicoController(Node):
    def __init__(self):
        super().__init__('pico_controller')

        # Initialize PID constants (starting with some arbitrary values)
        self.Kp = [0.1, 0.1, 0.1]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]

        self.setpoint = [2.0, 2.0, 19.0]  # Desired setpoint for x, y, z
        self.drone_position = [0.0, 0.0, 0.0]  # Current drone position from pose sensor
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.error_rate = [0.0, 0.0, 0.0]
        
        # Min/max throttle values
        self.min_values = [1000, 1000, 1000]
        self.max_values = [2000, 2000, 2000]

        # For Ziegler-Nichols tuning
        self.Ku = 0
        self.Tu = 0
        self.oscillation_start_time = None
        self.oscillation_end_time = None
        self.oscillations = []
        self.oscillating = False
        self.initial_tuning = True
        self.tuning_step_size = 0.01

        # PID update frequency
        self.sample_time = 0.1  # 10 Hz
        self.last_time = time.time()

        # ROS publishers and subscribers
        self.pose_subscriber = self.create_subscription(
            PoseArray, '/whycon/poses', self.pose_callback, 10)
        self.command_pub = self.create_publisher(Int32, '/drone_command', 10)
        self.timer = self.create_timer(self.sample_time, self.pid)
        self.arm()

    def pose_callback(self, msg):
        # Assume that msg contains the position of the drone
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def pid(self):
        current_time = time.time()
        dt = current_time - self.last_time

        if dt >= self.sample_time:
            error = [0, 0, 0]
            output = [0, 0, 0]

            for i in range(3):
                error[i] = self.setpoint[i] - self.drone_position[i]
                self.error_sum[i] += error[i] * dt
                self.error_rate[i] = (error[i] - self.prev_error[i]) / dt

                proportional = self.Kp[i] * error[i]
                integral = self.Ki[i] * self.error_sum[i]
                derivative = self.Kd[i] * self.error_rate[i]

                output[i] = proportional + integral + derivative

                output[i] = max(min(output[i] + 1500, self.max_values[i]), self.min_values[i])
                self.prev_error[i] = error[i]

            # Publish the command to the drone
            self.publish_drone_command(output)

            # Automated PID tuning
            self.tune_pid(output)

            self.last_time = current_time

    def tune_pid(self, output):
        # Example: tuning only the throttle (z-axis)
        if self.initial_tuning:
            self.Kp[2] += self.tuning_step_size  # Gradually increase Kp for throttle

            if self.is_oscillating(output[2]):
                self.oscillating = True
                if not self.oscillation_start_time:
                    self.oscillation_start_time = time.time()
                elif time.time() - self.oscillation_start_time >= 2:
                    self.oscillation_end_time = time.time()
                    self.Tu = self.oscillation_end_time - self.oscillation_start_time
                    self.Ku = self.Kp[2]
                    self.calculate_final_pid()

                    self.initial_tuning = False
                    self.oscillating = False
            else:
                self.oscillation_start_time = None

    def is_oscillating(self, throttle_output):
        if len(self.oscillations) >= 10:
            self.oscillations.pop(0)
        self.oscillations.append(throttle_output)

        if len(self.oscillations) > 1:
            # Check if the sign of throttle output is changing (oscillation)
            return all(
                self.oscillations[i] * self.oscillations[i-1] < 0
                for i in range(1, len(self.oscillations))
            )
        return False

    def calculate_final_pid(self):
        # Ziegler-Nichols PID tuning
        self.Kp[2] = 0.6 * self.Ku
        self.Ki[2] = 2 * self.Kp[2] / self.Tu
        self.Kd[2] = self.Kp[2] * self.Tu / 8
        self.get_logger().info(f"Final PID values for throttle - Kp: {self.Kp[2]}, Ki: {self.Ki[2]}, Kd: {self.Kd[2]}")

    def publish_drone_command(self, output):
        # Example publishing for throttle (output[2])
        cmd_msg = Int32()
        cmd_msg.data = int(output[2])
        self.command_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    pico_controller = SwiftPicoController()
    rclpy.spin(pico_controller)
    pico_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
