#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
import json
import time
import math
import traceback

# Helper function (keep as is)
def euler_from_quaternion(quaternion: Quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

class LaneCenteringNode(Node):
    def __init__(self):
        super().__init__('lane_centering_node')

        # --- State Machine ---
        self.STATE_IDLE = "IDLE"
        self.STATE_ALIGNING_BEFORE = "ALIGNING_BEFORE_MOVE"
        self.STATE_MOVING_FORWARD = "MOVING_FORWARD"
        self.STATE_ALIGNING_AFTER = "ALIGNING_AFTER_MOVE"
        self.STATE_SENDING_FINISHED = "SENDING_FINISHED"
        self.state = self.STATE_IDLE

        # --- Parameters ---
        # Proportional gain for centering *during* forward movement
        self.declare_parameter('kp', 0.001) # default 0.001
        # *** NEW: Proportional gain for alignment turns ***
        self.declare_parameter('kp_align', 0.0005) # Start with a value, might need tuning # default 0.005
        # *** INCREASED: Pixel difference tolerance ***
        self.declare_parameter('distance_tolerance_pixels', 20.0) # Increased from 20
        # *** REMOVED: Fixed alignment speed - Now using kp_align ***
        # self.declare_parameter('alignment_angular_vel', 0.3)
        # *** NEW: Max speed for alignment turns (safety clamp) ***
        self.declare_parameter('max_alignment_angular_vel', 0.2) # Clamp alignment speed # default 0.3
        # Max time for alignment attempts
        self.declare_parameter('alignment_timeout_sec', 5.0) # Slightly increased timeout
        # Max angular speed *during* forward move (can be different from alignment)
        self.declare_parameter('max_angular_velocity', 0.5)
        # Constant forward speed
        self.declare_parameter('forward_speed', 0.03)
        # Control loop frequency
        self.declare_parameter('control_frequency', 10.0)
        # Topic Names
        self.declare_parameter('lane_data_topic', 'detected_lane_data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('move_forward_topic', 'move_forward_distance')
        self.declare_parameter('distance_status_topic', '/distance_status')

        # Get parameter values
        self.Kp = self.get_parameter('kp').value
        self.Kp_align = self.get_parameter('kp_align').value # Get new gain
        self.centering_tolerance = self.get_parameter('distance_tolerance_pixels').value
        # self.alignment_angular_vel = self.get_parameter('alignment_angular_vel').value # Removed
        self.max_alignment_angular_vel = self.get_parameter('max_alignment_angular_vel').value # Get new clamp
        self.alignment_timeout = self.get_parameter('alignment_timeout_sec').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.forward_speed = self.get_parameter('forward_speed').value
        control_period = 1.0 / self.get_parameter('control_frequency').value
        lane_data_topic = self.get_parameter('lane_data_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        move_forward_topic = self.get_parameter('move_forward_topic').value
        distance_status_topic = self.get_parameter('distance_status_topic').value

        # --- Publisher ---
        self.velocity_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.distance_status_publisher = self.create_publisher(String, distance_status_topic, 10)
        self.get_logger().info(f" Publishing Distance Status on: '{distance_status_topic}'")

        # --- Subscribers ---
        self.lane_data_subscriber = self.create_subscription(
            String, lane_data_topic, self.line_data_callback, 10)
        self.odometry_subscriber = self.create_subscription(
            Odometry, odom_topic, self.odometry_callback, 10)
        self.move_forward_subscriber = self.create_subscription(
            Float32, move_forward_topic, self.move_forward_callback, 10)

        # --- State Variables ---
        self.latest_line_data = None
        self.latest_odometry = None
        self.target_distance = 0.0
        self.distance_traveled = 0.0
        self.start_pose = None
        self.alignment_start_time = None

        # --- Timer for Control Loop ---
        self.control_timer = self.create_timer(control_period, self.control_loop_callback)

        self.get_logger().info("Lane Centering Node Initialized (Tuned Alignment).")
        self.get_logger().info(f" Kp (Forward): {self.Kp:.4f}, Kp_align: {self.Kp_align:.4f}")
        self.get_logger().info(f" Tolerance: {self.centering_tolerance:.1f} px")
        self.get_logger().info(f" Max Align Vel: {self.max_alignment_angular_vel:.2f} rad/s, Timeout: {self.alignment_timeout:.1f}s")
        self.get_logger().info(f" Max Forward Turn Vel: {self.max_angular_vel:.2f} rad/s")
        self.get_logger().info(f" Forward Speed: {self.forward_speed:.2f} m/s")


    def line_data_callback(self, msg: String):
        """ Processes incoming lane detection data (JSON string). """
        try:
            lane_data = json.loads(msg.data)
            if not isinstance(lane_data, list):
                # self.get_logger().warn(f"Received lane data is not a list: {type(lane_data)}. Skipping.", throttle_duration_sec=5.0)
                self.latest_line_data = None
                return
            self.latest_line_data = lane_data
        except json.JSONDecodeError:
            self.get_logger().error(f"Received invalid JSON data on {self.lane_data_subscriber.topic}: {msg.data[:100]}...", throttle_duration_sec=5.0)
            self.latest_line_data = None
        except Exception as e:
            self.get_logger().error(f"Error processing line data: {e}")
            traceback.print_exc()
            self.latest_line_data = None

    def odometry_callback(self, msg: Odometry):
        """ Stores the latest odometry data. """
        self.latest_odometry = msg

    def move_forward_callback(self, msg: Float32):
        """ Initiates the move sequence, starting with potential pre-alignment. """
        self.get_logger().info("!!! Move forward callback triggered !!!")
        distance_to_move = msg.data

        if distance_to_move <= 0:
            self.get_logger().warn(f"Received non-positive move distance ({distance_to_move:.3f}m). Ignoring.")
            return

        if self.state != self.STATE_IDLE:
            self.get_logger().warn(f"Received move command while not in IDLE state (State: {self.state}). Ignoring.")
            return

        if self.latest_odometry is None:
            self.get_logger().error("Cannot initiate move: Odometry data not available yet. Ignoring command.")
            return

        # Store command details
        self.target_distance = distance_to_move
        self.start_pose = self.latest_odometry.pose.pose
        self.distance_traveled = 0.0

        # --- Check if pre-alignment is needed ---
        centering_error, can_align = self.get_centering_error()

        if can_align and abs(centering_error) > self.centering_tolerance:
            self.get_logger().info(f"Pre-alignment needed (Error: {centering_error:.1f}px > Tol: {self.centering_tolerance:.1f}px).")
            self.state = self.STATE_ALIGNING_BEFORE
            self.alignment_start_time = self.get_clock().now()
            self.get_logger().info(f"State -> {self.state}")
        elif not can_align:
            self.get_logger().info("No lane data for pre-alignment. Skipping.")
            self.state = self.STATE_MOVING_FORWARD
            self.get_logger().info(f"State -> {self.state}")
        else: # Already aligned
            self.get_logger().info(f"Already aligned (Error: {centering_error:.1f}px <= Tol: {self.centering_tolerance:.1f}px).")
            self.state = self.STATE_MOVING_FORWARD
            self.get_logger().info(f"State -> {self.state}")


    def get_centering_error(self) -> (float, bool):
        """
        Calculates the centering error based on latest_line_data.
        Returns (error, can_calculate_flag)
        Error > 0 means robot is too far left (needs to turn right).
        Error < 0 means robot is too far right (needs to turn left).
        """
        if self.latest_line_data is None:
            return 0.0, False

        left_data = next((item for item in self.latest_line_data if item.get('side') == 'left'), None)
        right_data = next((item for item in self.latest_line_data if item.get('side') == 'right'), None)

        if not left_data or not right_data:
            return 0.0, False

        try:
            # Ensure distance_from_center exists and is a number
            dist_left = float(left_data['distance_from_center'])
            dist_right = float(right_data['distance_from_center'])
            # Positive error -> too left, Negative error -> too right
            # Larger error magnitude means further off center
            error = dist_right - dist_left
            return error, True
        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().error(f"Error extracting distance data: {e}. Data: L={left_data}, R={right_data}", throttle_duration_sec=5.0)
            return 0.0, False

    def _calculate_alignment_command(self) -> (float, bool):
        """
        Helper function to calculate angular velocity for alignment using PROPORTIONAL control.
        Returns (target_angular_vel, is_aligned_flag)
        """
        error, can_align = self.get_centering_error()

        if not can_align:
            self.get_logger().warn("Cannot align: No valid lane data.", throttle_duration_sec=5.0)
            return 0.0, True # Treat as "aligned" if we can't calculate, to avoid getting stuck

        is_aligned = abs(error) < self.centering_tolerance
        target_angular_vel = 0.0

        if is_aligned:
            self.get_logger().debug(f"Alignment check: Within tolerance (Error {error:.1f} <= Tol {self.centering_tolerance}).")
        else:
            # *** USE PROPORTIONAL CONTROL FOR ALIGNMENT ***
            # Negative Kp_align: positive error (too left) -> turn right (negative angular vel)
            target_angular_vel = -self.Kp_align * error

            # *** CLAMP the alignment speed ***
            target_angular_vel = max(-self.max_alignment_angular_vel, min(self.max_alignment_angular_vel, target_angular_vel))

            self.get_logger().debug(f"Alignment Calc: Error={error:.1f}, Target Angular Vel: {target_angular_vel:.3f} rad/s")

        return target_angular_vel, is_aligned


    def control_loop_callback(self):
        """ Main control logic executed periodically, driven by state machine. """

        current_time = self.get_clock().now()

        # --- State: IDLE ---
        if self.state == self.STATE_IDLE:
            return

        # --- State: ALIGNING_BEFORE_MOVE ---
        elif self.state == self.STATE_ALIGNING_BEFORE:
            angular_vel, is_aligned = self._calculate_alignment_command()

            if self.alignment_start_time is None:
                 self.get_logger().error("Alignment start time not set! Aborting alignment.")
                 self.stop_robot()
                 return

            elapsed_alignment_time = (current_time - self.alignment_start_time).nanoseconds / 1e9

            if is_aligned:
                self.get_logger().info("Pre-alignment complete.")
                self.stop_robot(set_idle=False)
                self.state = self.STATE_MOVING_FORWARD
                self.get_logger().info(f"State -> {self.state}")
            elif elapsed_alignment_time > self.alignment_timeout:
                self.get_logger().warn(f"Pre-alignment timed out after {elapsed_alignment_time:.1f}s. Proceeding with move.")
                self.stop_robot(set_idle=False)
                self.state = self.STATE_MOVING_FORWARD
                self.get_logger().info(f"State -> {self.state}")
            else:
                # Continue aligning
                self.publish_velocity(0.0, angular_vel) # Only angular velocity
            return

        # --- State: MOVING_FORWARD ---
        elif self.state == self.STATE_MOVING_FORWARD:
            if self.latest_odometry is None or self.start_pose is None:
                self.get_logger().error("MOVING_FORWARD state missing Odometry or start pose! Stopping.")
                self.stop_robot()
                status_msg = String()
                status_msg.data = "error: missing odom/start_pose"
                self.distance_status_publisher.publish(status_msg)
                return

            # Calculate distance traveled
            current_pose = self.latest_odometry.pose.pose
            dx = current_pose.position.x - self.start_pose.position.x
            dy = current_pose.position.y - self.start_pose.position.y
            self.distance_traveled = math.sqrt(dx*dx + dy*dy)

            # Check if target distance reached
            if self.distance_traveled >= self.target_distance:
                self.get_logger().info(f"Target distance {self.target_distance:.3f}m reached (traveled {self.distance_traveled:.3f}m). Stopping linear move.")
                self.stop_robot(set_idle=False)

                # --- Check if post-alignment is needed ---
                centering_error, can_align = self.get_centering_error()

                if can_align and abs(centering_error) > self.centering_tolerance:
                    self.get_logger().info(f"Post-alignment needed (Error: {centering_error:.1f}px > Tol: {self.centering_tolerance:.1f}px).")
                    self.state = self.STATE_ALIGNING_AFTER
                    self.alignment_start_time = self.get_clock().now()
                    self.get_logger().info(f"State -> {self.state}")
                elif not can_align:
                    self.get_logger().info("No lane data for post-alignment. Skipping.")
                    self.state = self.STATE_SENDING_FINISHED
                    self.get_logger().info(f"State -> {self.state}")
                else: # Already aligned
                    self.get_logger().info(f"Already aligned after move (Error: {centering_error:.1f}px <= Tol: {self.centering_tolerance:.1f}px).")
                    self.state = self.STATE_SENDING_FINISHED
                    self.get_logger().info(f"State -> {self.state}")
                return

            # --- If distance not reached, continue moving ---
            linear_vel = self.forward_speed
            angular_vel = 0.0 # Default to straight

            # Attempt centering correction using Kp (for forward motion)
            error, can_align = self.get_centering_error()
            if can_align:
                if abs(error) > self.centering_tolerance:
                    # Apply proportional control, clamped by max_angular_vel (for forward motion)
                    angular_vel = -self.Kp * error
                    angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

            self.publish_velocity(linear_vel, angular_vel)
            return

        # --- State: ALIGNING_AFTER_MOVE ---
        elif self.state == self.STATE_ALIGNING_AFTER:
            angular_vel, is_aligned = self._calculate_alignment_command() # Use alignment logic

            if self.alignment_start_time is None:
                 self.get_logger().error("Alignment start time not set! Aborting alignment.")
                 self.stop_robot()
                 return

            elapsed_alignment_time = (current_time - self.alignment_start_time).nanoseconds / 1e9

            if is_aligned:
                self.get_logger().info("Post-alignment complete.")
                self.stop_robot(set_idle=False)
                self.state = self.STATE_SENDING_FINISHED
                self.get_logger().info(f"State -> {self.state}")
            elif elapsed_alignment_time > self.alignment_timeout:
                self.get_logger().warn(f"Post-alignment timed out after {elapsed_alignment_time:.1f}s. Sending finished status anyway.")
                self.stop_robot(set_idle=False)
                self.state = self.STATE_SENDING_FINISHED
                self.get_logger().info(f"State -> {self.state}")
            else:
                # Continue aligning
                self.publish_velocity(0.0, angular_vel) # Only angular velocity
            return

        # --- State: SENDING_FINISHED ---
        elif self.state == self.STATE_SENDING_FINISHED:
            # Publish the finished status
            status_msg = String()
            status_msg.data = "finished"
            self.get_logger().info(f"Publishing '{status_msg.data}' to {self.distance_status_publisher.topic}")
            self.distance_status_publisher.publish(status_msg)

            # Reset movement variables and return to IDLE
            self.start_pose = None
            self.target_distance = 0.0
            self.distance_traveled = 0.0
            self.alignment_start_time = None
            self.state = self.STATE_IDLE
            self.get_logger().info(f"State -> {self.state}")
            return

        else:
            self.get_logger().error(f"Unknown state: {self.state}. Stopping and setting state to IDLE.")
            self.stop_robot()


    def publish_velocity(self, linear: float, angular: float):
        """ Publishes Twist message """
        twist_msg = Twist()
        # Safety check for speeds
        # Clamp linear speed based on forward_speed parameter
        twist_msg.linear.x = max(-abs(self.forward_speed), min(abs(self.forward_speed), linear))

        # Clamp angular speed based on the *appropriate* limit for the current state
        max_ang_vel_limit = self.max_angular_vel # Default for forward motion
        if self.state == self.STATE_ALIGNING_BEFORE or self.state == self.STATE_ALIGNING_AFTER:
            max_ang_vel_limit = self.max_alignment_angular_vel # Use alignment limit

        twist_msg.angular.z = max(-max_ang_vel_limit, min(max_ang_vel_limit, angular))

        self.velocity_publisher.publish(twist_msg)


    def stop_robot(self, set_idle=True):
        """ Publishes zero velocity to stop the robot and optionally sets state to IDLE """
        self.publish_velocity(0.0, 0.0)
        if set_idle and self.state != self.STATE_IDLE:
            self.state = self.STATE_IDLE
            self.get_logger().info(f"State -> {self.state}")
            # Clear movement variables only when transitioning to IDLE
            self.start_pose = None
            self.target_distance = 0.0
            self.distance_traveled = 0.0
            self.alignment_start_time = None


def main(args=None):
    rclpy.init(args=args)
    lane_centering_node = None
    try:
        lane_centering_node = LaneCenteringNode()
        rclpy.spin(lane_centering_node)
    except KeyboardInterrupt:
        if lane_centering_node:
            lane_centering_node.get_logger().info('Keyboard interrupt, shutting down.')
    except Exception as e:
        if lane_centering_node:
            lane_centering_node.get_logger().error(f"Unhandled exception in main loop: {e}")
        else:
            print(f"Unhandled exception before node initialization: {e}")
        traceback.print_exc()
    finally:
        if lane_centering_node:
            lane_centering_node.get_logger().info('Node shutdown sequence starting.')
            lane_centering_node.stop_robot(set_idle=True)
            time.sleep(0.2)
            lane_centering_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 Shutdown complete.")

if __name__ == '__main__':
    main()
