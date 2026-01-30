#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
import numpy as np
from scipy.optimize import minimize
from collections import deque

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscribers
        # 1. Subscribe to commands from network_delay_node
        self.cmd_sub = self.create_subscription(
            Twist,
            '/turtle_controller_input',
            self.cmd_callback,
            10)

        # 2. Subscribe to delay status from network_delay_node
        self.delay_status_sub = self.create_subscription(
            Bool,
            '/network_delay_active',
            self.delay_status_callback,
            10)

        # 3. Subscribe to turtle pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

        # State variables
        self.pose = Pose()
        self.pose_received = False
        self.delay_active = False

        # Command history: store last 10 commands from teleop
        self.command_history = deque(maxlen=10)

        # MPC Parameters
        self.horizon = 10
        self.dt = 0.1

        # Predicted goal state (when delay is active)
        self.predicted_goal_x = None
        self.predicted_goal_y = None

        # MPC control loop timer (only active during delay)
        self.mpc_timer = None

        self.get_logger().info("Turtle Controller Started")

    def pose_callback(self, msg):
        """Update current turtle pose"""
        self.pose = msg
        self.pose_received = True

    def cmd_callback(self, msg):
        """
        Receive commands from network_delay_node.
        Store in history and forward to turtle (if no delay).
        """
        # Add command to history
        self.command_history.append({
            'linear': msg.linear.x,
            'angular': msg.angular.z
        })

        # If no delay is active, check for obstacles and forward the command
        if not self.delay_active:
            self.cmd_vel_pub.publish(msg)
            self.get_logger().debug(f"Forwarding command: v={msg.linear.x:.2f}, w={msg.angular.z:.2f}")

    def delay_status_callback(self, msg):
        """
        Handle delay status changes from network_delay_node.
        When delay becomes active (True), predict goal and start MPC.
        When delay ends (False), stop MPC and resume normal operation.
        """
        self.delay_active = msg.data

        if self.delay_active:
            self.get_logger().warn("⚠️  Network delay detected! Activating MPC prediction...")

            # Predict goal state based on last 10 commands
            self.predict_goal_from_history()

            # Start MPC control loop
            if self.mpc_timer is None:
                self.mpc_timer = self.create_timer(self.dt, self.mpc_control_loop)
        else:
            self.get_logger().info("✓ Network delay resolved. Resuming normal operation.")

            # Stop MPC control loop
            if self.mpc_timer is not None:
                self.mpc_timer.cancel()
                self.mpc_timer = None

            # Reset predicted goal
            self.predicted_goal_x = None
            self.predicted_goal_y = None

    def predict_goal_from_history(self):
        """
        Predict where the turtle would be if the last 10 commands were applied.
        This gives us a goal state for the MPC to navigate to during delay.
        """
        if len(self.command_history) == 0 or not self.pose_received:
            self.get_logger().warn("Cannot predict goal: insufficient data")
            return

        # Start from current pose
        predicted_x = self.pose.x
        predicted_y = self.pose.y
        predicted_theta = self.pose.theta

        # Simulate forward using the command history
        for cmd in self.command_history:
            v = cmd['linear']
            w = cmd['angular']

            # Simple kinematic model (same as MPC)
            predicted_x += v * np.cos(predicted_theta) * self.dt
            predicted_y += v * np.sin(predicted_theta) * self.dt
            predicted_theta += w * self.dt

        self.predicted_goal_x = predicted_x
        self.predicted_goal_y = predicted_y

        self.get_logger().info(
            f"📍 Predicted goal: ({self.predicted_goal_x:.2f}, {self.predicted_goal_y:.2f}) "
            f"from current ({self.pose.x:.2f}, {self.pose.y:.2f})"
        )

    def mpc_control_loop(self):
        """
        MPC control loop - only runs during network delay.
        Navigates turtle toward predicted goal state.
        """
        if not self.pose_received or self.predicted_goal_x is None:
            return

        # Check if we've reached the predicted goal
        dist_to_goal = np.sqrt(
            (self.predicted_goal_x - self.pose.x)**2 +
            (self.predicted_goal_y - self.pose.y)**2
        )

        if dist_to_goal < 0.1:
            self.get_logger().info("Predicted goal reached! Waiting for delay to resolve...")
            self.stop_robot()
            return

        # Initial guess for optimizer
        initial_guess = np.zeros(self.horizon * 2)
        initial_guess[0::2] = 0.1  # Small forward velocity

        # Constraints on velocity
        bounds = []
        for _ in range(self.horizon):
            bounds.append((0.0, 2.0))    # v bounds
            bounds.append((-2.0, 2.0))   # w bounds

        # Run optimization
        result = minimize(
            self.cost_function,
            initial_guess,
            bounds=bounds,
            method='SLSQP'
        )

        # Apply first optimal control
        optimal_controls = result.x.reshape((self.horizon, 2))
        v_opt = optimal_controls[0, 0]
        w_opt = optimal_controls[0, 1]

        # Publish MPC command
        msg = Twist()
        msg.linear.x = float(v_opt)
        msg.angular.z = float(w_opt)
        self.cmd_vel_pub.publish(msg)

        self.get_logger().debug(f"MPC control: v={v_opt:.2f}, w={w_opt:.2f}, dist={dist_to_goal:.2f}")

    def model_unicycle(self, state, control):
        """
        Kinematic model of the turtle.
        State: [x, y, theta]
        Control: [v, w]
        """
        x, y, theta = state
        v, w = control

        x_new = x + v * np.cos(theta) * self.dt
        y_new = y + v * np.sin(theta) * self.dt
        theta_new = theta + w * self.dt

        return [x_new, y_new, theta_new]

    def cost_function(self, u_flat):
        """
        MPC cost function - minimize distance to predicted goal while avoiding obstacles.
        """
        controls = u_flat.reshape((self.horizon, 2))

        cost = 0.0
        current_state = [self.pose.x, self.pose.y, self.pose.theta]

        for i in range(self.horizon):
            v = controls[i, 0]
            w = controls[i, 1]

            # Simulate one step forward
            current_state = self.model_unicycle(current_state, [v, w])

            # 1. Distance cost to predicted goal
            dx = self.predicted_goal_x - current_state[0]
            dy = self.predicted_goal_y - current_state[1]
            dist_sq = dx**2 + dy**2
            cost += dist_sq

            # 2. Heading cost (point toward goal)
            target_angle = np.arctan2(dy, dx)
            angle_error = target_angle - current_state[2]
            angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
            cost += 2.0 * (angle_error**2)

            # 3. Control effort cost (smooth movement)
            cost += 0.01 * (v**2 + w**2)
        return cost

    def stop_robot(self):
        """Stop the turtle"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
