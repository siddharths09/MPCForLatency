#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
import time
import math
import numpy as np
from scipy.optimize import minimize

class MPCSupervisor(Node):
    def __init__(self):
        super().__init__('mpc_supervisor_node')

        # --- SUPERVISOR SETTINGS ---
        self.timeout_threshold = 2.0  # Seconds before MPC kicks in
        self.last_network_msg_time = time.time()
        self.robot_pose = None
        self.current_network_cmd = Twist()
        self.mpc_active = False

        # --- MPC PARAMETERS  ---
        self.horizon = 10   # Look 10 steps into the future
        self.dt = 0.1       # Time step duration
        
        # EMERGENCY GOAL: Where should the robot go if network fails?
        # Option A: A safe "Home" location (Center of map)
        self.goal_x = 5.54
        self.goal_y = 5.54
        # Option B: You could set this dynamically to the last valid waypoint if you wanted.

        # --- SUBSCRIBERS & PUBLISHERS ---
        # 1. Listen to the "Unsafe" Network (Output of Tunnel Node)
        self.net_sub = self.create_subscription(
            Twist, '/cmd_network', self.network_callback, 10)
        
        # 2. Listen to Robot State
        self.pose_sub = self.create_subscription(
            TurtlePose, '/turtle1/pose', self.pose_callback, 10)

        # 3. Publish to Robot
        self.safe_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 4. Control Loop (10Hz to match MPC dt)
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("MPC Supervisor Online: Ready with Scipy Optimization.")

    def network_callback(self, msg):
        self.last_network_msg_time = time.time()
        self.current_network_cmd = msg
        if self.mpc_active:
            self.get_logger().info("Network Restored: Handing back control.")
            self.mpc_active = False

    def pose_callback(self, msg):
        self.robot_pose = msg

    def control_loop(self):
        current_time = time.time()
        time_since_last_packet = current_time - self.last_network_msg_time

        # --- LOGIC: NORMAL vs EMERGENCY ---
        if time_since_last_packet > self.timeout_threshold:
            # BLACKOUT DETECTED
            if not self.mpc_active:
                self.get_logger().warn(f"⚠️ BLACKOUT DETECTED ({time_since_last_packet:.1f}s) -> OPTIMIZING TRAJECTORY")
                self.mpc_active = True
            
            # Run the heavy MPC math
            safe_cmd = self.solve_mpc_emergency()
            self.safe_pub.publish(safe_cmd)

        else:
            # NORMAL MODE (Passthrough)
            self.safe_pub.publish(self.current_network_cmd)

    # ==========================================================
    # MY MPC IMPLEMENTATION (Transplanted)
    # ==========================================================
    
    def model_unicycle(self, state, control):
        """Kinematic model: x, y, theta updates"""
        x, y, theta = state
        v, w = control
        
        x_new = x + v * np.cos(theta) * self.dt
        y_new = y + v * np.sin(theta) * self.dt
        theta_new = theta + w * self.dt
        
        return [x_new, y_new, theta_new]

    def cost_function(self, u_flat, start_pose):
        """Calculates how 'bad' a trajectory is."""
        controls = u_flat.reshape((self.horizon, 2))
        cost = 0.0
        current_state = start_pose
        
        for i in range(self.horizon):
            v = controls[i, 0]
            w = controls[i, 1]
            
            # Simulate forward
            current_state = self.model_unicycle(current_state, [v, w])
            
            # 1. Distance Cost
            dx = self.goal_x - current_state[0]
            dy = self.goal_y - current_state[1]
            dist_sq = dx**2 + dy**2
            cost += dist_sq
            
            # 2. Heading Cost (Point at goal)
            target_angle = np.arctan2(dy, dx)
            angle_error = target_angle - current_state[2]
            # Normalize angle
            angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
            cost += 2.0 * (angle_error**2)
            
            # 3. Effort Cost (Smoothness)
            cost += 0.01 * (v**2 + w**2)

        return cost

    def solve_mpc_emergency(self):
        cmd = Twist()
        if self.robot_pose is None:
            return cmd

        # Define current state [x, y, theta]
        start_state = [self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta]

        # Initial guess (small forward velocity to help gradients)
        initial_guess = np.zeros(self.horizon * 2)
        initial_guess[0::2] = 0.1 

        # Bounds: v=[0, 2.0], w=[-2.0, 2.0]
        bounds = []
        for _ in range(self.horizon):
            bounds.append((0.0, 2.0))
            bounds.append((-2.0, 2.0))

        # --- THE OPTIMIZATION STEP ---
        # Note: We pass 'start_state' as an extra argument to the cost function
        result = minimize(
            self.cost_function, 
            initial_guess, 
            args=(start_state,), 
            bounds=bounds, 
            method='SLSQP'
        )
        
        # Extract the first optimal move
        optimal_controls = result.x.reshape((self.horizon, 2))
        v_opt = optimal_controls[0, 0]
        w_opt = optimal_controls[0, 1]

        cmd.linear.x = float(v_opt)
        cmd.angular.z = float(w_opt)
        
        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = MPCSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()