#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from collections import deque
import time
import random

class NetworkDelayNode(Node):
    def __init__(self):
        super().__init__('network_delay_node')

        # --- SETTINGS ---
        self.latency_ms = 2000.0   # 2000ms Delay
        self.drop_probability = 0.0 # 0% Packet Loss (will change to test later)
        self.delay_every_n_commands = 10  # Apply delay to every 10th command

        # Counter for commands
        self.command_counter = 0

        # 1. Taking "Clean" input (from MPC)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_input',
            self.listener_callback,
            10)

        # 2. Publish to turtle_controller (not directly to turtlesim)
        self.publisher = self.create_publisher(
            Twist,
            '/turtle_controller_input',
            10)

        # 3. Publish delay status (True when delay is applied)
        self.delay_status_publisher = self.create_publisher(
            Bool,
            '/network_delay_active',
            10)

        self.msg_buffer = deque()
        self.timer = self.create_timer(0.01, self.process_buffer)
        self.get_logger().info(f"Network Node Started: {self.latency_ms}ms delay every {self.delay_every_n_commands} commands")

    def listener_callback(self, msg):
        # Increment command counter
        self.command_counter += 1

        # Check if this is the Nth command
        should_delay = (self.command_counter % self.delay_every_n_commands == 0)

        # Store message with arrival time and delay flag
        self.msg_buffer.append((msg, time.time(), should_delay))

        if should_delay:
            self.get_logger().info(f"Command #{self.command_counter}: Applying {self.latency_ms}ms delay")
            # Publish delay status as True
            delay_msg = Bool()
            delay_msg.data = True
            self.delay_status_publisher.publish(delay_msg)

    def process_buffer(self):
        if not self.msg_buffer:
            return

        current_time = time.time()
        msg, arrival_time, should_delay = self.msg_buffer[0]

        # Determine the delay to apply
        delay_to_apply = self.latency_ms if should_delay else 0.0

        # If enough time has passed...
        if (current_time - arrival_time) * 1000 >= delay_to_apply:
            self.msg_buffer.popleft()

            # Send it (unless dropped)
            if random.random() > self.drop_probability:
                self.publisher.publish(msg)

                # Publish delay status as False when command is sent
                if should_delay:
                    delay_msg = Bool()
                    delay_msg.data = False
                    self.delay_status_publisher.publish(delay_msg)
                    self.get_logger().info(f"Delayed command published after {self.latency_ms}ms")

def main(args=None):
    rclpy.init(args=args)
    node = NetworkDelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()