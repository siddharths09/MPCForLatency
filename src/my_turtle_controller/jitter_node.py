## Jitter Node
#This node simulates WiFi 
# instability where ping spikes randomly (e.g., between 300ms and 700ms).
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from collections import deque
import time
import random

class JitterNode(Node):
    def __init__(self):
        super().__init__('jitter_node')
        
        # --- SETTINGS ---
        self.min_latency = 300.0   # Minimum delay (ms)
        self.max_latency = 700.0   # Maximum delay (ms)
        
        self.subscription = self.create_subscription(
            Twist, '/cmd_input', self.listener_callback, 10)
        self.publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
            
        self.msg_buffer = deque()
        self.timer = self.create_timer(0.01, self.process_buffer)
        self.get_logger().info(f"Jitter Node Started: {self.min_latency}-{self.max_latency}ms range")

    def listener_callback(self, msg):
        # Assign a RANDOM delay to this specific packet right now
        target_delay = random.uniform(self.min_latency, self.max_latency)
        # Store: (Message, Arrival Time, Target Delay for this packet)
        self.msg_buffer.append((msg, time.time(), target_delay))

    def process_buffer(self):
        if not self.msg_buffer:
            return

        current_time = time.time()
        # Peek at the tuple: (msg, arrival_time, target_delay)
        msg, arrival_time, target_delay = self.msg_buffer[0]
        
        # Check if THIS specific packet has waited long enough
        if (current_time - arrival_time) * 1000 >= target_delay:
            self.msg_buffer.popleft()
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JitterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()