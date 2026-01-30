## Tunnel Node
# This node simulates driving through a tunnel: 
# 3 seconds of perfect connection, then 2 seconds of total silence.
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from collections import deque
import time

class TunnelNode(Node):
    def __init__(self):
        super().__init__('tunnel_node')
        
        # --- SETTINGS ---
        self.latency_ms = 100.0   # Small base delay
        self.is_connected = True
        
        # Toggle connection every 2.0 seconds (Simulates tunnel entry/exit)
        self.connection_timer = self.create_timer(2.0, self.toggle_connection)
        
        self.subscription = self.create_subscription(
            Twist, '/cmd_input', self.listener_callback, 10)
        self.publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
            
        self.msg_buffer = deque()
        self.process_timer = self.create_timer(0.01, self.process_buffer)
        self.get_logger().info("Tunnel Node Started: Connection toggles every 2s")

    def toggle_connection(self):
        self.is_connected = not self.is_connected
        status = "Connected" if self.is_connected else "BLACKOUT (Tunnel)"
        self.get_logger().warn(f"Network Status: {status}")

    def listener_callback(self, msg):
        self.msg_buffer.append((msg, time.time()))

    def process_buffer(self):
        if not self.msg_buffer:
            return

        current_time = time.time()
        msg, arrival_time = self.msg_buffer[0]
        
        if (current_time - arrival_time) * 1000 >= self.latency_ms:
            self.msg_buffer.popleft()
            
            # ONLY send if we are currently "Connected"
            if self.is_connected:
                self.publisher.publish(msg)
            # If not connected, the message is effectively "dropped" or blocked

def main(args=None):
    rclpy.init(args=args)
    node = TunnelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()