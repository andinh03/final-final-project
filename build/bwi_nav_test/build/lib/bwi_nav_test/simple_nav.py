import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import time

class SimpleNavNode(Node):
    """Simple navigation node for the BWI robot."""
    
    def __init__(self):
        super().__init__('simple_nav_node')
        
        # Create action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Simple Navigation Node initialized!')
        
        # Wait for Nav2 action server to be available
        self.get_logger().info('Waiting for Nav2 action server...')
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 action server...')
        self.get_logger().info('Nav2 action server available!')
    
    def move_to_position(self, x, y):
        """Navigate to a specific position using Nav2."""
        self.get_logger().info(f'Moving to position: x={x}, y={y}')
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation to face forward (quaternion)
        goal_msg.pose.pose.orientation.w = 1.0
        
        # Send goal
        self.get_logger().info('Sending goal...')
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
    result = future.result().result
    status = future.result().status

    if status == 4:  # SUCCEEDED
        self.get_logger().info('Goal succeeded!')
    elif status == 6:  # ABORTED
        self.get_logger().error('Goal aborted — likely unreachable or invalid.')
    else:
        self.get_logger().error(f'Goal failed with status: {status}')
    
    def feedback_callback(self, feedback_msg):
        """Handle the feedback from Nav2."""
        feedback = feedback_msg.feedback
        # You can log navigation progress here if needed
        
    def test_navigation(self):
        """Test navigation by moving to a few positions."""
        # These are example positions - adjust based on your actual map
        positions = [
            (1.0, 0.0),  # 1 meter forward
            (1.0, 1.0),  # 1 meter to the left
            (0.0, 0.0)   # Back to start
        ]
        
        for i, (x, y) in enumerate(positions):
            self.get_logger().info(f"Moving to test position {i+1}: ({x}, {y})")
            self.move_to_position(x, y)
            # Wait for navigation to complete (should use proper synchronization in production)
            time.sleep(15)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavNode()
    
    try:
        # Test navigation
        node.test_navigation()
        
        # Keep the node running to receive callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()