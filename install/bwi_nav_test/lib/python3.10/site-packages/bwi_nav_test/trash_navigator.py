
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import time

class TrashNavigationNode(Node):
    """ROS2 node for navigating the BWI robot between trash cans."""
    
    def __init__(self):
        super().__init__('trash_navigation_node')
        
        # Create publishers, subscribers, and action clients
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create a transform listener to get robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store positions of trash cans (to be updated with actual positions or detection)
        self.trash_can_positions = {
            'empty_trash': {'x': 1.0, 'y': 0.0},
            'full_trash': {'x': 3.0, 'y': 0.0}
        }
        
        self.get_logger().info('Trash Navigation Node initialized!')
        
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
        """Handle the result of navigation."""
        result = future.result().result
        status = future.result().status
        if status == 4:  # StatusEnum.SUCCEEDED
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
    
    def feedback_callback(self, feedback_msg):
        """Handle the feedback from Nav2."""
        feedback = feedback_msg.feedback
        # You can log navigation progress here if needed
        
    def move_between_trash_cans(self):
        """Navigate between the two trash cans in sequence."""
        # Move to empty trash can
        self.move_to_position(
            self.trash_can_positions['empty_trash']['x'],
            self.trash_can_positions['empty_trash']['y']
        )
        
        # Wait for navigation to complete
        time.sleep(10)  # Simple delay, in a real system you'd wait for the result callback
        
        # Move to full trash can
        self.move_to_position(
            self.trash_can_positions['full_trash']['x'],
            self.trash_can_positions['full_trash']['y']
        )
        
    def get_current_pose(self):
        """Get the current position of the robot."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            return x, y
        except TransformException as ex:
            self.get_logger().error(f'Could not get current pose: {ex}')
            return None, None

def main(args=None):
    rclpy.init(args=args)
    node = TrashNavigationNode()
    
    try:
        # Example: Move between trash cans
        node.move_between_trash_cans()
        
        # Keep the node running to receive callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()