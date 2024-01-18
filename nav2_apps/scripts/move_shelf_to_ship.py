import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.action import ActionClient
import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from msgs_attach_shelf.action import GoToLoading




class ShelfToShipNavigator(Node):
    def __init__(self):
        super().__init__('shelf_to_ship_navigator')

        self.initial_position = [0.0, 0.0, 0.0]
        self.loading_position = {"shelf": [5.75, 0.0, -1.57]}
        self.shipping_destinations = {"shipping_position": [0.7, -3.3, -1.57]}
        self.raise_shelf = False
        self.shelf_raised = False
        self.navigator = BasicNavigator()
        self._action_client = ActionClient(self, GoToLoading, '/attach_shelf_action')
        self.set_initial_pose()
        self.execute_navigation_task('shelf', 'shipping_position')



    #method to convert quaternion to euler angle in radians
    def euler_to_quaternion(self, roll, pitch, yaw):
        return tf_transformations.quaternion_from_euler(roll, pitch, yaw)


    #method to set the intial pose
    def set_initial_pose(self):
        quaternion = self.euler_to_quaternion(0, 0, self.initial_position[2])
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_position[0]
        initial_pose.pose.position.y = self.initial_position[1]
        initial_pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.orientation.w = quaternion[3]
        self.navigator.setInitialPose(initial_pose)

    

    def execute_navigation_task(self, request_item_location, request_destination):
        self.navigator.waitUntilNav2Active()

        # Navigate to shelf position
        self.go_to_pose(self.loading_position[request_item_location], 'go_near_shelf')

        # Process navigation result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("reached infront of the shelf")
            self.raise_shelf = True
        else:
            self.handle_navigation_failure(result, request_item_location)
        if self.raise_shelf:
            # call action
            self.send_goal()
            
            #if the result of action returns true then shelf.raised = True

        # if self.shelf_raised:
        #     self.go_to_pose(self.shipping_destinations[request_destination], 'go_to_shipping_position')

    def send_goal(self):
        goal_msg = GoToLoading.Goal()
        goal_msg.attach_to_shelf = True
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.complete:
            self.shelf_raised = True
            self.get_logger().info("successully raised the shelf and brought to open position")
        else:
            self.get_logger().info("Failed to raise the shelf or brought to open position")
    

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback_operation_status))



    def go_to_pose(self, position, task_name):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        quaternion = self.euler_to_quaternion(0, 0, position[2])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        print(f'Received request for {task_name} at {position}.')
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            pass

    def handle_navigation_failure(self, result, location):
        if result == TaskResult.CANCELED:
            print(f'Task at {location} was canceled. Returning to staging point...')
        elif result == TaskResult.FAILED:
            print(f'Task at {location} failed!')
        exit(-1)



def main(args=None):
    rclpy.init(args=args)
    navigator = ShelfToShipNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
