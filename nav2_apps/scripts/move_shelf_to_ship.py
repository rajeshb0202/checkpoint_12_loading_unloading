import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.action import ActionClient
import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from msgs_attach_shelf.action import GoToLoading
from std_msgs.msg import Empty
from geometry_msgs.msg import Polygon, Point32
from geometry_msgs.msg import Twist
import math
import time




class ShelfToShipNavigator(Node):
    def __init__(self):
        super().__init__('shelf_to_ship_navigator')

        self.initial_position = [0.0, 0.0, 0.0]
        self.loading_position = {"shelf": [5.75, 0.0, -1.57]}
        self.shipping_destinations = {"shipping_position": [0.7, -2.9, -1.57]}
        self.raise_shelf = False
        self.shelf_raised = False
        self.unloaded_shelf_status= False
        self.moved_back_status = False
        self.side_length = 0.6
        timer_period = 0.5          #timer period to call the timer callback method
        self.i = 0                  #this variable is used to print the attach_shelf_action feedback at set frequency.
        self.linear_speed = 0.2
        self.angular_speed = 0.0
        self.move_back_interval = 15
        self.navigator = BasicNavigator()
        self.elevator_down_publisher = self.create_publisher(Empty, '/elevator_down', 10)
        self.global_footprint_publisher = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_footprint_publisher = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.vel_publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self._action_client = ActionClient(self, GoToLoading, '/attach_shelf_action')
        self.set_initial_pose()
        self.execute_initial_navigation_task('shelf')

    
    def timer_callback(self):
        if (self.unloaded_shelf_status and not self.moved_back_status): 
            vel_msg = Twist()
            vel_msg.linear.x = -1.0 * self.linear_speed
            vel_msg.angular.z = self.angular_speed
            # publish the velocity for certian time period
            for i in range (self.move_back_interval):   
                self.get_logger().info("moving the robot out of the shelf")
                self.vel_publisher.publish(vel_msg)
                time.sleep(0.5)
            #stop the robot after it backs to an open area
            self.moved_back_status = True
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.vel_publisher.publish(vel_msg)
            self.timer.cancel()             #cancelling the timer after coming backout of the shelf
            self.get_logger().info("Came out of the shelf and calling navigation task to the intial position")
        else:
            pass
            

            


    # mehtod to publish the footprint to any polygon shape
    def publish_footprint(self, polygon):
        self.global_footprint_publisher.publish(polygon)
        self.local_footprint_publisher.publish(polygon)


    # mehtod to change the robot'footprint to square after loading the shelf (i.e. after successfully finishing the attach_shelf_action)
    def change_to_square_footprint(self):
        half_side = self.side_length / 2
        square_points = [
            Point32(x=half_side, y=half_side, z=0.0),
            Point32(x=-half_side, y=half_side, z=0.0),
            Point32(x=-half_side, y=-half_side, z=0.0),
            Point32(x=half_side, y=-half_side, z=0.0)
        ]
        square_footprint = Polygon()
        square_footprint.points = square_points
        self.get_logger().info("Changing robot's footprint to square")
        self.publish_footprint(square_footprint)


    #method to set the very intial pose
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


    # method to execute intial navigation i.e. going infront of the shelf
    def execute_initial_navigation_task(self, request_item_location):
        self.navigator.waitUntilNav2Active()
        # Navigate to shelf position
        self.go_to_pose(self.loading_position[request_item_location], 'go_near_shelf')
        # Process navigation result for reaching the loading position
        result1 = self.navigator.getResult()
        if result1 == TaskResult.SUCCEEDED:
            self.get_logger().info("reached infront of the shelf")
            self.raise_shelf = True
        else:
            self.handle_navigation_failure(result1, request_item_location)
        # perform shelf raise operation after completing the initial navigation task
        if self.raise_shelf:
            # call action
            self.get_logger().info("called the action /attach_server")
            self.send_attach_shelf_action_goal()


    # method to send goal. It will be called once after the robot reaches infront of the shelf
    def send_attach_shelf_action_goal(self):
        goal_msg = GoToLoading.Goal()
        goal_msg.attach_to_shelf = True
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)        #the feedback method is called
        self._send_goal_future.add_done_callback(self.attach_shelf_action_goal_response_callback)


    # method to process the goal callback of the attach_shelf_action
    def attach_shelf_action_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_attach_shelf_action_result_callback)


    # Callback mehtod to process the result of the attach_shelf_action goal. if it sucess, then it will call the method: proceed to shipping destination
    def get_attach_shelf_action_result_callback(self, future):
        result = future.result().result
        if result.complete:
            self.shelf_raised = True
            # self.get_logger().info("shelf raised status", self.shelf_raised)       # for debug
            self.get_logger().info("successully raised the shelf and brought to open position")
            #change the robot footprint to square after bringing the shelf to the open position
            self.change_to_square_footprint()
            #calling the second navigation task
            self.proceed_to_shipping_destination()            #this is hided for debugging other tasks
        else:
            self.get_logger().info("Failed to raise the shelf or brought to open position")
        
    
    # mehtod to proceed to shipping destination after loading the shelf
    def proceed_to_shipping_destination(self):
        self.get_logger().info("Moving to the shipping destination")
        self.go_to_pose(self.shipping_destinations["shipping_position"], 'go_to_shipping_position')

        # Process navigation result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Reached near the shipping destination.")
            #if succeded, unload the shelf at the shipping position
            self.unload_shelf_at_shipping_position()
        else:
            self.handle_navigation_failure(result, "shipping_position")


    # method to unload the shelf, once the robot reaches the loading position
    def unload_shelf_at_shipping_position(self):
            #unloading the shelf
            msg = Empty()
            self.elevator_down_publisher.publish(msg)
            self.get_logger().info("Successfully unloaded the shelf")
            self.unloaded_shelf_status = True           #after unloading, bring back the robot out of the shelf
            

          
    
    #Callback method to print the feedback of the attach_shelf_action
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.i += 1
        if (self.i%5 == 0):                 #this is to ensure not to print the feedback everytime
            print('Received feedback: {0}'.format(feedback.feedback_operation_status))  


    #mehtod to navigate to a certain position
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
        while not self.navigator.isTaskComplete():                  #to ensure it will wait till the action is finished
            pass


    #method to handle navigation failure if any from the navigation stack.
    def handle_navigation_failure(self, result, location):
        if result == TaskResult.CANCELED:
            print(f'Task at {location} was canceled. Returning to staging point...')
            #add GoToPose to intial point if there is anything failed.
        elif result == TaskResult.FAILED:
            print(f'Task at {location} failed!')
        exit(-1)

    
    #method to convert quaternion to euler angle in radians
    def euler_to_quaternion(self, roll, pitch, yaw):
        return tf_transformations.quaternion_from_euler(roll, pitch, yaw)


def main(args=None):
    rclpy.init(args=args)
    navigator = ShelfToShipNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
