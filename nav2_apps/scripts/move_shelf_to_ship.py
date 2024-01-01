import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult



class ShelfToShipNavigator(Node):
    def __init__(self):
        super().__init__('shelf_to_ship_navigator')

        self.initial_position = [0.0, 0.0, 0.0]
        self.loading_position = {"shelf_A": [5.75, 0.0, -1.57]}
        self.shipping_destinations = {"shipping_position": [0.7, -3.3, -1.57]}
        self.shelf_raised = True
        self.navigator = BasicNavigator()
        self.set_initial_pose()
        self.execute_navigation_task('shelf_A', 'shipping_position')



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
        self.go_to_pose(self.loading_position[request_item_location], 'shelf')

        # Process navigation result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("reached infornt of shelf")
            if self.shelf_raised:
                self.go_to_pose(self.shipping_destinations[request_destination], 'shipping')
        else:
            self.handle_navigation_failure(result, request_item_location)



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
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
