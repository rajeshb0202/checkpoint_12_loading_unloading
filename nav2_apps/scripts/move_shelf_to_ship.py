

import time
from copy import deepcopy
import tf_transformations

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

initial_position = [0.0, 0.0, 0.0]

# Shelf positions for picking
shelf_positions = {
    "shelf_A": [5.75, 0.0, -1.57]}

# Shipping destination for picked products
shipping_destinations = {
    "goal_position": [0.7, -3.3, -1.57]}


def euler_to_quaternion(roll, pitch, yaw):
    return tf_transformations.quaternion_from_euler(roll, pitch, yaw)


def main():
    ####################
    request_item_location = 'shelf_A'
    request_destination = 'goal_position'
    ####################

    rclpy.init()

    # Convert Euler angle to quaternion
    initial_orientation_quaternion = euler_to_quaternion(0, 0, initial_position[2])
    shelf_orientation_quaternion = euler_to_quaternion(0, 0, shelf_positions[request_item_location][2])
    


    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = initial_position[0]
    initial_pose.pose.position.y = initial_position[1]
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.x = initial_orientation_quaternion[0]
    initial_pose.pose.orientation.y = initial_orientation_quaternion[1]
    initial_pose.pose.orientation.z = initial_orientation_quaternion[2]
    initial_pose.pose.orientation.w = initial_orientation_quaternion[3]
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = 1.0
    shelf_item_pose.pose.orientation.x = shelf_orientation_quaternion[0]
    shelf_item_pose.pose.orientation.y = shelf_orientation_quaternion[1]
    shelf_item_pose.pose.orientation.z = shelf_orientation_quaternion[2]
    shelf_item_pose.pose.orientation.w = shelf_orientation_quaternion[3]

    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

   
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination + ')...')
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
        shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
        shipping_destination.pose.orientation.z = 1.0
        shipping_destination.pose.orientation.w = shipping_destinations[request_destination][2]
        navigator.goToPose(shipping_destination)

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()