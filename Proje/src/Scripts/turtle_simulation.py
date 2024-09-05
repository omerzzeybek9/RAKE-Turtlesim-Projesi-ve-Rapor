#!/usr/bin/env python
#Import necessary libraries

import math
import rospy
from geometry_msgs.msg import Twist
from pexpect import spawn
from turtlesim.srv import Spawn
from turtlesim.msg import Pose

#Creating a class

class TurtleSimulation:
    def __init__(self):
        #Initalize Ros node
        rospy.init_node('turtle_simulation')

        #Target coordinates from ROS parameters
        self.coordinates = rospy.get_param("~coordinates", "")
        if not self.coordinates:
            rospy.logerr("No coordinates parameter provided.")
            rospy.signal_shutdown("No coordinates")

        self.target_points = self.parse_coordinates(self.coordinates)

        #Subscriber and publisher for turtle1
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        #Pose of turtle1
        self.turtle1_pose = Pose()

        rospy.wait_for_service("spawn")
        self.spawn_target_turtle()

    def parse_coordinates(self, coordinates):
        coords = coordinates.split(',')
        if len(coords) % 2 != 0:
            rospy.logerr("Number of coordinates must be an even number.")
            rospy.signal_shutdown("Invalid coordinates")

        parsed_coords = [(float(coords[i]), float(coords[i + 1])) for i in range(0, len(coords), 2)]

        for x, y in parsed_coords:
            if x < 0 or x > 11 or y < 0 or y > 11:
                rospy.logerr(f"Coordinate ({x}, {y}) is out of bounds.")
                rospy.signal_shutdown("Coordinates out of bounds")

        rospy.loginfo(f"Parsed coordinates: {parsed_coords}")
        return parsed_coords

    def update_pose(self, msg):
        #Update the pose based on received msg for turtle1
        self.turtle1_pose = msg

    def spawn_target_turtle(self):
        #Spawn turtles
        for i, (x, y) in enumerate(self.target_points):
            spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
            spawn_turtle(x,y,0, "turtle{}".format(i+2))


    def move_to_target(self, target_x, target_y):
        #Move to target using P-Control
        velocity = Twist()

        while not rospy.is_shutdown():
            distance = self.euclidean_distance(target_x, target_y)
            #Computes linear and angular speed
            linear_speed = min(1.5 * distance, 1.0)
            angular_speed = 4 * (math.atan2(target_y - self.turtle1_pose.y,
                                            target_x - self.turtle1_pose.x) - self.turtle1_pose.theta)
            angular_speed = min(max(angular_speed, -1.0), 1.0)

            #Setting the velocity message
            velocity.linear.x = linear_speed
            velocity.angular.z = angular_speed
            self.vel_pub.publish(velocity)

            #Checks if turtle has arrived to target
            if distance < 0.1:
                break
        #Stop turtle
        self.vel_pub.publish(Twist())

    def euclidean_distance(self, goal_x, goal_y):
        #Euclidean distance between the current pose and the goal
        return math.sqrt(pow((goal_x - self.turtle1_pose.x), 2) + pow((goal_y - self.turtle1_pose.y), 2))

    def run(self):
        #Move turtle to each target
        for target_x, target_y in self.target_points:
            rospy.loginfo(f"Moving to target : ({target_x} , {target_y})")
            self.move_to_target(target_x, target_y)

if __name__ == '__main__':
    #Run node
    run = TurtleSimulation()
    run.run()