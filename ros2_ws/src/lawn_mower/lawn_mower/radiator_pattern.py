import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class RadiatorPattern(Node):
    def __init__(self, goal_points):
        self.xp = None
        self.yp = None
        self.yaw = None
        self.ang_vel = 0
        self.line_vel = 0
        self.command = Twist()
        self.parameters = [1, 0.0001, 0.9, 3, 0.0001, 0.8]

        self.linear_acc_limit = (10, -10)
        self.angular_acc_limit = (1, -1)

        self.chase_distance = 10
        self.distance_int = 0
        self.yaw_int = 0
        self.yaw_diff = 0
        self.goal_points = goal_points
        self.goal_x = 0
        self.goal_y = 0

        super().__init__('go_radiator')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # meg type, topic name
        self.subscription_pose = self.create_subscription(Pose, '/turtle1/pose', self._pose_callback, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self._reach_goal)

        # self.subscription_pose

    def _pose_callback(self, msg):
        self.xp = msg.x
        self.yp = msg.y
        self.yaw = msg.theta
        self.speed = msg.linear_velocity
        self.angular_speed = msg.angular_velocity

    def _reach_goal(self):

        for i in range(0, len(self.goal_points)):
            self.goal_x = self.goal_points[i][0]
            self.goal_y = self.goal_points[i][1]
            self.rotate()
            print('Rotate Complete')
            self.move_to()
            print('Move complete')

    def move_to(self):
        self.chase_distance = math.sqrt((self.xp - self.goal_x) ** 2 + (self.yp - self.goal_y) ** 2)
        while self.chase_distance > 0.07:
            self.command.linear.x = self._set_linear_speed()
            # self.command.linear.x = 5
            self.command.angular.z = 0.0
            self.pub.publish(self.command)

    def rotate(self):
        target_rad = math.atan2(self.goal_y - self.yp, self.goal_x - self.xp)
        self.yaw_diff = target_rad - self.yaw
        print('YAW diff : ', self.yaw_diff)
        while self.yaw_diff > 0.01 or self.yaw_diff < -0.01:
            self.command.angular.z = self._set_angular_speed()
            self.command.linear.x = 0.0
            self.pub.publish(self.command)

    def _set_linear_speed(self):
        self.chase_distance = math.sqrt((self.xp - self.goal_x) ** 2 + (self.yp - self.goal_y) ** 2)
        self.distance_int = self.distance_int + self.chase_distance
        command_linear_speed = (self.parameters[0] * self.chase_distance) + ((-self.parameters[2]) * self.line_vel) + \
                               (self.parameters[1] * self.distance_int)
        linear_acc = (command_linear_speed - self.record_linear_speed[-1]) / 0.01

        if linear_acc > self.linear_acc_limit[0]:
            linear_acc = self.linear_acc_limit[0]
            command_linear_speed = (linear_acc * 0.01) + self.record_linear_speed[-1]

        if linear_acc < self.linear_acc_limit[1]:
            linear_acc = self.linear_acc_limit[1]
            command_linear_speed = (linear_acc * 0.01) + self.record_linear_speed[-1]

        return command_linear_speed


def main():
    params = [10, 0.0001, 0.8, 2, 0.0001, 0.8]
    goal_points = [[10, 1], [10, 2], [1, 2], [1, 3], [10, 3], [10, 4], [1, 4], [1, 5], [10, 5]]
    obj = RadiatorPattern(goal_points)


if __name__ == '__main__':
    main()
