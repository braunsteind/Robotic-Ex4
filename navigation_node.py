#!/usr/bin/python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import tf
import path_planner as pp
from navigation_task.srv import *


def handle_navigate(request):
    """
    Handles navigation request
    :param request: request
    :return:
    """
    # Get robot location
    start_x, start_y, yaw = get_robot_location()
    # Round point to .5
    rounded_x = round(start_x * 2) / 2
    rounded_y = round(start_y * 2) / 2
    # Check if already in current point
    if rounded_x == request.x and rounded_y == request.y:
        return navigateResponse(True)
    # Get path
    path, temp = pp.get_path(robot_size, [rounded_x, rounded_y], [request.x, request.y])
    # Create short path
    short_path = create_short_path(path)
    # Check if start point in short path and remove it
    if (rounded_x, rounded_y) in short_path:
        short_path.remove((rounded_x, rounded_y))
    # Move robot to point
    mb = MoveRobot(short_path)
    mb.move_to_target()
    # Return true
    return navigateResponse(True)


def navigation_server():
    """
    Navigation server
    :return: null
    """
    s = rospy.Service('navigate', navigate, handle_navigate)
    rospy.spin()


def get_robot_location():
    """
    Returns robot location
    :return: robot location - (x, y, yaw)
    """
    (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    robot_x = trans[0]
    robot_y = trans[1]
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    rospy.loginfo("current position (%f,%f)" % (robot_x, robot_y))
    rospy.loginfo("yaw is: %f", yaw)
    return robot_x, robot_y, yaw


def create_short_path(path):
    """
    Returns short path from given path
    :param path: path for robot
    :return: short path
    """
    short_path = []
    # Check if there is more then 2 points to check first point
    if len(path) > 2:
        p1 = path[1]
        p2 = path[2]
        p3 = path[0]
        # Check if we can remove current point from path
        if (p1[0] == p2[0] and p1[0] == p3[0]) or (p1[1] == p2[1] and p1[1] == p3[1]):
            x = 1
        else:
            short_path.append(path[0])

    for i in range(1, len(path) - 1):
        p2 = path[i + 1]
        p3 = path[i - 1]
        p1 = path[i]
        # Check if we can remove current point from path
        if (p1[0] == p2[0] and p1[0] == p3[0]) or (p1[1] == p2[1] and p1[1] == p3[1]):
            continue
        else:
            short_path.append(p1)
    short_path.append(path[len(path) - 1])
    return short_path


def convert_direction_to_angle(direction):
    """
    Converts direction to angle
    :param direction: direction in string
    :return: angle
    """
    if direction == 'up':
        return 3.1
    elif direction == 'down':
        return 0
    elif direction == 'left':
        return -1.57
    elif direction == 'right':
        return 1.57
    else:
        return 0


def get_robot_direction():
    """
    Returns robot direction
    :return: robot direction
    """
    threshold = 0.15
    x, y, yaw = get_robot_location()
    # Checks by current yaw in which direction we are
    if 0 - threshold < abs(yaw) < 0 + threshold:
        return 'down'
    elif 3 - threshold < abs(yaw) < 3 + threshold:
        return 'up'
    elif 1.5 - threshold < yaw < 1.5 + threshold:
        return 'right'
    elif -1.5 - threshold < yaw < -1.5 + threshold:
        return 'left'
    else:
        return 'Cant_decide'


class MoveRobot(object):
    def __init__(self, path_to_goal, forward_speed=0.1, rotation_speed=0.2):
        """
        Constructor
        :param path_to_goal: path to goal
        :param forward_speed: wanted forward speed
        :param rotation_speed: wanted rotation speed
        """
        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed
        self.path = path_to_goal
        # Set dynamically min dist from obstacle with forward speed params
        self.min_dist_from_obstacle = forward_speed + 0.2
        self.keep_moving = True
        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

    def calibrate_yaw(self, direction):
        """
        Calibrate yaw to closest yaw to target
        :param direction: wanted direction to calibrate to
        :return: null
        """
        threshold = 0.005
        slow_turn_speed = 0.01

        # Check if the algorithm couldn't decide in which direction the robot is turn to, and find closest direction
        if direction == 'Cant_decide':
            print 'Cant decide case calibrate'
            x, y, yaw = get_robot_location()
            values = {}
            values['down'] = abs(yaw - 0)
            values['right'] = abs(yaw - 1.5)
            values['up'] = abs(abs(yaw) - 3)
            values['left'] = abs(yaw + 1.5)
            # Check min value of closest direction and calibrate to it
            result = min(values.items(), key=lambda i: i[1])
            direction = result[1]

        wanted_val = 0
        if direction == 'down':
            wanted_val = 0
        elif direction == 'up':
            wanted_val = 3.1
        elif direction == 'right':
            wanted_val = 1.5
        elif direction == 'left':
            wanted_val = -1.5

        vel_msg = Twist()

        x, y, yaw = get_robot_location()
        turn_direction = 0
        # Check which direction to turn to by current yaw
        if yaw < wanted_val:
            turn_direction = 1
        elif yaw > wanted_val:
            turn_direction = -1

        if direction == 'up' and yaw < 0:
            turn_direction = -1

        if direction == 'up' and yaw > 0:
            turn_direction = 1
        # Set z value
        vel_msg.angular.z = slow_turn_speed * turn_direction

        if direction == 'up':
            yaw = abs(yaw)

        print 'Calibrating angle slowly, please wait!'
        while not (wanted_val - threshold) < yaw < (wanted_val + threshold):
            direction = get_robot_direction()
            x, y, yaw = get_robot_location()
            self.command_pub.publish(vel_msg)
            if direction == 'up':
                yaw = abs(yaw)

    def get_next_direction(self, current_point, goal_point):
        """
        Returns next direction
        :param current_point: current point
        :param goal_point: goal point
        :return: next direction
        """
        # round point to .5
        x_c, y_c = round(current_point[0] * 2) / 2, round(current_point[1] * 2) / 2
        x_g, y_g = goal_point

        if x_c < x_g:
            return 'down'
        elif x_c > x_g:
            return 'up'
        elif y_c < y_g:
            return 'right'
        elif y_c > y_g:
            return 'left'

    def move_to_target(self):
        """
        Moves robot to target trough path
        :return: null
        """
        # Set threshold
        threshold = 0.08
        # Accuracy for keeping in line
        turn_for_accuracy = 0.2

        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        move_msg.angular.z = turn_for_accuracy
        # Calibrate robot according to current path
        direction = get_robot_direction()
        self.calibrate_yaw(direction)

        for point in self.path:
            x_c, y_c, wanted_yaw = get_robot_location()
            # Get next direction
            next_direction = self.get_next_direction((x_c, y_c), point)
            # Check if we need to turn
            if next_direction != direction:
                # Turn and calibrate
                self.turn_around(direction, next_direction)
                direction = next_direction
                self.calibrate_yaw(direction)

            x_c, y_c, yaw = get_robot_location()
            x_g, y_g = point
            turn_around_dir = 1
            fix_round_move = False
            # Move robot to wanted point
            while not ((x_g - threshold < x_c < x_g + threshold) and (y_g - threshold < y_c < y_g + threshold)):
                direction = get_robot_direction()
                # Check if we need to fix robot yaw
                if fix_round_move:
                    move_msg.angular.z = turn_for_accuracy * turn_around_dir

                # Send publish to move robot
                self.command_pub.publish(move_msg)
                x_c, y_c, yaw = get_robot_location()
                # Check to which direction we want to fix yaw
                if yaw < convert_direction_to_angle(next_direction):
                    turn_around_dir = 1
                    fix_round_move = True
                else:
                    turn_around_dir = -1
                    fix_round_move = True

    def turn_around(self, current_direction, goal_direction):
        """
        Turns around the robot to goal direction
        :param current_direction: current direction
        :param goal_direction: goal direction
        :return: nothing
        """
        # Set threshold
        threshold = 0.15
        vel_msg = Twist()
        goal_angle = convert_direction_to_angle(goal_direction)
        check_dir_move = goal_angle - convert_direction_to_angle(current_direction)
        # Check which direction to turn
        if check_dir_move < 0:
            turn_direction = -1
        else:
            turn_direction = 1

        # Set direction speed as turn direction params
        vel_msg.angular.z = self.rotation_speed * turn_direction
        x, y, yaw = get_robot_location()
        # Turn around until we set wanted angle
        while not goal_angle - threshold < yaw < goal_angle + threshold:
            self.command_pub.publish(vel_msg)
            x, y, yaw = get_robot_location()
            if goal_direction == 'up':
                if -3.2 <= yaw <= - 3.1:
                    break


if __name__ == '__main__':
    rospy.init_node('navigation_task_node')
    # Default parameter
    robot_size = 0.35
    if rospy.has_param('/robot_size'):
        robot_size = rospy.get_param('/robot_size')

    listener = tf.TransformListener()
    rate = rospy.Rate(2)
    listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
    navigation_server()




