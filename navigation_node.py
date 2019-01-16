#!/usr/bin/python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import tf
import path_planner
from navigation_task.srv import *

# constants
ANGULAR_Z = 0.2
SPEED = 0.1
TURN = 0.1
TURN_LOW = TURN * 0.1
DOWN = 'down'
UP = 'up'
LEFT = 'left'
RIGHT = 'right'


def main():
    global robot_size, listener
    # initialize node
    rospy.init_node('navigation_task_node')
    # set the parameter size
    if rospy.has_param('/robot_size'):
        robot_size = rospy.get_param('/robot_size')
    else:
        robot_size = 0.35
    listener = tf.TransformListener()
    rospy.Rate(2)
    listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
    # run
    rospy.Service('navigate', navigate, navigation_func)
    rospy.spin()


def navigation_func(request):
    # get pos
    (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    x, y = trans[0], trans[1]

    newX = round(x * 2) / 2
    newY = round(y * 2) / 2
    # Check if the point equals to request point
    if newX == request.x and newY == request.y:
        return navigateResponse(True)

    # get the plan from ex3
    path, temp = path_planner.get_plan([newX, newY], [request.x, request.y], robot_size)

    path_list = []
    # if we have 3 or more points
    if len(path) > 2:
        p1 = path[1]
        p2 = path[2]
        p3 = path[0]
        if not (p1[0] == p2[0] and p1[0] == p3[0]) or (p1[1] == p2[1] and p1[1] == p3[1]):
            path_list.append(path[0])

    for i in range(1, len(path) - 1):
        p2 = path[i + 1]
        p3 = path[i - 1]
        p1 = path[i]
        if not (p1[0] == p2[0] and p1[0] == p3[0]) or (p1[1] == p2[1] and p1[1] == p3[1]):
            path_list.append(p1)
    path_list.append(path[len(path) - 1])

    # if the point in path list
    if (newX, newY) in path_list:
        # remove it
        path_list.remove((newX, newY))

    start_moving(path_list)

    return navigateResponse(True)


# converting a given direction to the relevant angle
def dict_to_angle(direction):
    directions = [UP, DOWN, LEFT, RIGHT]
    values = [3.1, 0, -1.57, 1.57]

    for i in range(len(directions)):
        if direction == directions[i]:
            return values[i]

    return 0


def get_robot_direction():
    threshold = 0.15
    (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    # Checks by current yaw in which direction we are
    if 0 - threshold < abs(yaw) < 0 + threshold:
        return DOWN
    elif 3 - threshold < abs(yaw) < 3 + threshold:
        return UP
    elif 1.5 - threshold < yaw < 1.5 + threshold:
        return RIGHT
    elif -1.5 - threshold < yaw < -1.5 + threshold:
        return LEFT
    else:
        return 'UNK'


def update_yaw(direction, publisher):
    threshold = 0.005

    # if unknown direction go to the closest direction to the target
    if direction == 'UNK':
        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        (roll, pitch, yaw) = euler_from_quaternion(rot)
        values = {}
        values[DOWN] = abs(yaw - 0)
        values[RIGHT] = abs(yaw - 1.5)
        values[UP] = abs(abs(yaw) - 3)
        values[LEFT] = abs(yaw + 1.5)
        result = min(values.items(), key=lambda i: i[1])
        direction = result[1]

    target = 0
    if direction == DOWN:
        target = 0
    elif direction == UP:
        target = 3.1
    elif direction == RIGHT:
        target = 1.5
    elif direction == LEFT:
        target = -1.5

    vel_msg = Twist()

    (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    left_or_right = 0
    # check if should turn to the left or to the right
    if yaw < target:
        left_or_right = 1
    elif yaw > target:
        left_or_right = -1
    if direction == UP and yaw < 0:
        left_or_right = -1
    if direction == UP and yaw > 0:
        left_or_right = 1

    # Start turning until we are st to go to the target
    vel_msg.angular.z = TURN_LOW * left_or_right
    if direction == UP:
        yaw = abs(yaw)
    while not (target - threshold) < yaw < (target + threshold):
        direction = get_robot_direction()
        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        (roll, pitch, yaw) = euler_from_quaternion(rot)
        publisher.publish(vel_msg)
        if direction == UP:
            yaw = abs(yaw)


def start_moving(path_list):
    """
    start moving the robot
    :param path_list: The path list with the points
    """
    publisher = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
    threshold = 0.08

    move_msg = Twist()
    move_msg.linear.x = SPEED
    move_msg.angular.z = ANGULAR_Z

    direction = get_robot_direction()
    update_yaw(direction, publisher)

    # go over the points in the path list
    for point in path_list:
        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        x, y = trans[0], trans[1]
        euler_from_quaternion(rot)
        currentX, currentY = x, y
        currentX, currentY = round(currentX * 2) / 2, round(currentY * 2) / 2
        x_g, y_g = point
        # set the way the robot should go
        if currentX < x_g:
            goto = DOWN
        elif currentX > x_g:
            goto = UP
        elif currentY < y_g:
            goto = RIGHT
        elif currentY > y_g:
            goto = LEFT

        # if need to change direction
        if goto != direction:
            direction = change_direction(direction, goto, publisher)

        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        x, y = trans[0], trans[1]
        euler_from_quaternion(rot)
        currentX, currentY = x, y
        x_g, y_g = point
        left_or_right = 1
        should_set_z = False

        # move the robot
        while not ((x_g - threshold < currentX < x_g + threshold) and (y_g - threshold < currentY < y_g + threshold)):
            direction = get_robot_direction()
            if should_set_z:
                move_msg.angular.z = ANGULAR_Z * left_or_right

            publisher.publish(move_msg)
            (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            currentX, currentY = trans[0], trans[1]
            (roll, pitch, yaw) = euler_from_quaternion(rot)

            if yaw < dict_to_angle(goto):
                left_or_right = 1
                should_set_z = True
            else:
                left_or_right = -1
                should_set_z = True


def change_direction(direction, goto, publisher):
    rotate(direction, goto, publisher)
    direction = goto
    update_yaw(direction, publisher)
    return direction


def rotate(current_way, wanted_way, publisher):
    threshold = 0.15
    vel_msg = Twist()
    new_angle = dict_to_angle(wanted_way)
    decide_left_or_right = new_angle - dict_to_angle(current_way)
    # check if should turn to the left or to the right
    if decide_left_or_right < 0:
        left_or_right = -1
    else:
        left_or_right = 1

    # turn z
    vel_msg.angular.z = TURN * left_or_right
    (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    # turn until were ready to go
    while not new_angle - threshold < yaw < new_angle + threshold:
        publisher.publish(vel_msg)
        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        (roll, pitch, yaw) = euler_from_quaternion(rot)
        if wanted_way == UP:
            if -3.2 <= yaw <= - 3.1:
                break


if __name__ == '__main__':
    main()
