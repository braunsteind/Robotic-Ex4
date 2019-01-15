#!/usr/bin/python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import tf
import path_planner
from navigation_task.srv import *

# constants
DOWN = 'down'
UP = 'up'
LEFT = 'left'
RIGHT = 'right'


def navigationFunc(request):


    (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    x, y = trans[0], trans[1]

    newX = round(x * 2) / 2
    newY = round(y * 2) / 2
    # Check if already in current point
    if newX == request.x and newY == request.y:
        return navigateResponse(True)


    path, temp = path_planner.get_plan([newX, newY], [request.x, request.y], robot_size)



    short_path = []

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

        if (p1[0] == p2[0] and p1[0] == p3[0]) or (p1[1] == p2[1] and p1[1] == p3[1]):
            continue
        else:
            short_path.append(p1)
    short_path.append(path[len(path) - 1])


    if (newX, newY) in short_path:
        short_path.remove((newX, newY))


    mover = MoveRobot(short_path)
    mover.move_to_target()

    return navigateResponse(True)








# converting a given direction to the relevant angle
def dictToAngle(direction):

    directions = [UP, DOWN, LEFT, RIGHT]
    values = [3.1, 0, -1.57, 1.57]

    for i in range(len(directions)):
        if direction == directions[i]:
            return values[i]

    return 0





def get_robot_direction():

    threshold = 0.15
    (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    x, y = trans[0], trans[1]
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
        return 'Cant_decide'







class MoveRobot(object):




    def __init__(self, wantedPath, forward_speed=0.1, rotation_speed=0.2):

        # initialize all parameters
        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed
        self.path = wantedPath
        self.min_dist_from_obstacle = forward_speed + 0.2
        self.keep_moving = True
        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)








    def calibrate_yaw(self, direction):

        threshold = 0.005
        slow_turn_speed = 0.01

        if direction == 'Cant_decide':
            print 'Cant decide case calibrate'
            (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            x, y = trans[0], trans[1]
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            values = {}
            values[DOWN] = abs(yaw - 0)
            values[RIGHT] = abs(yaw - 1.5)
            values[UP] = abs(abs(yaw) - 3)
            values[LEFT] = abs(yaw + 1.5)
            # Check min value of closest direction and calibrate to it
            result = min(values.items(), key=lambda i: i[1])
            direction = result[1]

        wanted_val = 0
        if direction == DOWN:
            wanted_val = 0
        elif direction == UP:
            wanted_val = 3.1
        elif direction == RIGHT:
            wanted_val = 1.5
        elif direction == LEFT:
            wanted_val = -1.5

        vel_msg = Twist()

        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        x, y = trans[0], trans[1]
        (roll, pitch, yaw) = euler_from_quaternion(rot)
        turn_direction = 0
        # Check which direction to turn to by current yaw
        if yaw < wanted_val:
            turn_direction = 1
        elif yaw > wanted_val:
            turn_direction = -1

        if direction == UP and yaw < 0:
            turn_direction = -1

        if direction == UP and yaw > 0:
            turn_direction = 1
        # Set z value
        vel_msg.angular.z = slow_turn_speed * turn_direction

        if direction == UP:
            yaw = abs(yaw)

        print 'Calibrating angle slowly, please wait!'
        while not (wanted_val - threshold) < yaw < (wanted_val + threshold):
            direction = get_robot_direction()
            (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            x, y = trans[0], trans[1]
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            self.command_pub.publish(vel_msg)
            if direction == UP:
                yaw = abs(yaw)









    def move_to_target(self):

        threshold = 0.08

        turn_for_accuracy = 0.2

        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        move_msg.angular.z = turn_for_accuracy

        direction = get_robot_direction()
        self.calibrate_yaw(direction)

        for point in self.path:
            (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            x, y = trans[0], trans[1]
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            currentX, currentY, wanted_yaw = x, y, yaw



            currentX, currentY = round(currentX * 2) / 2, round(currentY * 2) / 2
            x_g, y_g = point

            if currentX < x_g:
                next_direction = DOWN
            elif currentX > x_g:
                next_direction = UP
            elif currentY < y_g:
                next_direction = RIGHT
            elif currentY > y_g:
                next_direction = LEFT



            if next_direction != direction:

                self.rotate(direction, next_direction)
                direction = next_direction
                self.calibrate_yaw(direction)

                (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
                x, y = trans[0], trans[1]
                (roll, pitch, yaw) = euler_from_quaternion(rot)
                currentX, currentY = x, y


            x_g, y_g = point
            turn_around_dir = 1
            fix_round_move = False

            while not ((x_g - threshold < currentX < x_g + threshold) and (y_g - threshold < currentY < y_g + threshold)):
                direction = get_robot_direction()

                if fix_round_move:
                    move_msg.angular.z = turn_for_accuracy * turn_around_dir


                self.command_pub.publish(move_msg)
                (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
                currentX, currentY = trans[0], trans[1]
                (roll, pitch, yaw) = euler_from_quaternion(rot)

                if yaw < dictToAngle(next_direction):
                    turn_around_dir = 1
                    fix_round_move = True
                else:
                    turn_around_dir = -1
                    fix_round_move = True






    def rotate(self, currentWay, wantedDirection):


        threshold = 0.15
        vel_msg = Twist()
        goal_angle = dictToAngle(wantedDirection)
        check_dir_move = goal_angle - dictToAngle(currentWay)

        if check_dir_move < 0:
            turn_direction = -1
        else:
            turn_direction = 1

        vel_msg.angular.z = self.rotation_speed * turn_direction
        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        x, y = trans[0], trans[1]
        (roll, pitch, yaw) = euler_from_quaternion(rot)

        while not goal_angle - threshold < yaw < goal_angle + threshold:

            self.command_pub.publish(vel_msg)
            (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            x, y = trans[0], trans[1]
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            if wantedDirection == UP:
                if -3.2 <= yaw <= - 3.1:
                    break






if __name__ == '__main__':
    # initialize node
    rospy.init_node('navigation_task_node')
    # set the parameter size
    if rospy.has_param('/robot_size'):
        robot_size = rospy.get_param('/robot_size')
    else:
        robot_size = 0.35

    listener = tf.TransformListener()
    rate = rospy.Rate(2)
    listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
    # run
    s = rospy.Service('navigate', navigate, navigationFunc)
    rospy.spin()