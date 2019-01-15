#!/usr/bin/python

import math
import sys
from operator import itemgetter

import numpy as np
import rospy
from nav_msgs.srv import GetMap

import a_star


def get_plan(starting_location, goal_location, robot_size):
    rospy.wait_for_service('static_map')

    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
        rospy.loginfo("Received a %d X %d map @ %.3f m/px" % (
            response.map.info.width, response.map.info.height, response.map.info.resolution))

        # Create new grid by robot size and update the starting and goal location
        grid = create_occupancy_grid(response.map, robot_size)
        starting_location = update_grid_location(response.map.info.resolution, robot_size, starting_location)
        goal_location = update_grid_location(response.map.info.resolution, robot_size, goal_location)

        flipud_grid = np.flipud(grid)

        # if the goal is on obstacle
        if is_obstacle(flipud_grid, goal_location):
            goal_location = transfer_location(flipud_grid, goal_location)

        # active A* algorithm
        path = a_star.PathPlanner(np.array(flipud_grid), False).a_star(np.array(starting_location),
                                                                       np.array(goal_location))
        path += [(goal_location[0], goal_location[1])]

        print_map_to_file(grid)

        # TODO change
        origin_point = (float(response.map.info.origin.position.x), float(response.map.info.origin.position.y))
        map_origin = (origin_point[0], (response.map.info.height - 1) * response.map.info.resolution + origin_point[1])
        path_origin_points = convert_new_grid_point(robot_size, r_path, response.map.info.resolution)

        print_path_to_file(starting_location, goal_location, path)
        return path_origin_points, path
    
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


# def get_params():
#     starting_location = 0, 0
#     goal_location = -70, -20
#     robot_size = 0.35
#
#     if rospy.has_param('/starting_location'):
#         starting_location = tuple(map(float, rospy.get_param('/starting_location').split(',')))
#     if rospy.has_param('/goal_location'):
#         goal_location = tuple(map(float, rospy.get_param('/goal_location').split(',')))
#     if rospy.has_param('/robot_size'):
#         robot_size = rospy.get_param('/robot_size')
#     rospy.wait_for_service('static_map')
#
#     return starting_location, goal_location, robot_size


def print_map_to_file(grid):
    with open("new_grid.txt", "w+") as grid_file:
        for row in reversed(grid):
            for cell in row:
                grid_file.write("1") if cell else grid_file.write("0")
            grid_file.write("\n")


def print_path_to_file(starting_location, goal_location, path):
    with open("path.txt", "w+") as grid_file:
        grid_file.write(" ".join(str(x) for x in starting_location) + "\n")
        for line in path:
            grid_file.write(" ".join(str(x) for x in line) + "\n")
        grid_file.write(" ".join(str(x) for x in goal_location))


def update_grid_location(response, robot_size, point):
    x, y = point
    return [int((x - map_origin[0]) / response / (robot_size / response)) + 1,
            int((map_origin[1] - y) / response / (robot_size / response)) + 1]


def create_occupancy_grid(my_map, robot_size):
    # cells for robot
    cell = int(math.ceil(robot_size / my_map.info.resolution))

    width = int(math.floor(my_map.info.width / cell))
    height = int(math.floor(my_map.info.height / cell))

    # build grid
    grid = [[None] * width for _ in xrange(height)]

    # go over the old grid and check for every old cell if there is an obstacle in the smaller cells
    old_height = 0
    old_width = 0
    for i in xrange(height):
        for j in xrange(width):
            obstacle = False
            for k in xrange(old_height, old_height + cell):
                if not obstacle:
                    for z in xrange(old_width, old_width + cell):
                        if my_map.data[k * my_map.info.width + z] != 0:
                            grid[i][j] = True
                            obstacle = True
                            break
            if not obstacle:
                grid[i][j] = False
            old_width += cell
        old_width = 0
        old_height += cell

    return grid


def is_obstacle(grid, location):
    x, y = location
    if grid[y][x]:
        return True
    return False


def transfer_location(grid, location):
    # calculate the steps from clear location
    right = calculate_steps(grid, location, "right")
    left = calculate_steps(grid, location, "left")
    down = calculate_steps(grid, location, "down")
    up = calculate_steps(grid, location, "up")

    # return the closest location
    steps = [up, down, left, right]
    return min(steps, key=itemgetter(1))[0]


def calculate_steps(grid, location, direction):
    x, y = location
    steps = 0

    # look for empty spot or out of bound
    try:
        while grid[y][x]:
            if direction == "left":
                y += -1
            elif direction == "right":
                y += 1
            elif direction == "down":
                x += 1
            else:
                x += -1

            steps += 1

        return [x, y], steps
    # for out of bound
    except:
        return location, sys.maxint


def prepare_path(path):
    new_path = [path[0]]
    old = path[0]

    for i, p in enumerate(path):
        if old[0] != p[0] and old[1] != p[1]:
            new_path.append(path[i - 1])
            old = path[i - 1]

    new_path.append(path[-1])
    return new_path


# TODO change name
def convert_new_grid_point(map_origin, size, points, response):
    """
    Calculates new point in new grid.
    :param size: new robot size
    :param point: point to convert
    :return: converted point
    """
    new_points = prepare_path(points)
    old_points = []
    for point in new_points:
        x, y = point
        # TODO put inside
        old_point_x = float(x) * (size / response) * response + map_origin[0]
        old_point_y = -(float(y) * (size / response) * response - map_origin[1])
        old_point_x = round(old_point_x * 2) / 2
        old_point_y = round(old_point_y * 2) / 2

        if not (old_point_x, old_point_y) in old_points:
            old_points.append((old_point_x, old_point_y))

    return old_points


def map_origin_set(origin, res, size):
    global map_origin
    map_origin = (origin[0], (size - 1) * res + origin[1])
