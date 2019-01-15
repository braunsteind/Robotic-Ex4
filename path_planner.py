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
        points = (float(response.map.info.origin.position.x), float(response.map.info.origin.position.y))
        my_map = (points[0], (response.map.info.height - 1) * response.map.info.resolution + points[1])
        starting_location = update_grid_location(response.map.info.resolution, robot_size, starting_location,
                                                 my_map)
        goal_location = update_grid_location(response.map.info.resolution, robot_size, goal_location, my_map)

        flipud_grid = np.flipud(grid)

        # if the goal is on obstacle
        if is_obstacle(flipud_grid, goal_location):
            goal_location = transfer_location(flipud_grid, goal_location)

        # active A* algorithm
        path = a_star.PathPlanner(np.array(flipud_grid), False).a_star(np.array(starting_location),
                                                                       np.array(goal_location))
        path += [(goal_location[0], goal_location[1])]

        print_map_to_file(grid)
        path2 = upgrade_grid_point(my_map, robot_size, path, response.map.info.resolution)
        print_path_to_file(starting_location, goal_location, path)
        return path2, path

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


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


def update_grid_location(response, robot_size, point, my_map):
    x, y = point
    return [int((x - my_map[0]) / response / (robot_size / response)) + 1,
            int((my_map[1] - y) / response / (robot_size / response)) + 1]


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


def upgrade_grid_point(my_map, size, points, response):
    points = prepare_path(points)
    prev_points = []
    for point in points:
        x, y = point
        prev_x = round((float(x) * (size / response) * response + my_map[0]) * 2) / 2
        prev_y = round((-(float(y) * (size / response) * response - my_map[1])) * 2) / 2
        if not (prev_x, prev_y) in prev_points:
            prev_points.append((prev_x, prev_y))
    return prev_points
