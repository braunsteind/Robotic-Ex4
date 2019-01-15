#!/usr/bin/python

import rospy, sys
import math
from nav_msgs.srv import GetMap
import numpy as np
import a_star as a


def print_map_to_file():
    """
    Writes new grid to file.
    :return: null
    """
    with open("new_grid.txt", "w+") as grid_file:
        # Use converted_grid
        global converted_grid
        for row in reversed(converted_grid):
            for cell in row:
                grid_file.write("1") if cell else grid_file.write("0")
            grid_file.write("\n")


def print_path_string_to_file(start_point, goal_point, path):
    """
    Created path file.
    :param start_point: starting point, marked as 'S'.
    :param goal_point: goal point, marked as 'G'.
    :param path: correct path.
    :return: null
    """
    # Write to file
    with open("path.txt", "w+") as grid_file:
        grid_file.write("Start point: " + " ".join(str(x) for x in start_point) + "\n")
        for line in path:
            grid_file.write(" ".join(str(x) for x in line) + "\n")
        grid_file.write("Goal point: " + " ".join(str(x) for x in goal_point))


def print_path_to_file(start_point, goal_point, path, symbols):
    """
    Created path file.
    :param start_point: starting point, marked as 'S'.
    :param goal_point: goal point, marked as 'G'.
    :param path: correct path.
    :param symbols: Symbols that marks the path direction.
    :return: null
    """
    lines = []

    global converted_grid
    # Create string lines for each line in converted grid
    for row in reversed(converted_grid):
        line = ''
        for cell in row:
            if cell:
                line += '1'
            else:
                line += '0'
        line += '\n'
        lines.append(line)
    # Update start point cell to be 'S'
    x_start_point, y_start_point = start_point
    line = list(lines[y_start_point])
    line[x_start_point] = 'S'
    lines[y_start_point] = "".join(line)

    # Update goal point cell to be 'G'
    x_goal_point, y_goal_point = goal_point
    line = list(lines[y_goal_point])
    line[x_goal_point] = 'G'
    lines[y_goal_point] = "".join(line)
    # Remove first symbol(we have one extra symbol)
    symbols.pop(0)
    # For each move, put on map it's relative symbol
    for move, symbol in zip(path, symbols):
        x_current, y_current = move
        line = list(lines[y_current])
        line[x_current] = symbol
        lines[y_current] = "".join(line)
    # Write to file
    with open("path.txt", "w+") as grid_file:
        for line in lines:
            grid_file.write(line)


def creat_occupancy_grid(my_map):
    """
    creating the occupancy grid
    :param my_map: wanted map to create grid on it
    :return: null
    """
    global grid
    grid = [[None] * my_map.info.width for i in xrange(my_map.info.height)]
    for i in xrange(my_map.info.height):
        for j in xrange(my_map.info.width):
            if my_map.data[i * my_map.info.width + j] == 0:
                grid[i][j] = False
            else:
                grid[i][j] = True


def create_short_path(path):
    short_path = [path[0]]
    prev = path[0]

    for i, p in enumerate(path):
        if prev[0] != p[0] and prev[1] != p[1]:
            short_path.append(path[i-1])
            prev = path[i-1]

    short_path.append(path[-1])
    return short_path


def convert_new_grid_point(size, new_points, res):
    """
    Calculates new point in new grid.
    :param size: new robot size
    :param point: point to convert
    :return: converted point
    """
    global converted_grid, map_origin
    new_points = create_short_path(new_points)
    old_points = []
    for point in new_points:
        x, y = point

        old_point_x = float(x) * (size / res) * res + map_origin[0]
        old_point_y = -(float(y) * (size / res) * res - map_origin[1])
        #
        old_point_x = round(old_point_x * 2) / 2
        old_point_y = round(old_point_y * 2) / 2

        if not (old_point_x, old_point_y) in old_points:
            old_points.append((old_point_x, old_point_y))

    return old_points


def map_origin_set(origin, res, size):
    global map_origin
    map_origin = (origin[0], (size - 1) * res + origin[1])


def convert_old_grid_point(size, point, res):
    """
    Calculates new point in new grid.
    :param size: new robot size
    :param point: point to convert
    :return: converted point
    """
    global converted_grid, map_origin
    x, y = point

    new_point_x = int((x - map_origin[0]) / res / (size / res)) + 1
    new_point_y = int((map_origin[1] - y) / res / (size / res)) + 1

    return [new_point_x, new_point_y]


def create_grid_by_robot_size(my_map, size):
    """
    Creates new grid with perspective to new robot size
    :param my_map: map to use it's info
    :param size: new robot size
    :return: null
    """
    global converted_grid
    # Calc how much cells the robot needs
    new_resolution = int(math.ceil(size / my_map.info.resolution))
    # Calc new width and height with perspective to new resolution
    new_width = int(math.floor(my_map.info.width / new_resolution))
    new_height = int(math.floor(my_map.info.height / new_resolution))
    converted_grid = [[None] * new_width for i in xrange(new_height)]

    i_old_size = 0
    j_old_size = 0
    for i in xrange(new_height):
        for j in xrange(new_width):
            # Mark by default that there is not obstacle
            found_obs = False
            for x in range(i_old_size, i_old_size + new_resolution):
                if found_obs:
                    break
                for y in range(j_old_size, j_old_size + new_resolution):
                    # Check if there is obstacle
                    if my_map.data[x * my_map.info.width + y] != 0:
                        converted_grid[i][j] = True
                        # Stop the iteration as we already mark this cell
                        found_obs = True
                        break
            # Check if there wasn't any obstacle and mark this cell to False
            if not found_obs:
                converted_grid[i][j] = False
            j_old_size += new_resolution
        j_old_size = 0
        i_old_size += new_resolution


def find_closest_empty_place_vertical(start_point, g, direction):
    """
    Find closest empty place vertical
    :param start_point: point to start looking for
    :param g: grid
    :param direction: direction to move
    :return: tuple of new goal point and how it's close to old point
    """
    global converted_grid
    try:
        x, y = start_point
        counter = 0
        while g[y][x]:
            x += direction
            counter += 1
        return [x, y], counter
    except e:
        # If there will be exception(out of borders) - return origin point and max int(so it will
        # be chosen as best point)
        return start_point, sys.maxint


def find_closest_empty_place_horizontal(start_point, g, direction):
    """
    Find closest empty place horizontal
    :param start_point: point to start looking for
    :param g: grid
    :param direction: direction to move
    :return: tuple of new goal point and how it's close to old point
    """
    try:
        x, y = start_point
        counter = 0
        while g[y][x]:
            y += direction
            counter += 1
        return [x, y], counter
    except e:
        # If there will be exception(out of borders) - return origin point and max int(so it will
        # be chosen as best point)
        return start_point, sys.maxint


def validate_goal_location(g_location, g):
    """
    Validate that goal location is not on obstacle. If it is, find closest place to it
    :param g_location: goal location
    :param g: grid
    :return: Goal location not on an obstacle
    """
    x, y = g_location
    # Check if on obstacle
    if g[y][x]:
        rospy.loginfo("Goal point is on an obstacle!!!!, changing goal point...")
        # Check for all direction and find closest point to mark as goal
        new_g_right, c_right = find_closest_empty_place_horizontal(g_location, g, 1)
        new_g_left, c_left = find_closest_empty_place_horizontal(g_location, g, -1)

        new_g_down, c_down = find_closest_empty_place_vertical(g_location, g, 1)
        new_g_up, c_up = find_closest_empty_place_vertical(g_location, g, -1)
        # Get smallest delta
        smallest_delta = min(c_right, c_left, c_down, c_up)
        # Return corresponding goal position
        if smallest_delta == c_right:
            return new_g_right
        elif smallest_delta == c_left:
            return new_g_left
        elif smallest_delta == c_up:
            return new_g_up
        elif smallest_delta == c_down:
            return new_g_down
    else:
        return g_location


def get_path(robot_size, starting_location, goal_location):
    global converted_grid
    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
        rospy.loginfo("Received a %d X %d map @ %.3f m/px" % (
        response.map.info.width, response.map.info.height, response.map.info.resolution))
        # Create new grid by robot size
        create_grid_by_robot_size(response.map, robot_size)

        origin_point = (float(response.map.info.origin.position.x), float(response.map.info.origin.position.y))
        map_origin_set(origin_point, response.map.info.resolution, response.map.info.height)

        # Convert start and goal point to be with perspective to new grid map
        converted_start_point = convert_old_grid_point(robot_size, starting_location, response.map.info.resolution)
        converted_goal_point = convert_old_grid_point(robot_size, goal_location, response.map.info.resolution)
        # Flip grid
        fliped_grid = np.flipud(converted_grid)

        # Check if goal point is on obstacle, and update it if necessary
        converted_goal_point = validate_goal_location(converted_goal_point, fliped_grid)
        # Use the A_start algorithm - it's require the given map to be flip and as numpy array
        test_planner = a.PathPlanner(np.array(fliped_grid), False)
        # Run the algorithm.
        r_path = test_planner.a_star(np.array(converted_start_point), np.array(converted_goal_point))
        r_path += [(converted_goal_point[0], converted_goal_point[1])]
        # Write the grid to 'new_grid' file
        print_map_to_file()
        path_origin_points = convert_new_grid_point(robot_size, r_path, response.map.info.resolution)
        print_path_string_to_file(converted_start_point, converted_goal_point, r_path)
        #print_path_to_file(converted_start_point, converted_goal_point, r_path, r_symbols)
        return path_origin_points, r_path
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)

