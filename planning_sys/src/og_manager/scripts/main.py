#!/usr/bin/env python3

import rospy

import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from nav_msgs.msg import OccupancyGrid, Odometry

from geometry_msgs.msg import PointStamped

from tf.transformations import euler_from_quaternion

# from utils import bresenham

def bresenham(x0, y0, x1, y1):
    ''' Bresenham's line algorithm '''
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x, y))
    return points


class GridManager:
    # MAP_TOPIC = '/registered_scan'
    MAP_TOPIC = '/explored_areas'
    ODOM_TOPIC = '/state_estimation'
    OCCUPANCY_TOPIC = '/occupancy_grid'
    GRID_RESOLUTION = .3
    DEFAULT_HEIGHT = 0.0
    MIN_HEIGHT = 1.5
    MAX_HEIGHT = 2

    class GridState:
        UNEXPLORED = -1
        FREE = 0
        VISITED = 50
        OCCUPIED = 100

    def __init__(self, grid_height, grid_width):
        rospy.init_node('og_manager', anonymous=True)

        # subscribers 
        self.map_sub = rospy.Subscriber(self.MAP_TOPIC, PointCloud2, self._map_callback)
        self.odom_sub = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self._odom_callback)
        # publishers
        self.goal_pub = rospy.Publisher(self.GOAL_TOPIC, PointStamped, queue_size=10)
        self.occupany_grid_pub = rospy.Publisher(self.OCCUPANCY_TOPIC, OccupancyGrid, queue_size=10)

        # Iniitialize variables for grid
        self.grid_map_origin = (0,0)
        self.grid_origin = (int(grid_width//2),int(grid_height//2))
        self.map = None
        self.grid_mat = np.zeros((grid_height, grid_width))+self.GridState.UNEXPLORED
        self.grid = None
        self.position_in_grid = None
        self.orientation_in_grid = None

        # other variables related to planning
        self.odom = None
        self.rate = rospy.Rate(5)



    def run(self):
        while not rospy.is_shutdown():
            # self.plan_and_publish_goal()
            self.rate.sleep()

    def _map_callback(self, msg):
        self.map = msg
        if self.map is not None and self.odom is not None:
            # publish occupancy grid
            self._publish_grid()

    def _odom_callback(self, msg):
        self.odom = msg

    def _get_region_around_robot(self, radius, grid_pos):
        ''' Returns a set of coordinates around the robot'''
        # get current orientation
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, current_yaw = euler_from_quaternion(orientation_list)

        # get the coordinates of the region around the robot
        region = set()
        for degs in range(0, 360, 10):
            for r in range(1,radius+1):
                x = grid_pos[0] + r * np.cos(current_yaw + np.radians(degs))
                y = grid_pos[1] + r * np.sin(current_yaw + np.radians(degs))
                region.add((x,y))

        return region

    def _get_grid(self):
        # populate the grid with the map data
        for point in point_cloud2.read_points(self.map):
            if point[2] > self.MIN_HEIGHT and point[2] < self.MAX_HEIGHT:
                px,py = point[:2]
                # convert the points to coordinates on the grid
                x = int((px - self.grid_map_origin[0]) / self.GRID_RESOLUTION+self.grid_origin[0])
                y = int((py - self.grid_map_origin[1]) / self.GRID_RESOLUTION+self.grid_origin[1])

                # set the grid value to 100 (occupied)
                self.grid_mat[y][x] = self.GridState.OCCUPIED

        # get position on grid
        x = int((self.odom.pose.pose.position.x - self.grid_map_origin[0]) / self.GRID_RESOLUTION)+self.grid_origin[0]
        y = int((self.odom.pose.pose.position.y - self.grid_map_origin[1]) / self.GRID_RESOLUTION)+self.grid_origin[0]
        
        region_around_robot = self._get_region_around_robot(50, (x,y))
        self._set_free_space(region_around_robot, (x,y))

        # create a line from the current position and orinetation to that point
        # if there is an obstacle before that point -> set all of the cells before that obstacle to FREE
        # else -> set all of the cells before that point to FREE

        # get the region around the robot
        # self.grid_mat[y][x] = self.GridState.VISITED
        orientation = self.odom.pose.pose.orientation
        print(self.grid_mat)
        print(self.grid_mat.shape)
        return {
            'data': self.grid_mat,
            'position': (x, y),
            'orientation': euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        }
    
    def _get_region_around_robot(self, radius, grid_pos):
        ''' Returns a set of coordinates around the robot'''
        # get current orientation
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, current_yaw = euler_from_quaternion(orientation_list)

        # get the coordinates of the region around the robot
        region = set()
        for degs in range(-60, 60, 5):
            for r in range(1,radius+1):
                x = grid_pos[0] + r * np.cos(current_yaw + np.radians(degs))
                y = grid_pos[1] + r * np.sin(current_yaw + np.radians(degs))
                region.add((int(x),int(y)))

        return region
    
    def _set_free_space(self, region_around_robot, grid_pos):
        '''Sets the free position on the grid'''
        # loop over the region around the robot
        # loop over the grid position linearly till the line between the robot and the grid position
        # if there is an obstacle -> set all of the cells before that obstacle to FREE
        # else -> set all of the cells before that point to FREE
        for x,y in region_around_robot:
            # get the radial angle between the current position and the point

            # get the line from the current position to the point
            line = bresenham(grid_pos[0], grid_pos[1], x, y)

            # loop over the line
            for i in range(len(line)):
                # get the current cell
                cell = line[i]
                # check if the cell is occupied
                if self.grid_mat[cell[1]][cell[0]] == self.GridState.OCCUPIED:
                    break
                else:
                    # set the cell to FREE
                    self.grid_mat[cell[1]][cell[0]] = self.GridState.FREE

    def _publish_grid(self):
        # get the grid
        res = self._get_grid()
        self.position_in_grid = res['position']
        self.orientation_in_grid = res['orientation']

        # create occupancy grid message
        grid_msg = OccupancyGrid()
        grid_msg.info.resolution = self.GRID_RESOLUTION
        grid_msg.info.width = res['data'].shape[1]
        grid_msg.info.height = res['data'].shape[0]
        grid_msg.info.origin.position.x = -110
        grid_msg.info.origin.position.y = -110
        grid_msg.info.origin.position.z = 0
        # grid_msg.info.origin.orientation.w = 1
        grid_msg.data = res['data'].flatten().astype(int).tolist()

        # publish the grid
        self.grid = grid_msg
        self.occupany_grid_pub.publish(grid_msg)


if __name__ == '__main__':
    oc_manager = GridManager(700,700)
    oc_manager.run()
    print('a7eeh')