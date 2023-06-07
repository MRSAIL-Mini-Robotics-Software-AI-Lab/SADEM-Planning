#!/usr/bin/env python3

import rospy

import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from nav_msgs.msg import OccupancyGrid, Odometry

from geometry_msgs.msg import PointStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ExploratoryPlanner:
    # MAP_TOPIC = '/registered_scan'
    MAP_TOPIC = '/explored_areas'
    ODOM_TOPIC = '/state_estimation'
    GOAL_TOPIC = '/way_point'
    OCCUPANCY_TOPIC = '/occupancy_grid'
    GRID_RESOLUTION = .3
    DEFAULT_HEIGHT = 0.0
    MIN_HEIGHT = 0.7
    MAX_HEIGHT = 1

    class GridState:
        UNEXPLORED = -1
        FREE = 0
        VISITED = 50
        OCCUPIED = 100

    def __init__(self):
        rospy.init_node('exploratory_planner', anonymous=True)

        # subscribers 
        self.map_sub = rospy.Subscriber(self.MAP_TOPIC, PointCloud2, self._map_callback)
        self.odom_sub = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self._odom_callback)
        # publishers
        self.goal_pub = rospy.Publisher(self.GOAL_TOPIC, PointStamped, queue_size=10)
        self.occupany_grid_pub = rospy.Publisher(self.OCCUPANCY_TOPIC, OccupancyGrid, queue_size=10)

        # Iniitialize variables
        self.map = None
        self.odom = None
        self.grid = None
        self.position_in_grid = None
        self.orientation_in_grid = None
        self.path = []

        self.rate = rospy.Rate(5)
        self.rotation_rate = rospy.Rate(10)

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

        # print ofodometry data
        print('odom', msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

    def plan_and_publish_goal(self):
        if self.grid is not None and self.odom is not None:
            # Perform long-term planning based on the occupancy grid and odometry data
            self._plan_goal_position()

            # get goal
            goal = self.path.pop(0)

            # publish goal
            goal_msg = PointStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.point.x = goal[0]
            goal_msg.point.y = goal[1]
            goal_msg.point.z = self.DEFAULT_HEIGHT
            self.goal_pub.publish(goal_msg)

        else:
            rospy.logwarn('No occupancy grid or odometry data received')

    def _plan_goal_position(self):
        if self.grid is None:
            rospy.logwarn('No occupancy grid received')
            return
        pass

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
        # get map data
        min_x = min_y = float('inf')
        max_x = max_y = float('-inf')
        max_z = float('-inf')
        min_z = float('inf')
        # determine the min and max x and y values for the grid size
        for point in point_cloud2.read_points(self.map): 
            # only get points close to 0.9m and 1.1m in z
            if point[2] > self.MIN_HEIGHT and point[2] < self.MAX_HEIGHT:
                x,y = point[:2]
                min_x = min(min_x, x)
                min_y = min(min_y, y)
                max_x = max(max_x, x)
                max_y = max(max_y, y)
                # max_z = max(max_z, point[2])
                # min_z = min(min_z, point[2])

        # print('min max',min_z, max_z)
        # calculate the grid size
        grid_width = int((max_x - min_x) / self.GRID_RESOLUTION)+1
        grid_height = int((max_y - min_y) / self.GRID_RESOLUTION)+1

        # create occupanvy grid with all zeros
        grid = np.zeros((grid_height, grid_width))+self.GridState.UNEXPLORED

        # populate the grid with the map data
        for point in point_cloud2.read_points(self.map):
            if point[2] > self.MIN_HEIGHT and point[2] < self.MAX_HEIGHT:
                px,py = point[:2]
                # convert the points to coordinates on the grid
                x = int((px - min_x) / self.GRID_RESOLUTION)
                y = int((py - min_y) / self.GRID_RESOLUTION)

                # set the grid value to 100 (occupied)
                grid[y][x] = self.GridState.OCCUPIED

        # get position on grid
        x = int((self.odom.pose.pose.position.x - min_x) / self.GRID_RESOLUTION)
        y = int((self.odom.pose.pose.position.y - min_y) / self.GRID_RESOLUTION)
        # print(grid.shape)

        # set current position on grid to visited
        grid[y][x] = self.GridState.VISITED
        grid[y+1][x] = self.GridState.VISITED
        grid[y-1][x] = self.GridState.VISITED
        grid[y][x+1] = self.GridState.VISITED
        grid[y][x-1] = self.GridState.VISITED

        # get the 2d orientation
        orientation = self.odom.pose.pose.orientation

        return {
            'data': grid,
            'position': (x, y),
            'orientation': euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        }
    
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
        grid_msg.info.origin.position.x = 0
        grid_msg.info.origin.position.y = 0
        grid_msg.info.origin.position.z = 0
        # grid_msg.info.origin.orientation.w = 1
        grid_msg.data = res['data'].flatten().astype(int).tolist()

        # publish the grid
        self.grid = grid_msg
        self.occupany_grid_pub.publish(grid_msg)


if __name__ == '__main__':
    planner = ExploratoryPlanner()
    planner.run()