import rospy

from nav_msgs.msg import OccupancyGrid, Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PointStamped

class LocalPlanner:
    def __init__(self,position_on_grid, min_x_in_3d, min_y_in_3d, grid):
        self.grid = grid
        self.position_on_grid = position_on_grid
        self.min_x_in_3d = min_x_in_3d
        self.min_y_in_3d = min_y_in_3d
        self.odom = None


    def start(self):
        pass

    def plan(self):
        # get the current position and orientation in the grid
        x,y = self.position_on_grid
        orientation = self.odom.pose.pose.orientation

        

    def set_odom(self, odom):
        self.odom = odom

    def _prepare_message(self):
        pass

    def _publish_goal(self):
        pass

    def _grid_callback(self, msg):
        pass

    def _odom_callback(self, msg):
        pass

    def _get_position_in_3d(self, position_in_grid, orinetaion):
        x = position_in_grid[0]*self.grid.info.resolution + self.min_x_in_3d
        y = position_in_grid[1]*self.grid.info.resolution + self.min_y_in_3d

        return x,y,orinetaion

if __name__ == "__main__":
    pass