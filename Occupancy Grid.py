# Ben Lehrburger
# COSC 081 PA4

# ** DEPENDENCIES **
import math
import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg

# ** TOPICS **
DEFAULT_SCAN_TOPIC = 'base_scan'
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_OCCUPANCY_TOPIC = 'map'
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'

# ** CONSTANTS **
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = math.pi / 8

# Wrap an Occupancy Grid object
class OccupancyGrid:

    # Accept grid width, height, and resolution
    def __init__(self, width, height, resolution):

        # Store the robot's x location in the odom frame
        self.x = None
        # Store the robot's y location in the odom frame
        self.y = None
        # Store the robot's yaw relative to the odom frame
        self.yaw = None
        
        # Store the width of the grid
        self.width = width
        # Store the height of the grid
        self.height = height
        # Store the resolution of the grid
        self.resolution = resolution
        
        # Set the x origin at the center of the grid
        self.origin_x = int(-self.width/2)
        # Set the y origin at the center of the grid
        self.origin_y = int(-self.height/2)
    
        # Initialize the grid as an empty matrix with dimensions W x H
        self.grid = np.zeros((int(self.width), int(self.height)))
        
        # ** SUBSCRIBERS **
        
        # Initialize odom subscriber
        self._odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)
        # Initialize laser subscriber
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        
        # ** PUBLISHERS **
        
        # Initialize twist publisher
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Initialize occupancy grid publisher
        self._occupancy_pub = rospy.Publisher(DEFAULT_OCCUPANCY_TOPIC, OccupancyGridMsg, queue_size=1)

    # ** CALLBACKS **
    
    # Get information about the local reference frame from odom
    def _odom_callback(self, msg):
        
        # Get the robot's pose in the odom frame
        odom = msg.pose.pose.position
        # Capture rotational geometry
        self.orientation = msg.pose.pose.orientation
        # Convert from quaternion to euler
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        
        # Store x location
        self.x = odom.x
        # Store y location
        self.y = odom.y
    
    # Retrieve and process laser readings from laser sensor
    def _laser_callback(self, msg):
        
        # For the laser reading at each angle
        for i in range(len(msg.ranges)):
            
            # Calculate the angle of that reading relative to the robot
            angle = i * msg.angle_increment + msg.angle_min
            # Calculate the distance of the object from the robot
            distance = msg.ranges[i]
            
            # Add those readings to the occupancy grid
            self.populate_grid(angle, distance)
        
        # Publish the grid each iteration of laser scans
        self.grid_publisher()
    
    # ** MAIN FUNCTIONS **
    
    # Initialize occupancy grid callback
    def grid_publisher(self):
        
        # Initialize map
        grid = OccupancyGridMsg()
        
        # Initialize header
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = 'odom'
        
        # Pass grid dimensions and structure
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.resolution = self.resolution
        grid.info.origin = Pose(Point(self.origin_x, self.origin_y, 0), Quaternion(0, 0, 0, 1))
        
        # Padd grid data
        grid.data = list(self.grid.flatten())
    
        # Publish grid to map topic
        self._occupancy_pub.publish(grid)
    
    # Process laser data and determine its implications for the grid
    def populate_grid(self, angle, distance):
        
        # Current x-y
        x1 = self.x
        y1 = self.y
        
        # x-y laser reading
        x2 = int(self.x + distance * math.cos(angle + self.yaw) / self.resolution)
        y2 = int(self.y + distance * math.sin(angle + self.yaw) / self.resolution)
    
        # Calculate open grid spots with Bresenham trig
        self.bresenham(x1, y1, x2, y2)

        # Transform the x-y laser reading in the odom frame
        transformed_point = self.transform(x2, y2)
        tx = int(transformed_point[0])
        ty = int(transformed_point[1])

        # Add x-y laser reading to the grid marked as occupied if it fits within the grid dimensions
        if abs(tx) <= self.width and abs(ty) <= self.height:
            self.grid[ty, tx] = 100

    # Transform points from map to odom reference frame
    def transform(self, x, y):

        # Translation to/from the origin
        transformation_matrix = np.matrix(([1, 0, self.origin_x],
                                           [0, 1, self.origin_y],
                                           [0, 0, 1]))

        # Point to be translated in vector form
        point = np.matrix(([x],
                          [y],
                          [0]))

        # Compute new point with matrix multiplication
        new_point = transformation_matrix * point
        
        # Scale new x-y coordinate to fit the grid
        tx = new_point.item(0) + self.width / self.resolution / 2
        ty = new_point.item(1) + self.height / self.resolution / 2
        grid_x = int(round(tx * self.resolution))
        grid_y = int(round(ty * self.resolution))

        # Return translated coordinates
        return (grid_x, grid_y)

    # Adapted from: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    # Determine which grid coordinates should be considered unoccupied in a straight line reading
    def bresenham(self, x1, y1, x2, y2):
        
        if abs(y2 - y1) < abs(x2 - x1):
            
            # Gradient of y is less than 0
            if x1 > x2:
                self.decrease_y(x2, y2, x1, y1)
            else:
                self.decrease_y(x1, y1, x2, y2)
    
        else:
            
            # Gradient of y is greater than 0
            if y1 > y2:
                self.increase_y(x2, y2, x1, y1)
            else:
                self.increase_y(x1, y1, x2, y2)

    # When the gradient of y is less than 0
    def decrease_y(self, x1, y1, x2, y2):
        
        dx = x2 - x1
        dy = y2 - y1
        yi = 1
        
        if dy < 0:
            yi = -1
            dy = -dy

        D = (2 * dy) - dx
        y = y1

        # For each x between the current point and the laser reading
        for x in range(int(x1), int(x2 + 1)):

            transformed_point = self.transform(x, y)
            tx = int(transformed_point[0])
            ty = int(transformed_point[1])
            
            # Add x-y laser reading to the grid marked as an unoccupied if it fits within the grid dimensions
            if abs(tx) <= self.width and abs(ty) <= self.height:
                self.grid[ty, tx] = 0

            if D > 0:
                y += yi
                D += (2 * (dy - dx))
            else:
                D += (2 * dy)

    # When the gradient of y is greater than 0
    def increase_y(self, x1, y1, x2, y2):
        
        dx = x2 - x1
        dy = y2 - y1
        xi = 1
        
        if dx < 0:
            xi = -1
            dx = -dx

        D = (2 * dx) - dy
        x = x1

        # For each y between the current point and the laser reading
        for y in range(int(y1), int(y2 + 1)):

            transformed_point = self.transform(x, y)
            tx = int(transformed_point[0])
            ty = int(transformed_point[1])
            
            # Add x-y laser reading to the grid marked as an unoccupied if it fits within the grid dimensions
            if abs(tx) <= self.width and abs(ty) <= self.height:
                self.grid[ty, tx] = 0

            if D > 0:
                x += xi
                D += (2 * (dx - dy))
            else:
                D += (2 * dx)
    
    # ** BOILERPLATE MOVEMENT FUNCTIONS **
    
    # Move the robot with linear and angular velocity
    def move(self, linear_vel, angular_vel):
        
        twist_msg = Twist()
        
        # Set linear and angular velocities
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        
        # Publish move command
        self._cmd_pub.publish(twist_msg)
        self.stop()
    
    # Stop the robot from moving and publish
    def stop(self):
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    # Move the robot in a straight line
    def translate(self, double_d):
        
        rospy.sleep(1)
        
        # Get start time
        start_time = rospy.get_rostime()
        
        # Move for specified runtime at constant linear velocity
        while not rospy.is_shutdown():
            
            # Calculate how long the robot needs to move
            run_time = abs(double_d) / LINEAR_VELOCITY
            
            # Move linearly until the runtime expires
            if rospy.get_rostime() - start_time <= rospy.Duration(run_time):
                self.move(LINEAR_VELOCITY, 0)
            
            # break and sleep once runtime expires
            else:
                break
            self.rate.sleep()
        
        rospy.sleep(1)
            
    # Rotate the robot relative to its current pose
    def rotate_rel(self, double_angle):
        
        rospy.sleep(1)
        
        # Get start time
        start_time = rospy.get_rostime()
        
        # Store the laser scan before rotating
        before = self.whats_ahead
        
        # Rotate for specified runtime at constant angular velocity
        while not rospy.is_shutdown():
            
            # Calculate how long the robot needs to rotate for
            run_time = abs(double_angle) / ANGULAR_VELOCITY
            
            # Rotate until the runtime expires
            if rospy.get_rostime() - start_time <= rospy.Duration(run_time):
                if double_angle < 0:
                    self.move(0, ANGULAR_VELOCITY)
                else:
                    self.move(0, -ANGULAR_VELOCITY)
        
            # stop, break, and sleep once the time expires
            else:
                self.stop()
                break
            self.rate.sleep()

        rospy.sleep(1)

    # Test the robot's mapping with movement
    def test(self):
        
        rospy.sleep(5)
        
        # Test translate
        self.translate(3)
        rospy.sleep(2)
    
        # Test rotate
        self.rotate_rel(math.pi/2)
        rospy.sleep(2)

        rospy.spin()


# ** BUILD OCCUPANCY GRID OBJECT **

# Boilerplate main function
def main():
    rospy.sleep(2)
    
    # Initialize occupancy grid node with arbitrary dimensions and resolution
    rospy.init_node('occupancy_grid')
    occupancy_grid = OccupancyGrid(30, 30, 0.5)
    
    rospy.sleep(1)
    rospy.on_shutdown(occupancy_grid.stop)
    
    # Run the main function
    try:
        occupancy_grid.test()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    main()
