# Ben Lehrburger
# COSC 081 PA1

import math
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Set constants
LINEAR_VELOCITY = 0.8 # m/s
ANGULAR_VELOCITY = math.pi/16 # rad/s
FREQUENCY = 10 #Hz
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = '/odom'

# Wrap a polygon problem object
class PolygonProblem:

    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
    
        self.polyline = None
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        
        self.x = None
        self.y = None
        self.yaw = None
        self.t = None
        
        # Initialize publisher
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Initialize subscriber
        self._odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self.odom_callback)
    
    # Get information about the local reference frame from Odom
    def odom_callback(self, msg):
        odom = msg.pose.pose.position
        # Capture rotational geometry
        self.orientation = msg.pose.pose.orientation
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        
        self.x = odom.x
        self.y = odom.y
    
    # Stop the robot from moving and publish
    def stop(self):
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)
    
    # Move the robot with linear and angular velocity
    def move(self, linear_vel, angular_vel):
        twist_msg = Twist()
        
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        # Publish move command
        self._cmd_pub.publish(twist_msg)
        self.stop()
    
    # Have the robot create a curve for the D problem
    def curve(self, linear_vel, angular_vel, theta):
        self.stop()
        start_time = rospy.get_rostime()
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)
        # Calculate the runtime needed for the curve at a given angle
        run_time = abs(theta) / angular_vel
        # Until that runtime has elapsed
        while rospy.get_rostime() - start_time <= rospy.Duration(run_time):
            twist_msg.linear.x = linear_vel
            if theta < 0:
                twist_msg.angular.z = angular_vel
            else:
                twist_msg.angular.z = -angular_vel
            # Publish the move commands
            self._cmd_pub.publish(twist_msg)
        self.stop()
    
    # Get input from user about what problem they want to solve
    def get_problem_input(self):
        input = raw_input("What problem would you like to solve? Type 't' for trapezoid, 'd' for D-shape, or 'p' for polygon: ")
        return input

    # Get input from user about what radius they want to use
    def get_radius_input(self):
        input = raw_input("Give a radius: ")
        return float(input)
    
    # Get input from user about trapezoid angle
    def get_angle_input(self):
        input = raw_input("Give an angle in degrees: ")
        return math.radians(float(input))
    
    # Get input from user about polygon coordinates
    def get_polygon_input(self):
        input = raw_input("Insert coordinates of your polygon: ")
        return list(eval(input))

    # Check if the robot is at a particular coordinate
    def at_point(self, point):
        x, y = point[0], point[1]
        # It's close enough if it's within .2 meters
        if abs(x - self.x) < .2 and abs(y - self.y) <= .2:
            self.stop()
            return True
        else:
            return False

    # Check if the robot is at a particular angle
    def at_angle(self, angle):
        # It's close enough if it is within .03 radians of the angle
        if abs(self.yaw - angle) <= .03:
            self.stop()
            return True
        else:
            return False

    # Get the angle at which we want to turn
    def get_theta(self, p1):
        x1, y1 = p1[0], p1[1]

        # Angle is tangent to the right triangle created by its intersection
        theta = math.atan2(y1-self.y,x1-self.x)
        
        return theta
    
    # Make one rotation and one movement at a time
    def make_junction(self, point):
        # While the robot is not shut down
        while not rospy.is_shutdown():
            # Rotate if it's not at the desired angle
            if not self.at_angle(self.get_theta(point)):
                self.move(0, self.angular_velocity)
            # Move linearly if it's not at the desired point
            else:
                self.move(self.linear_velocity, 0)
            if self.at_point(point):
                break
    
    def transform(self, vector):
        # Initialize transformation matrix
        p1, p2 = self.polyline[0], self.polyline[1]
        theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
        # Rotation about the Z axis plus x-y translation
        self.t = np.array([[math.cos(theta), -math.sin(theta), 0, self.x - p1[0]],
                           [math.sin(theta), math.cos(theta), 0, self.y - p1[1]],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
                
        # Take dot product of each vector and the transformation matrix
        tv = self.t.dot(vector)
        
        return (tv[0], tv[1])
    
    # Make a trapezoid
    def make_trapezoid(self):
        
        radius = self.get_radius_input()
        angle = self.get_angle_input()
        
        # Drive up the radius
        self.make_junction((0, radius))
        # First junction
        self.make_junction((radius*math.sin(angle), radius*math.cos(angle)))
        # Second junction
        self.make_junction((radius*math.sin(angle), -radius*math.cos(angle)))
        # Drive back up the radius
        self.make_junction((0, -radius))
        # Return to origin
        self.make_junction((0, 0))
    
    # Make a D-shape
    def make_d(self):
    
        radius = self.get_radius_input()
        
        # Drive up the radius
        self.make_junction((0, radius))
        # Rotate 90˚
        self.curve(0, self.angular_velocity, math.pi/2)
        # Rotate at 180˚ with proportional linear velocity
        self.curve(self.angular_velocity*radius, self.angular_velocity, math.pi)
        # Return to origin
        self.make_junction((0, 0))
    
    def make_polygon(self):
        
        self.polyline = self.get_polygon_input()
        
        # Make vectors for each point in the polyline for the transformation matrix
        transformed = []
        for point in self.polyline:
            v = np.array((point[0], point[1], 0, 1))
            transformed.append(self.transform(v))
        
        print("Transformation matrix: ")
        print(self.t)
        print("Transformed coordinates: ")
        print(transformed)
        
        self.polyline.append(self.polyline[0])
        
        # Make a junction for each point in the polyline
        for point in self.polyline:
            self.make_junction(point)

    # Run the program
    def execute(self):
        problem = self.get_problem_input()
        
        # If user input is trapezoid problem
        if problem == 't':
            self.make_trapezoid()
        # If user input is D problem
        elif problem == 'd':
            self.make_d()
        # If user input is polygon problem
        elif problem == 'p':
            self.make_polygon()
        else:
            print('Please input either t, d, or p')
            self.execute()

def main():
    
    rospy.init_node('polygon_problem')
    polygon_problem = PolygonProblem()
    rospy.sleep(1)
    rospy.on_shutdown(polygon_problem.stop)
    
    # Robot builds shape
    try:
        polygon_problem.execute()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    main()
