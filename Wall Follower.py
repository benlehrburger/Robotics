import math
import rospy
import numpy as np
from enum import Enum
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Set constants
LINEAR_VELOCITY = 0.4 # m/s
ANGULAR_VELOCITY = math.pi/16 # rad/s
FREQUENCY = 10 #Hz
WALL_DISTANCE = 0.5 # m
BUFFER = 0.1 # m

# Set controller constants
KP = 1
KD = 1
K = 0

# Initialize topics
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan'
DEFAULT_ODOM_TOPIC = '/odom'

# Wrap a finite state machine object
class fsm(Enum):
    WALL = 1
    ROTATE = 2
    FORWARD = 3

# Wrap a wall following robot object
class FollowWall:

    def __init__(self):
        
        # Initialize publisher
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Initialize subscriber
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        
        # Get the scan ahead of the robot and to its right
        self.range = None
        self.right = None
        self.ahead = None
        self.time_series = []
    
        # Initialize starting state
        self.start = True
    
        # Initialize finite state machine
        self._fsm = None

    # PD controller to moderate angular velocity
    def controller(self):
        # Current distance
        dt = 0
        # Last iteration's distance
        dt1 = 0
        
        # If there are more than two iterations of distance saved
        if len(self.time_series) >= 2:
            # Current distance
            dt = self.time_series[-1]
            # Last iteration's distance
            dt1 = self.time_series[-2]
        
        # If there is only one iteration of distance
        elif len(self.time_series) == 1:
            # Current distance
            dt = self.time_series[0]
            # Last iteration's distance
            dt1 = 0
        
        # Current error
        err1 = abs(WALL_DISTANCE - dt)
        # Last iteration's error
        err2 = abs(WALL_DISTANCE - dt1)

        # Derivative of error
        edot = (err1 - err2) / (1.0 / FREQUENCY)
        
        # Return PD controller
        return (K + KP * err1 + KD * edot)
    
    # Initialize laser callback
    def _laser_callback(self, msg):
        # Min angle width on the right side of the robot
        min_right_angle = -90.0 / 180 * math.pi
        # Max angle width on the right side of the robot
        max_right_angle = 0

        # Min angle width ahead of the robot
        min_ahead_angle = -45.0 / 180 * math.pi
        # Max angle width ahead of the robot
        max_ahead_angle = 45.0 / 180 * math.pi
        
        # Get all laser scans
        self.range = msg.ranges
        # Get length of laser scan array
        length = int((msg.angle_max - msg.angle_min) // msg.angle_increment)
        
        # Find right side lower bound
        right_lower_bound = int(length // 2 + min_right_angle // msg.angle_increment)
        # Find right side upper bound
        right_upper_bound = int(length // 2 + max_right_angle // msg.angle_increment)
        # Find forward lower bound
        ahead_lower_bound = int(length // 2 + min_ahead_angle // msg.angle_increment)
        # Find forward upper bound
        ahead_upper_bound = int(length // 2 + max_ahead_angle // msg.angle_increment)
        
        # Get range of scans in our field of view ahead
        ahead = self.range[ahead_lower_bound:ahead_upper_bound+1]
        # Get range of scans in out field of view to the right
        right = self.range[right_lower_bound:right_upper_bound+1]
        
        # Get closest object ahead of the robot
        self.ahead = min(ahead)
        # Get closest object to the right of the robot
        self.right = min(right)
        # Append closest object to the right to our error tracker
        self.time_series.append(self.right)
        
        # If we're save more than this iteration and the last iteration's error
        if len(self.time_series) >= 2:
            # Delete the third iteration's error
            del self.time_series[0]

        # If there's an object ahead within our threshold
        if self.ahead <= WALL_DISTANCE + BUFFER:
            # We're no longer in the starting state
            self.start = False
            # We want to rotate to the left away from that object
            self._fsm = fsm.ROTATE
        
        # If we are in the starting state
        elif self.start:
            # Move forwards towards a wall
            self._fsm = fsm.WALL
        
        # Otherwise move forwards controlled by the PD controller
        else:
            self._fsm = fsm.FORWARD

    # Boilerplate code to move the robot with linear and angular velocity
    def move(self, linear_vel, angular_vel):
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)
    
    # Boilerplate code to stop the robot from moving
    def stop(self):
        
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    # Commands to follow the wall within our threshold distance
    def follow(self):
        
        # Loop at 10 hz
        rate = rospy.Rate(FREQUENCY)
        
        # While the machine is not shut down
        while not rospy.is_shutdown():
            
            # If our current state is moving towards the wall
            if self._fsm == fsm.WALL:
                print('moving to wall')
                # Move forwards at constant linear velocity
                self.move(LINEAR_VELOCITY, 0)
            
            # If our current state is moving forwards
            if self._fsm == fsm.FORWARD:
                print('moving forward')
                # Control the robot's movement using the PD controller
                self.move(LINEAR_VELOCITY, self.controller())
            
            # If our current state is rotating
            if self._fsm == fsm.ROTATE:
                print('rotating')
                # Rotate left at constant angular velocity
                self.move(0, ANGULAR_VELOCITY)

# Boilerplate main function
def main():
    
    rospy.init_node("follow_wall")
    rospy.sleep(2)
    follow_wall = FollowWall()
    rospy.on_shutdown(follow_wall.stop)

    try:
        follow_wall.follow()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    main()
