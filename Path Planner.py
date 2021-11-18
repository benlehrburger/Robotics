# Ben Lehrburger
# COSC 081 PA3

# This program uses BFS to make a plan for a robot with a known occupancy grid and visualizes the path

# Dependencies
import rospy
import math
import numpy
import tf
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from collections import deque

# Constants

# Scale between global and map reference frames
SCALE = float(20)
# Frequency with which the loop operates (Hz)
FREQUENCY = 100


# Wrap a grid object
class Grid:
    
    def __init__(self, occupancy_grid_data, width, height, resolution):
        
        # Store occupancy grid
        self.grid = numpy.reshape(occupancy_grid_data, (height, width))
        # Store map resolution
        self.resolution = resolution

    # Getter method to find occupancy status of a particular cell
    def cell_at(self, x, y):
        return self.grid[y, x]


# Wrap a point object
class Pose:
    
    def __init__(self, coordinate, parent, pose=None):
        
        # Store the point's coordinates
        self.coordinate = coordinate
        self.x = coordinate[0]
        self.y = coordinate[1]
        # Store the point's parent
        self.parent = parent
        # Store the point's orientation
        self.pose = pose


# Wrap a plan object
class Plan:

    def __init__(self):

        # Store the map
        self.map = None
        # Store the starting coordinates
        self.start = None
        # Store the goal coordinates
        self.goal = None
        # Store the path
        self.path = None
        # Store the resolution
        self.resolution = None

        # Subscribe to the occupancy grid topic
        self.sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        
        # Publish to the markers topic
        self.marker_pub = rospy.Publisher("markers", Marker, queue_size = 100)

        # Publish to the pose topic
        self.pose_pub = rospy.Publisher("pose_sequence", PoseStamped, queue_size = 100)

    # Callback function to get map data
    def map_callback(self, msg):
        
        # Store the resolution
        self.resolution = msg.info.resolution
        # Store the map in a grid object
        self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)

    # Get starting coordinates from the user
    def get_start(self):
        
        # Prompt user for input
        input = raw_input("Give a starting location x,y: ")
        
        # Parse the input string
        x = int(float(input.split(",")[0]) * (1 / self.resolution)) + 1
        y = int(float(input.split(",")[1]) * (1 / self.resolution)) + 1
        
        # Return a point object for the starting location
        return Pose((x,y), "start")

    # Get goal coordinates from the user
    def get_goal(self):
        
        # Prompt user for input
        input = raw_input("Give a goal location x,y: ")
        
        # Parse the input string
        x = int(float(input.split(",")[0]) * (1 / self.resolution)) + 1
        y = int(float(input.split(",")[1]) * (1 / self.resolution)) + 1
        
        # Return a point object for the goal location
        return Pose((x, y), "goal")

    # backtrack from the current node to the root and return a list
    def backchain(self, node):
    
        # initialize new path
        path = []
        # add the current node to path
        path.append(node)
        # set an intermediate variable to point to each current node
        curr = node
        
        # until we reach the root
        while curr.parent is not "start":
            # grab the current node's parent
            curr = curr.parent
            # insert that node at the front of our path
            path.insert(0, curr)
        
        # return the path as a list
        return path

    # Conduct a breadth first search
    def bfs(self):

        # Initialize a queue to hold nodes on the frontier
        frontier = deque()
        # Initialize a set to track visited nodes
        visited = set()
        
        # Add the starting node to the tree at the root
        frontier.append(self.start)

        # While the queue is not empty
        while frontier:

            # Pop the foremost node in the queue
            current_node = frontier.popleft()
            # Add that node to the visited set
            visited.add(current_node)
            
            # Stop iterating through the frontier queue it we've reached our goal state
            if current_node.coordinate == self.goal.coordinate:
                break

            # Retrieve the current x and y coordinates
            x, y = current_node.x, current_node.y

            # If we're within the rightmost boundary
            if x+1 < 200:
                # If the cell to the right is not occupied
                if self.map.cell_at(x+1, y) != 100:
                    # Create a new point for that cell
                    right = Pose((x+1, y), current_node)
                    # If we haven't visited that node yet
                    if right.coordinate not in visited:
                        # Add it to the frontier queue and visited set
                        frontier.append(right)
                        visited.add(right.coordinate)

            # If we're within the topmost boundary
            if y+1 < 200:
                # If the cell above is not occupied
                if self.map.cell_at(x, y+1) != 100:
                    # Create a new point for that cell
                    up = Pose((x, y+1), current_node)
                    # If we haven't visited that node yet
                    if up.coordinate not in visited:
                        # Add it to the frontier queue and visited set
                        frontier.append(up)
                        visited.add(up.coordinate)

            # If we're within the leftmost boundary
            if x-1 > -200:
                # If the cell to the left is not occupied
                if self.map.cell_at(x-1, y) != 100:
                    # Create a new point for that cell
                    down = Pose((x-1, y), current_node)
                    # If we haven't visited that node yet
                    if down.coordinate not in visited:
                        # Add it to the frontier queue and visited set
                        frontier.append(down)
                        visited.add(down.coordinate)

            # If we're within the bottommost boundary
            if y-1 > -200:
                # If the cell below is not occupied
                if self.map.cell_at(x, y-1) != 100:
                    # Create a new point for that cell
                    left = Pose((x, y-1), current_node)
                    # If we haven't visited that node yet
                    if left.coordinate not in visited:
                        # Add it to the frontier queue and visited set
                        frontier.append(left)
                        visited.add(left.coordinate)

        # Backtrack from the goal location to retrieve the path
        self.path = self.backchain(current_node)

    # Print the robot's current position
    def to_string(self):
        
        # For each position in the robot's path
        for index in range(0, len(self.path)):
            # Point to that node
            curr = self.path[index]
            # If it's not our goal location point to the next node
            if index != len(self.path) - 1:
                next = self.path[index+1]
                # Save pose for 0 degree turn
                if next.x > curr.x:
                    curr.pose = 0
                # Save pose for 180 degree turn
                if next.x < curr.x:
                    curr.pose = 180
                # Save pose for 90 degree turn
                if next.y > curr.y:
                    curr.pose = 90
                # Save pose for 270 degree turn
                if next.y < curr.y:
                    curr.pose = 270
            # Save pose for 0 degree turn
            else:
                curr.pose = 0

        # Print each pose in the robot's path to terminal
        for pose in self.path:
            print "(" + str(float(pose.x/SCALE)) + "," + str(float(pose.y/SCALE)) + "," + str(pose.pose) + ")"

    # Convert each pose from euler angle to quaternion rotation
    def to_quaternion(self, point, msg):
        
        # Conversion factor for 45-45-90 triangle
        QUATERNION_ROTATION = math.sqrt(2) / 2
        
        # Quaternion rotation at 0 degrees
        if point.pose == 0:
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 0
        # Quaternion rotation at 90 degrees
        if point.pose == 90:
            msg.pose.orientation.z = QUATERNION_ROTATION
            msg.pose.orientation.w = QUATERNION_ROTATION
        # Quaternion rotation at 180 degrees
        if point.pose == 180:
            msg.pose.orientation.z = QUATERNION_ROTATION
            msg.pose.orientation.w = 0
        # Quaternion rotation at 270 degrees
        if point.pose == 270:
            msg.pose.orientation.z = QUATERNION_ROTATION
            msg.pose.orientation.w = -QUATERNION_ROTATION
                
    # Publish the robot's pose to the pose_sequence topic
    # Uncommented lines are boilerplate codes from in class examples
    def publish_pose(self, point):
        
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "map"

        # Scale x position from global to map reference frame
        pose.pose.position.x = float(point.x)/SCALE
        # Scale y position from global to map reference frame
        pose.pose.position.y = float(point.y)/SCALE
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        
        # Convert point's orientation to quaternion rotation
        self.to_quaternion(point, pose)
        
        # Publish pose
        self.pose_pub.publish(pose)

    # Mark point using an arrow in rviz
    # Uncommented lines are boilerplate codes from in class examples
    def mark_point(self, point, id):
    
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "map"
        
        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.ARROW
        marker_msg.id = id
        
        # Scale x position from global to map reference frame
        marker_msg.pose.position.x = float(point.x)/SCALE
        # Scale y position from global to map reference frame
        marker_msg.pose.position.y = float(point.y)/SCALE
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        # Convert point's orientation to quaternion rotation
        self.to_quaternion(point, marker_msg)
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.scale.x = 0.25
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        
        # Publish point
        self.marker_pub.publish(marker_msg)
    
    # Mark start and goal locations using a sphere in rviz
    # Uncommented lines are boilerplate codes from in class examples
    def bookend(self, point, id):
        
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "map"
        
        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.SPHERE
        marker_msg.id = id
        
        # Scale x position from global to map reference frame
        marker_msg.pose.position.x = float(point.x)/SCALE
        # Scale y position from global to map reference frame
        marker_msg.pose.position.y = float(point.y)/SCALE
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.color.a = 1.0
        marker_msg.color.g = 1.0
        marker_msg.scale.x = 0.25
        marker_msg.scale.y = 0.25
        marker_msg.scale.z = 0.25

        # Publish point
        self.marker_pub.publish(marker_msg)

    # Main executable function
    def search_and_draw(self):
        
        # Set rate
        rate = rospy.Rate(FREQUENCY)
        # Get starting location
        self.start = self.get_start()
        # Get goal location
        self.goal = self.get_goal()
        
        # Conduct BFS search
        self.bfs()
        # Print path to terminal
        self.to_string()
    
        # Mark starting location
        self.bookend(self.start, 1)
        # Mark goal location
        self.bookend(self.goal, 2)
        
        # ID tracker
        counter = 3
        # For each point in the robot's path
        for point in self.path:
            # Visualize every other point to not overpopulate the screen
            if self.path.index(point) % 2 != 0:
                # Publish the point's pose
                self.publish_pose(point)
                # Draw the point in rviz
                self.mark_point(point, counter)
                # Increment the ID
                counter += 1
                rate.sleep()


# Boilerplate main function
def main():
    rospy.init_node("plan")
    rospy.sleep(1)
    plan = Plan()
    rospy.sleep(1)
    plan.search_and_draw()

if __name__ == "__main__":
    main()

