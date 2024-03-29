# Robotics

ROS code written in COSC 081 21F. Descriptions of each file below.

# Random Walk

This program executes a random walk for a robot in an arbitrary environment. The robot moves forward with constant velocity in a straight line until it encounters some object within a particular threshold distance. At this point, the robot rotates to a random angle, doing so until there is no longer an object within the threshold distance. Then, it proceeds to move forward once more. The robot will move as such for an indefinite amount of time.

The specific parts of the program that I coded were (a) identifying whether an object was within the robot’s minimum threshold distance and (b) having the robot move forwards without an object in its path and rotating when there is. To identify whether there is an object in the robot’s path, I first took the array of scans and found the indices within that array at which our robot’s parameterized minimum and maximum scan angles are located. It is inside this slice of the array that our robot can actually see. So, within this slice, we need to know if there is an object or not. I located the closest object by taking the minimum of that slice. I made the min/max scan angles 45 degrees in each direction to allow for a wider field of vision. If that minimum is less than our minimum threshold distance, then I set our boolean for ‘close object’ to true. I made the minimum threshold distance 0.5 meters so the robot would have enough foresight to rotate appropriately if indeed there was an object within that range.

If there was no close object in the robot’s path, I had the robot move forward at a constant velocity of 0.5 m/s. This velocity allowed me to watch the robot physically move in the simulation without moving too quickly such that it got too close to an object before the scanner could catch it. If there was a close object, however, I had the robot rotate to a random angle and move at a constant
angular velocity of pi for however long it takes to reach the desired angle. I made the angular velocity greater so there was a better chance of the robot turning away from an object in fewer tries. If the angle was between -pi and 0, I had the robot rotate counterclockwise, and if it was between 0 and pi, I had it rotate clockwise. After doing so, I set the boolean for ‘close object’ to false.

# Polygon Problem

This program directs a robot to follow the path of various shapes. The first subproblem to solve was building a trapezoid. Given a radius and angle, my robot is able to draw a corresponding trapezoid. It does so by calculating the coordinates of the trapezoid vertices using right triangle geometry. But how does the robot actually reach those coordinates? My program routinely checks the robot’s current location and angle of orientation against its desired location and continues to move within a particular margin of error until it reaches a given point at a given angle. Then, it selects the next point and executes the same function.

To draw a ‘D’, however, was slightly more challenging because it invoked a curve. Instead of reducing the error between its desired location/angle and its current location/angle, to draw a curve, my program calculates the time it will need to move before reaching the end of an arc. It moves at a linear velocity equal to the angular velocity times the radius to achieve this.

Lastly, my program can accept a polyline of coordinates which it will use to draw the specified polygon. Using the same logic from the trapezoid problem, it works to reduce the error of its orientation and position. The program is also able to transform those global coordinates in the local reference frame using a transformation matrix consisting of a rotational matrix around the z-axis and a translation matrix along the x and y axes.

# Wall Fallower

My program controls a robot by keeping it within a certain threshold distance from a wall on its right. In this way, the robot can maneuver a space without ever deviating too far from the wall. Even when faced with complex terrain, the robot remains along the wall. The key computational feature behind this behavior is a PD controller – a controller that controls for proportional and derivative gain.

Let's delve into the specifics of the PD controller. The proportional gain term decreases rise time, increases the overshoot, drives a small change in the settling time, and decreases the steady-state error. This, on its own, would sway the robot's behavior, not controlling it for past error. This is where the derivative term comes in. The derivate term helps offset the proportional gain term, driving a small change in rise time, decreasing the overshoot, decreasing the settling time, and causing no change in the steady-state error.

Once we have the PD contoller, it's easy to implement the rest of the code to modify the robot's behavior when it's not moving forwards. In particular, we need to enable some sort of rotation such that the robot can avoid obstacles directly ahead of it, in which case the robot needs to rotate left around the obstacle to remain on the right side of the wall. Speaking of which, how does the robot keep on the right side of the wall using the PD controller? By taking a scan angle of 90 degrees to the right of the robot to 0 degrees directly in front of it, we can calculate the error between the closest object in that range and our minimum threshold distance. We can do the same for scans between -45 degrees and 45 degrees directly in front of the robot to check if there is an obstacle ahead.

Clearly, there are lots of different states that the robot can find itself in, so I implemented a finite state machine to switch between those states. The robot begins in a WALL state, moving forwards without angular velocity until it comes within the threshold distance of the wall. Then the robot enters a ROTATE state and rotates until there is no object within its field of view ahead. Once its field of view is cleared, it moves forward with angular velocity as determined by the PD controller. It continues to oscillate between ROTATE and FORWARD states as necessary.

# Path Planner

My program reads an occupancy grid and searches for a plan for a robot navigating that environment from an arbitrary start location to a goal. The program also visualizes that path using rviz and returns it incrementally to the user's terminal.

The program begins by prompting the user for a start and goal location of their choice. It wraps each point in a point object, which stores the point's coordinates, parent, and pose. The program then builds a search tree by creating a node for every point one unit away from its neighbor in all four directions, unless the coordinates of that node are occupied according to the occupancy map. I searched this tree using breadth-first search, which explores all nodes at a particular depth before proceeding to the next level. Once the algorithm reaches the goal location, it backtracks to the root and stores the path.

I then calculated the robot's pose at each step in its path based on its quaternion rotation. I also scaled the robot's coordinates down from the global reference frame to the map reference frame. Upon doing so, I published the robot's pose and created a marker for each coordinate in rviz. The start and goal locations are represented by green spheres; the individual steps in the robot's path are represented by red arrows.

# Occupancy Grid

My objective in this program was primarily to write algorithms that operate the RosBot. I did this in two parts: (1) estimating the robot's state using LIDAR and (2) mapping an occupancy grid of the robot's surrounding environment. These two programs are described below.

To estimate the robot's state, I first programmed the robot to move in three different ways: (a) translation, (b) relative rotation, and (c) absolute rotation. The code for translation and relative rotation was relatively straightforward as all I had to do was adapt it from past programming assignments. Programming an absolute rotation, however, was slightly more complex because I had to capture the robot's pose in the odom from, then subtract its yaw from the desired angle. After doing so, however, the robot rotated properly.

When using the RosBot, I was faced with translation and rotation errors as a result of the physical environment that I did not face when running these programs on simulations. So, it made sense to measure the error of my translations and rotations, specifically between the desired angle/distance, the lidar-measured angle/distance, and the manually measured angle/distance. I also programmed the robot to make a square by oscillating between translations and 90˚ absolute rotations.

I was then tasked with building an occupancy grid of the robot's surrounding environment. The Occupancy Grid class is initialized with a users preferred grid dimensions and resolution. To fill in the grid, I retrieved the robot's laser scan at each angle and the distance of the perceived object at that angle. I then used the Bresenham algorithm to calculate which grid spaces we can classify as unoccupied given the straight line distance between the robot and a perceived object. Before adding any point to the occupancy grid, I translated it from the laser reference frame to the odom reference frame so it would fit accordingly in my occupancy grid. Finally, I marked a point as occupied at the translated distance from the laser scan reading
