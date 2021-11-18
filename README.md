# Robotics

ROS code written in COSC 081 21F. Descriptions of each file below.

# Random Walk

### Description

The program that I completed for programming assignment 0 executes a random walk for a robot in an arbitrary environment. The robot moves forward with constant velocity in a straight line until it encounters some object within a particular threshold distance. At this point, the robot rotates to a random angle, doing so until there is no longer an object within the threshold distance. Then, it proceeds to move forward once more. The robot will move as such for an indefinite amount of time.

The specific parts of the program that I coded were (a) identifying whether an object was within the robot’s minimum threshold distance and (b) having the robot move forwards without an object in its path and rotating when there is. To identify whether there is an object in the robot’s path, I first took the array of scans and found the indices within that array at which our robot’s parameterized minimum and maximum scan angles are located. It is inside this slice of the array that our robot can actually see. So, within this slice, we need to know if there is an object or not. I located the closest object by taking the minimum of that slice. I made the min/max scan angles 45 degrees in each direction to allow for a wider field of vision. If that minimum is less than our minimum threshold distance, then I set our boolean for ‘close object’ to true. I made the minimum threshold distance 0.5 meters so the robot would have enough foresight to rotate appropriately if indeed there was an object within that range.

If there was no close object in the robot’s path, I had the robot move forward at a constant velocity of 0.5 m/s. This velocity allowed me to watch the robot physically move in the simulation without moving too quickly such that it got too close to an object before the scanner could catch it. If there was a close object, however, I had the robot rotate to a random angle and move at a constant
angular velocity of pi for however long it takes to reach the desired angle. I made the angular velocity greater so there was a better chance of the robot turning away from an object in fewer tries. If the angle was between -pi and 0, I had the robot rotate counterclockwise, and if it was between 0 and pi, I had it rotate clockwise. After doing so, I set the boolean for ‘close object’ to false.

### Evaluation

My program does work in that it satisfies all of the requirements of the project description. It successfully moves straight until encountering an object, at which point it rotates without making contact with that object. The only flaw in the program’s design – which was not a requirement of the assignment but is still inefficient – is that the robot wastes a lot of resources by using random angles. The robot has to ‘guess and check’ many times before finding an appropriate angle that won’t cause it to run into an object. Perhaps the program would work better if there were some way to inform the angle at which the robot rotates.

# Polygon Problem

### Description

PA1 asked us to program a robot to follow the path of various shapes. The first subproblem to solve was building a trapezoid. Given a radius and angle, my robot is able to draw a corresponding trapezoid. It does so by calculating the coordinates of the trapezoid vertices using right triangle geometry. But how does the robot actually reach those coordinates? My program routinely checks the robot’s current location and angle of orientation against its desired location and continues to move within a particular margin of error until it reaches a given point at a given angle. Then, it selects the next point and executes the same function.

To draw a ‘D’, however, was slightly more challenging because it invoked a curve. Instead of reducing the error between its desired location/angle and its current location/angle, to draw a curve, my program calculates the time it will need to move before reaching the end of an arc. It moves at a linear velocity equal to the angular velocity times the radius to achieve this.

Lastly, my program can accept a polyline of coordinates which it will use to draw the specified polygon. Using the same logic from the trapezoid problem, it works to reduce the error of its orientation and position. The program is also able to transform those global coordinates in the local reference frame using a transformation matrix consisting of a rotational matrix around the z-axis and a translation matrix along the x and y axes.

### Evaluation

My program is able to draw all of the shapes encapsulated in the problem space. However, it does not always do so perfectly. Because the program calculates the error between the current and desired positions and orientations within a particular margin of error, its movements are not always perfect. Thus, it may overrotate by a fraction of a radian or move too far or too short. So, the path the robot takes is not always smooth. That said, the robot is still able to draw all the necessary shapes even if its precision is not exact.

# Wall Fallower

### Description

My program controls a robot by keeping it within a certain threshold distance from a wall on its right. In this way, the robot can maneuver a space without ever deviating too far from the wall. Even when faced with complex terrain, the robot remains along the wall. The key computational feature behind this behavior is a PD controller – a controller that controls for proportional and derivative gain.

Let's delve into the specifics of the PD controller. The proportional gain term decreases rise time, increases the overshoot, drives a small change in the settling time, and decreases the steady-state error. This, on its own, would sway the robot's behavior, not controlling it for past error. This is where the derivative term comes in. The derivate term helps offset the proportional gain term, driving a small change in rise time, decreasing the overshoot, decreasing the settling time, and causing no change in the steady-state error.

Once we have the PD contoller, it's easy to implement the rest of the code to modify the robot's behavior when it's not moving forwards. In particular, we need to enable some sort of rotation such that the robot can avoid obstacles directly ahead of it, in which case the robot needs to rotate left around the obstacle to remain on the right side of the wall. Speaking of which, how does the robot keep on the right side of the wall using the PD controller? By taking a scan angle of 90 degrees to the right of the robot to 0 degrees directly in front of it, we can calculate the error between the closest object in that range and our minimum threshold distance. We can do the same for scans between -45 degrees and 45 degrees directly in front of the robot to check if there is an obstacle ahead.

Clearly, there are lots of different states that the robot can find itself in, so I implemented a finite state machine to switch between those states. The robot begins in a WALL state, moving forwards without angular velocity until it comes within the threshold distance of the wall. Then the robot enters a ROTATE state and rotates until there is no object within its field of view ahead. Once its field of view is cleared, it moves forward with angular velocity as determined by the PD controller. It continues to oscillate between ROTATE and FORWARD states as necessary.

### Evaluation

My program does work – the robot successfully follows the wall, keeping within a reasonable margin of the threshold distance and not going within it. The robot does oscillate some when moving forwards, but this is a natural result of the PD controller compensating for error. Adding in an integral term (to make it a PID) controller would have little to no effect besides on the settling time and the steady-state error. Still, it would increase the overshoot and not fix the oscillation problem.

I chose a proportional gain term and derivative term of 1; if I were to increase the proportional gain term, I would increase the overshoot, and if I were to increase the derivative term, I would decrease the overshoot. So, keeping the value of these terms equal appeared to be the most logical way forward for me.

# Path Planner

### Description

My program reads an occupancy grid and searches for a plan for a robot navigating that environment from an arbitrary start location to a goal. The program also visualizes that path using rviz and returns it incrementally to the user's terminal.

The program begins by prompting the user for a start and goal location of their choice. It wraps each point in a point object, which stores the point's coordinates, parent, and pose. The program then builds a search tree by creating a node for every point one unit away from its neighbor in all four directions, unless the coordinates of that node are occupied according to the occupancy map. I searched this tree using breadth-first search, which explores all nodes at a particular depth before proceeding to the next level. Once the algorithm reaches the goal location, it backtracks to the root and stores the path.

I then calculated the robot's pose at each step in its path based on its quaternion rotation. I also scaled the robot's coordinates down from the global reference frame to the map reference frame. Upon doing so, I published the robot's pose and created a marker for each coordinate in rviz. The start and goal locations are represented by green spheres; the individual steps in the robot's path are represented by red arrows.

### Evaluation

My program does work by finding the shortest path between the start and goal locations. At first I thought that perhaps implementing another search algorithm would identify even more optimal paths, but I quickly realized that when all costs are 1 (which they are in the present problem), BFS is optimal. So implementing another search algorithm would not help the optimality of my robot's planning.

However, the robot's behavior is not entirely optimal. The robot has no issue traveling right along a wall, which would probably appear strange if someone actually saw the robot operating in person. Still, it does not mean that there is anything wrong with the robot's path. Perhaps it's better that the robot stay along the wall anyways so as to not cause traffic. I illustrate this behavior in one of the attached screenshots in the present zip file.

Some further clarification on the robot's behavior is likely also necessary. The robot's path is updated every 0.05 units, but I only visualize half of those arrows in rviz so as to not overpopulate the simulation. The arrows also appear to overshoot each turn, but that is just because they are initialized from the tail, not the head. So consider the robot's turning point at the tail location of the "overshooting" arrow.

# Occupancy Grid

### Description

Our objective in this programming assignment was primarily to write algorithms that operate the RosBot. We did this in two parts: (1) estimating the robot's state using LIDAR and (2) mapping an occupancy grid of the robot's surrounding environment. These two programs are described below.

To estimate the robot's state, I first programmed the robot to move in three different ways: (a) translation, (b) relative rotation, and (c) absolute rotation. The code for translation and relative rotation was relatively straightforward as all I had to do was adapt it from past programming assignments. Programming an absolute rotation, however, was slightly more complex because I had to capture the robot's pose in the odom from, then subtract its yaw from the desired angle. After doing so, however, the robot rotated properly.

When using the RosBot, I was faced with translation and rotation errors as a result of the physical environment that I did not face when running these programs on simulations. So, it made sense to measure the error of my translations and rotations, specifically between the desired angle/distance, the lidar-measured angle/distance, and the manually measured angle/distance. I also programmed the robot to make a square by oscillating between translations and 90˚ absolute rotations.

I was then tasked with building an occupancy grid of the robot's surrounding environment. The Occupancy Grid class is initialized with a users preferred grid dimensions and resolution. To fill in the grid, I retrieved the robot's laser scan at each angle and the distance of the perceived object at that angle. I then used the Bresenham algorithm to calculate which grid spaces we can classify as unoccupied given the straight line distance between the robot and a perceived object. Before adding any point to the occupancy grid, I translated it from the laser reference frame to the odom reference frame so it would fit accordingly in my occupancy grid. Finally, I marked a point as occupied at the translated distance from the laser scan reading

### Evaluation

It turns out that just because a robot works in a simulation, it doesn't mean it will work in real life. This is a valuable lesson that I will take with me after this programming assignment.

As far as the actual performance of the program itself, both the state estimation and occupancy grid programs work, but do suffer from some noise. In the translation part of the state estimation problem, I found that my LIDAR readings were significantly more accurate when I put an object about a meter and a half in front of the robot, rather than just allowing the robot to read the wall across the room. Ultimately, the robot averaged less than 0.02m error between the desired distance and both the lidar measured distance and the manually measured distance. When it came to relative rotation, I found that I could only calculate the error using LIDAR when the robot was facing a wall and did not rotate more than 90˚, otherwise the trig that I used to calculate the angle of rotation no longer worked. So, I chose to only experiment with the relative rotation error at 30˚. Still, the robot performed well. It averaged less than 0.05rad error between the desired angle of rotation and the lidar measured angle of rotation while averaging less than 0.02rad error between the desired angle of rotation and the manually measured angle of rotation. As for the square, the video reveals that the robot did successfully make a square, but with slight error in its absolute rotations. This may have been a result of not resetting the robot's pose in the odom frame after running the program multiple times during testing.

As for the occupancy grid, the robot performed well in a static state. The robot perfectly mapped a uniform, square environment. In practice on the RosBot, however, the results were a little more noisy. To test if the RosBot was mapping the environment correctly, I arranged desks on the left side of the classroom while leaving the majority of the right side unoccupied. I then had the robot proceed 3m forward and rotate 90˚. The robot appropriately re-maps the desks after its translation and rotation. The mapping is not perfect, but it does reveal some correlations between the robot's physical state in the environment and its computed state on RVIZ.
