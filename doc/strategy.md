# Strategy

The strategy module works as a goal-directed problem solver managed by **robo_magellan_node**. A goal is pushed onto a stack and each available problem solver is given a chance to decide if it can help achieve the goal. Currently only one problem solver ever picks up the goal at a time, but that is not required.

As a problem solver picks up a goal, it can divide its solution into solving a series of sub problems and push new goals onto the stack.

When a problem solver works on a goal, it responds with a indication of how it handled the goal. If the problem solver succeeds or fails in achieving the goal, it removes the topmost goal from the stack and that status is captured so that any parent problem solver can react. If the response is **FATAL**, the node shuts down.

**robo_magellan_node** really just iterates over and over the list of known problem solvers and continually asks if each problem solver can achieve the current goal until there are no more goals to be achieved.

And example of how this would typically work is a human uses the service API **robom_magellan_node/push_goal** to cause a **SolveRoboMagellan** goal to be pushed onto the goal stack. The **robo_magellan_node** offers each problem solver a chance to achieve that goal, but only the **SolveRoboMagellan** problem solver picks it up—every other problem solver responds to the current goal with an **INACTIVE** response to indicate it can't help achieve the current goal. **SolveRoboMagellan** iterates over each point it found in the YAML-given list and for each point it subdivides the problem into the sequence:

- SeekToGps

  Try to get close to the waypoint.
- DiscoverCone

  Try to find a cone using the camera, rotating if necessary.
- MoveToCone

  Try to touch the cone.
- MoveFromCone

  Backup a bit, giving room for the robot to make a move towards the next GPS waypoint.

**SolveRoboMagellan** begins by pushing **SeekToGps** as the current goal. Now there are two goals on the stack—the topmost or current goal is **SeekToGps** and the goal below that on the stack is **SolveRoboMagellan**. The **SeekToGps** goal asks for a problem solver to move to the current waypoint in the list. **SolveRoboMagellan** waits until the **SeekToGps** is achieved or all problem solvers indicate they cannot achive the goal. 

If the **SeekToGps** goal is achieved, **SolveRoboMagellan** pushes **DiscoverCone** onto the goal stack and so on until **MoveFromCone** is achieved, at which point **SolveRoboMagellan** moves onto the next point. When there are no points left, the original goal is achieved and **SolveRoboMagellan** removes that goal from the stack and the whole process is done.

If, however, one of the sub problem solvers fails, it is expected that **SolveRoboMagellan** will come up with a different recovery strategy than any that might have been employed in the sub problem solvers.

For example, for the **MoveToCone** goal, a problem solver might loose sight of the cone. Especially when it gets close enough, slight movements might move the cone from camera view; or when the cone fills the frame of the camera, it may not pass the test for being a cone anymore (e.g., it may not be narrower at the top than the bottom). A **MoveToCone** problem solver might try a local recovery scheme by, say, pushing the **MoveFromCone** goal onto the stack which would probably back the robot up a bit, then see if that helps. If local recovery techniques fail for a problem solver, though, the goal **fails** and the parent problem solver should try a more complex recovery. For example, **SolveRoboMagellan** might try to move on a radial path to the cone for a bit and try to move at the cone from a different angle.

----
## SolveRoboMagellan
<to be done>

----
## SeekToGps
The problem solver attempts to move the robot near a GPS waypoint (see StrategyFn::**GPS_POINT**). The point used is that which is on top of the point stack (see StrategyFn::**pushGpsPoint**).

Whenever the problem solver is invoked, it begins by waiting on **ConeDetector**, **Odometry**, **Fix** and **Imu** messages. When at least one of each of those messages is received, the problem solver proceeds.

Each invocation of this problem solver tests whether the goal waypoint has an associated cone and, if so, whether the latest **ConeDetector** message indicates that it sees the cone. If so, the problem solver **succeeds** (achieves the goal).

Otherwise, movement towards the goal is required. For that, a heading is needed, and there are two ways to compute it:

* If the parameter **solve_using_odom** is given, the last **Odometry** message and the goal point's **x** and **y** position are used to compute a desired heading and a distance to the goal point is computed using the pythagorean theorem.
* Otherwise, a desired heading is computed using the last **Fix** message's latitude and longitude and the latitude and longitude of the goal point and a distance to the goal point is computed using the haversine formula.

If the computed distance to the goal point is less than the parameter **gps_close_distance_meters**, the goal **succeeds** as being "close enough".

The goal wants to keep the robot pointing directly at the target point. It needs to know the current heading of the robot, and it can determine that in one of two ways:

* If the **use_imu** parameter is _true_, the last **Imu** message is used to get the current heading. The value of the **magnetic_declination** parameter is added to the IMU heading to get a true heading.
* Otherwise the last **Odometry** message's `pose.pose.orientation` is used as the current heading. Note that the odometry heading is expected to already reflect a true heading—no magnetic declination is added.

A goal yaw delta is computed to be the smallest angle between the current heading and the desired goal heading. If the goal yaw delta is less than the parameter **goal_yaw_degrees_delta_threshold**, the current heading is deemed "close enough" for the moment and no yaw correction is made. Otherwise, the robot is commanded to yaw in the desired direction a bit.The `cmd_vel.angular.z` component is set to the parameter value **yaw_turn_radians_per_sec** with the appropriate sign, and the `cmd_vel.linear.x` value is set to half of the **linear_move_meters_per_sec** parameter value. 

The "close enough" test is made to prevent the robot from spending all of its time making relatively slow movements that are unlikely to ever result in the heading being exactlly correct. Adding an x component to the turn command results in a smoother turn. although it does prevent a "turn in place". Since the goal point is some distance away, "turn in place" should not be required.

If no corrected yaw command is needed, the robot is commanded to move forward by issuing a command with `cmd_vel.angular.z` equal to zero and `cmd_vel.linear.x` equal to the parameter value **linear_move_meters_per_sec**.

----
## DiscoverCone
The problem solver attempts to see a cone in the camera and will rotate up to one complete circle in an attempt to do so.

Whenever the problem solver is invoked, it begins by waiting on **ConeDetector** and **Odometry** messages. When at least one of each of those messages is received, the problem solver proceeds.

Each invocation of this problem solver tests whether the latest **ConeDetector** message indicates that it sees the cone. If so, the problem solver **succeeds** (achieves the goal) and the robot is commanded to stop.

Otherwise, the first time the problem solver is invoked for the goal, the current heading is picked up from the last **Odometry** message. Note that the actually heading isn't interesting, so there isn't a need to look at the IMU versus the odometry. The current heading by any means is good enough, as long as the problem solver can tell when a complete turn of 360 degrees has been made later.

For each invocation of the problem solver, it tests to see:
* Has the cone been seen yet?

  If so, the goal is achieved and the problem solver **succeeds**.

* Has the robot rotated 360 degrees from the beginning of this problem solver trying to achieve the goal?

  If so, the goal is not achieved and the problem solver **fails**.

Otherwise the robot is commanded to rotate with `cmd_vel.linear.x` set to zero and `cmd_vel.angular.z` set to 0.4.

----
## MoveToCone
The problem solver attempts to move close to and then touch a cone using information from the camera and the **ConeDetector** message.

Whenever the problem solver is invoked, it begins by waiting on **ConeDetector** messages. When at least one message is received, the problem solver proceeds.

The first time the problem solver is invoked for the goal, it picks up the current time as the last time a cone was detected. This will be used later.

Each time the problem solver is invoked for the goal, it tests whether the cone is still being detected. If not, it will wait up to five seconds for cone detection to occur again. This is because the cone detector is a bit erratic and will report that no cone is detected in one or more frames and, with nothing else changing, it reports that the cone is detected again. Each time a cone is detected, the last time a cone was detected is updated, restarting the five second clock.

The problem solver wants to move towards the cone, with the cone centered in the camera frame, until a bumper sensor says it touched the cone. The bumper hit is detected in one of two ways:

* The topic named by the **distance_displacement_1d_topic_name** parameter publishes an indication that the physical bumper was displaced enough.
* If the **equate_size_to_bumper_hit** parameter is true and the cone detector reports that the area of the detected cone in the frame is the parameter value **cone_area_for_bumper_hit** square pixels or more, a bumper hit is assumed. This was added so other robots could be tested without having a bumper sensor.

If a bumper hit is detected, the goal is achieved, the robot is commanded to stop and the problem solver indicates **success**. Otherise the robot is commanded to move towards the cone, making a centering adjustment along the way. The `cmd_vel.linear.x` value is set to 0.2 and the `cmd_vel.angular.z` value is set to `((image_width / 2.0) - last_object_detected_.object_x) / last_object_detected_.image_width` which is an attempt to steer harder when the cone is more off center, and steer easier when the cone is near the center of the frame.

----
## MoveFromCone

----
## StrategyFn
This class is the parent class of every problem solver. It contains a few class globals that are used by all problem solvers, such as the list of GPS points to be found, the goal stack, a related GPS point stack and the last problem solver response (only the **SUCCESS** and **FAILED** responses are captured).

- typedef struct **GPS_POINT**

  Each waypoint to be visited by the robot is converted from its YAML form into an enhanced structure that holds not only the original GPS latitude and longitude and whether or not the waypoint is expected to have a cone, but also the equivalent absolute x and y point corresponding to that latitude and longitude, with the assumption that x=0 and y=0 is where the robot was positioned at the start (viz., the latitude and longitude in the last **Fix** message at the first time the GPS_POINT was computed). Also, the true bearing (a.k.a., heading) and distance from the previous point is computed. Remember that the first point in the list is **NOT** the starting point of the robot. The starting point is implicityly the **Fix** message latitude and longitude at the time the **GPS_POINT** is created. So the first **GPS_POINT** bearing and distance is that from the starting point. After the first GPS_POINT is computed and pushed onto the list of waypoints, each succeeding point's bearing and distance is that from the previous point in the list.
~~~c++
	typedef struct GPS_POINT {
		double latitude;
		double longitude;
		bool has_cone;
		double x;		// Position relative to start in the Odom frame.
		double y;		// Position relative to start in the Odom frame.
		double bearing;		// Position bearing from previous point.
		double distance;	// Distance to point from previous point.
	} GPS_POINT;
~~~

- enum **RESULT_T**

  As each problem solver is asked to achieve the current goal, it responds with a result that indicates how it did. The most common responses are:
  
	* **INACTIVE**
	
  	  The problem solver doesn't see how it can acheve the goal.
	
	* **RUNNING**
	
  	  The problem solver is actively and incrementally trying to achieve the goal.
	
  	* **SUCCESS**
	
  	  The problem solver successfully achieved the goal and removed the goal from the stack.
	
  	* **FAILED**
	
  	  The problem solver was unable to achieve the goal, even with its own recovery strategies and removed the goal from the stack.
~~~C++
	enum RESULT_T {
		UNUSED_START = 0,	// Do not use, must be first element.
		FAILED,			// Strategy failed, do not continue.
		FATAL,			// Something is fatally wrong.
		INACTIVE,		// Strategy is not active.
		RUNNING,		// Strategy is in progress.
		SUCCESS,		// Strategy succeeded, continue on.
		UNUSED_END		// Do not use, must be last element.
	};
~~~

- static void **pushGpsPoint**(const **GPS_POINT**& gps_point)

  GPS points are an ordered list of points to be traversed by the strategy module. They are setup by invoking **pushGpsPoint**, where the first point pushed is the first point to be found and the last point pushed is the end point (goal)
