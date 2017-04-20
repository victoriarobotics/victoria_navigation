// Copyright 2017 Michael Wimble

// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse
// or promote products derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __VICTORIA_NAVIGATION_DISCOVER_CONE
#define __VICTORIA_NAVIGATION_DISCOVER_CONE

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "victoria_navigation/strategy_fn.h"
#include "victoria_perception/AnnotateDetectorImage.h"
#include "victoria_perception/ObjectDetector.h"

// A behavior that attempts to discover a RoboMagellan cone in the camera.
//
// The behavior depends on a few other components and parameters..
//		* "cone_detector_topic_name" is a topic listened to for an indication if a RoboMagellan cone is detected.
//		* "cmd_vel_topic_name" defines a topic to be used for moving the robot. Messages will be published
//		  to that topic. The robot will end up in a stopped state at the end of this behavior.
//		* "odometry_topic_name" defines a topic the MIGHT be listened to in order to determine the current
//		  heading. See the discussion of "imu_topic_name" above.
//
// The behavior works as follows:
//	* Wait until messages are received from the cone detector and Odometry.
//  * If the cone is seen, indicate SUCCESS and stop the robot.
//	* The first time this behavior is attempted, capture the current Odometry. This will be used
//	  to detect when a complete revolution has been made.
//	* If the robot hasn't yet made a complete revolution, rotate a bit.
//
// POSSIBLE IMPROVEMENTS:
//	* Find all cones in a complete rotation and choose the best.
//	* Choose the cone that is closest to the expected heading.

class DiscoverCone : public StrategyFn {
private:
	enum STATE {
		CAPTURE_STATE,								// Capture the current interesting state.
		ROTATING_TO_DISCOVER,						// Rotate until a RoboMagellan cone is discovered.
		RECOVER_WAIT_NEXT_CONE_DETECTOR_MESSAGE,	// Cone detector was adjusted, see if it worked.
	};

	typedef struct ClusterStatistics {
			int cluster_number = -1;
			int min_hue;
			int max_hue;
			int min_saturation;
			int max_saturation;
			int min_value;
			int max_value;
			int pixels;
			ClusterStatistics() :
				cluster_number(-1),
				min_hue(-1),
				max_hue(-1),
				min_saturation(-1),
				max_saturation(-1),
				min_value(-1),
				max_value(-1),
				pixels(-1) {}
	} ClusterStatistics;

	// Parameters.
	std::string cmd_vel_topic_name_;		// Topic name containing cmd_vel message.
	std::string cone_detector_topic_name_;	// Topic name containing ConeDetector message.
	std::string image_topic_name_;			// Topic name containing video stream.
	std::string odometry_topic_name_;		// Topic name containing Odometry message.
	float yaw_turn_radians_per_sec_;		// Rate to turn around z azis (radians/sec)

	// Publishers.
	ros::Publisher cmd_vel_pub_;

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;
	ros::Subscriber odometry_sub_;

	// Algorithm variables.
	bool cone_detected_sticky_;					// Sticky bit for cone detector object_detected.
	geometry_msgs::Quaternion previous_pose_;	// Pose from last Odometry message.
	int recovery_retry_count_;					// Recovery attempts performed.
	long int recovery_start_sequence_number_;	// Snapshot of count_object_detector_msgs_received_ for recovery.
	nav_msgs::Odometry starting_odometry_msg_;	// Odometry mesage at start of rotation strategy.
	double starting_yaw_;						// Starting yaw.
	double total_rotated_yaw_;					// Integration of rotational yaw since start.
	STATE state_;

	ros::ServiceClient coneDetectorAnnotatorService_;	// For annotating the cone detector image.
	victoria_perception::AnnotateDetectorImage annotator_request_;	// The annotation request.

	ros::ServiceClient computeKmeansService_;	// For recalculating kmeans.
	bool parseClusterStatistics(std::string cluster_statistics_string, ClusterStatistics& result);
	bool isLikelyConeCluster(ClusterStatistics cluster);
	ClusterStatistics computeLikelyConeParameters(const std::vector<ClusterStatistics>& clusters);

	/*! \brief Computer a set of likely-good cone detector parameters for the current video stream. */
	ClusterStatistics findNewConeDetectorParameters();

	/*! \brief Get the current A-filter values from the cone detector. */
	ClusterStatistics getCurrentAFilter();
	
	/*! \brief Put new values for the A-filter. */
	void setCurrentAFilter(ClusterStatistics values);

	/*! \brief attempt to adjust the cone detector parameters to find a cone in the image. */
	bool canFindCone();

	// Process one ConeDetector topic message.
	long int count_object_detector_msgs_received_;
	victoria_perception::ObjectDetector last_object_detector_msg_;
	void coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg);

	// Reset goal. After this, someone must request the goal again and it will start over.
	void resetGoal();

	/*! \brief utility to handle signalling goal failure. */
	RESULT_T failGoal();

	/*! \brief utility to handle signalling goal success. */
	RESULT_T succeedGoal();

	/*! \brief update captured state of total rotated yaw. */
	double computeTotalRotatedYaw();

	/*! \brief Start of problem solver. Capture interesting state. */
	RESULT_T doCaptureState(std::ostringstream& out_ss);

	/*! \brief Rotate a bit to see if the cone was found. */
	RESULT_T doRotatingToDiscover(std::ostringstream& out_ss);

	/*! \brief Waiting to see if cone detector adjustments were successful. */
	RESULT_T doRecoverWaitNextConeDetectionMessage(std::ostringstream& out_ss);

	/*! \brief Make sure the cone detector parameters are valid values. */
	void setSafeConeDetectorParameters(ClusterStatistics& int_out_parameters);

	// Process one Odometry topic message;
	long int count_odometry_msgs_received_;
	nav_msgs::Odometry last_odometry_msg_;
	void odometryCb(const nav_msgs::OdometryConstPtr& msg);

	// Singleton pattern.
	DiscoverCone();
	DiscoverCone(DiscoverCone const&) {}
	DiscoverCone& operator=(DiscoverCone const&) {}

public:
	RESULT_T tick();

	const std::string& goalName() {
		static std::string need_to_discover_cone = "DiscoverCone";
		return need_to_discover_cone;
	}

	const std::string& name() { 
		static std::string name = "DiscoverCone";
		return name;
	}

	static StrategyFn& singleton();

	static const std::string& stateName(STATE state) {
		static const std::string capture_state = "CAPTURE_STATE";
		static const std::string rotating_to_discover = "ROTATING_TO_DISCOVER";
		static const std::string recover_wait_next_cone_detector_message = "RECOVER_WAIT_NEXT_CONE_DETECTOR_MESSAGE";
		static const std::string unknown = "!!UNNOWN!!";

		switch (state) {
			case CAPTURE_STATE:						return capture_state;
			case ROTATING_TO_DISCOVER:				return rotating_to_discover;
			case RECOVER_WAIT_NEXT_CONE_DETECTOR_MESSAGE:	return recover_wait_next_cone_detector_message;
			default:								return unknown;
		}
	}

};

#endif // __VICTORIA_NAVIGATION_DISCOVER_CONE