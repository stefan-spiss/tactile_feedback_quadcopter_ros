/*
 * ObstacleObserver.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: steve
 */

#include <tactile_feedback_quadcopter_ros/ObstacleObserver.h>

ObstacleObserver::ObstacleObserver(std::vector<float> &distance_thresholds) :
		distance_thresholds(distance_thresholds), distances(4,
				distance_thresholds[0]), drone_is_flying(false) {
}

float ObstacleObserver::getDistBack() const {
	return distances[BACK];
}

float ObstacleObserver::getDistFront() const {
	return distances[FRONT];
}

float ObstacleObserver::getDistLeft() const {
	return distances[LEFT];;
}

float ObstacleObserver::getDistRight() const {
	return distances[RIGHT];;
}

void ObstacleObserver::setDistBack(
		const sensor_msgs::Range::ConstPtr& dist_data) {
	distances[BACK] = dist_data->range;
}

void ObstacleObserver::setDistFront(
		const sensor_msgs::Range::ConstPtr& dist_data) {
	distances[FRONT] = dist_data->range;
}

void ObstacleObserver::setDistLeft(
		const sensor_msgs::Range::ConstPtr& dist_data) {
	distances[LEFT] = dist_data->range;
}

void ObstacleObserver::setDistRight(
		const sensor_msgs::Range::ConstPtr& dist_data) {
	distances[RIGHT] = dist_data->range;
}

void ObstacleObserver::droneInfoCallback(
		const ardrone_autonomy::Navdata& navdata) {
	if (navdata.state == 3 || navdata.state == 7 || navdata.state == 4
			|| navdata.state == 6 || navdata.state == 8) {
		drone_is_flying = true;
	} else {
		drone_is_flying = false;
	}
}

bool ObstacleObserver::isDroneFlying() const {
	return drone_is_flying;
}

ObstacleObserver::ObstacleDirection ObstacleObserver::getNearestDirection() const {
	ObstacleDirection nearest_dir = NOTHING;
	float nearest = distance_thresholds[0];
	for (int i = 0; i < distances.size(); i++) {
		if (distances[i] < nearest) {
			nearest = distances[i];
			switch (i) {
			case (0):
				nearest_dir = FRONT;
				break;
			case (1):
				nearest_dir = BACK;
				break;
			case (2):
				nearest_dir = LEFT;
				break;
			case (3):
				nearest_dir = RIGHT;
				break;
			}
		}
	}
	return nearest_dir;
}
