#pragma once

#include <sensor_msgs/Range.h>
#include <ardrone_autonomy/Navdata.h>
#include <vector>
class ObstacleObserver {
public:
	enum ObstacleDirection {
		FRONT = 0, BACK = 1, LEFT = 2, RIGHT = 3, NOTHING = 4
	};

	/**
	 *	Constructor sets initially all distances to fardest away distance (distance_thresholds[0])
	 *
	 * @param distance_thresholds 	first entry is fardest away distance,
	 * 								second entry is middle distance,
	 * 								third entry is nearest distance for feedback
	 */
	ObstacleObserver(std::vector<float> &distance_thresholds);

	float getDistBack() const;
	float getDistFront() const;
	float getDistLeft() const;
	float getDistRight() const;
	void setDistBack(const sensor_msgs::Range::ConstPtr& dist_data);
	void setDistFront(const sensor_msgs::Range::ConstPtr& dist_data);
	void setDistLeft(const sensor_msgs::Range::ConstPtr& dist_data);
	void setDistRight(const sensor_msgs::Range::ConstPtr& dist_data);

	void droneInfoCallback(const ardrone_autonomy::Navdata& navdata);

	ObstacleDirection getNearestDirection() const;

	bool isDroneFlying() const;

private:
	bool drone_is_flying;
	std::vector<float> distances;
	std::vector<float> distance_thresholds;
};
