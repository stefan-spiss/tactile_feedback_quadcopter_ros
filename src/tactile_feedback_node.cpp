#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tactile_feedback_quadcopter_ros/ObstacleObserver.h>
#include <tactile_feedback_quadcopter_ros/Serial.h>

#define SERIAL_START_BIT	0x01
#define SERIAL_END_BIT		0x02
#define INTENSITY_ZERO		0x00

unsigned char INTENSITY_FARDEST = 100;
unsigned char INTENSITY_MIDDLE = 150;
unsigned char INTENSITY_NEAREST = 250;

float FARDEST = 1.2;
float MIDDLE = 0.8;
float NEAREST = 0.4;

bool PAUSED = false;

void feedback_function_nearest(ObstacleObserver& observer, Serial& serial) {
	unsigned char msg[6];
	msg[0] = SERIAL_START_BIT;
	msg[1] = INTENSITY_ZERO;
	msg[2] = INTENSITY_ZERO;
	msg[3] = INTENSITY_ZERO;
	msg[4] = INTENSITY_ZERO;
	msg[5] = SERIAL_END_BIT;

	if (observer.isDroneFlying()) {
		ObstacleObserver::ObstacleDirection direction =
				observer.getNearestDirection();

		switch (direction) {
		case (ObstacleObserver::FRONT):
			if (observer.getDistFront() < NEAREST) {
				msg[1] = INTENSITY_NEAREST;
				serial.write6Byte(msg);
			} else if (observer.getDistFront() < MIDDLE) {
				msg[1] = INTENSITY_MIDDLE;
				serial.write6Byte(msg);
			} else if (observer.getDistFront() < FARDEST) {
				msg[1] = INTENSITY_FARDEST;
				serial.write6Byte(msg);
			}
			std::cout << "Front is nearest" << std::endl;
			break;
		case (ObstacleObserver::BACK):
			if (observer.getDistBack() < NEAREST) {
				msg[2] = INTENSITY_NEAREST;
				serial.write6Byte(msg);
			} else if (observer.getDistBack() < MIDDLE) {
				msg[2] = INTENSITY_MIDDLE;
				serial.write6Byte(msg);
			} else if (observer.getDistBack() < FARDEST) {
				msg[2] = INTENSITY_FARDEST;
				serial.write6Byte(msg);
			}
			std::cout << "Back is nearest" << std::endl;
			break;
		case (ObstacleObserver::LEFT):
			if (observer.getDistLeft() < NEAREST) {
				msg[3] = INTENSITY_NEAREST;
				serial.write6Byte(msg);
			} else if (observer.getDistLeft() < MIDDLE) {
				msg[3] = INTENSITY_MIDDLE;
				serial.write6Byte(msg);
			} else if (observer.getDistLeft() < FARDEST) {
				msg[3] = INTENSITY_FARDEST;
				serial.write6Byte(msg);
			}
			std::cout << "Left is nearest" << std::endl;
			break;
		case (ObstacleObserver::RIGHT):
			if (observer.getDistRight() < NEAREST) {
				msg[4] = INTENSITY_NEAREST;
				serial.write6Byte(msg);
			} else if (observer.getDistRight() < MIDDLE) {
				msg[4] = INTENSITY_MIDDLE;
				serial.write6Byte(msg);
			} else if (observer.getDistRight() < FARDEST) {
				msg[4] = INTENSITY_FARDEST;
				serial.write6Byte(msg);
			}
			std::cout << "Right is nearest" << std::endl;
			break;
		default:
//				serial.write6Byte(msg);
			std::cout << "Nothing" << std::endl;
		}
	}
	serial.write6Byte(msg);
}

void feedback_function_all_simultaneous(ObstacleObserver& observer,
		Serial& serial) {
	unsigned char msg[6];
	msg[0] = SERIAL_START_BIT;
	msg[1] = INTENSITY_ZERO;
	msg[2] = INTENSITY_ZERO;
	msg[3] = INTENSITY_ZERO;
	msg[4] = INTENSITY_ZERO;
	msg[5] = SERIAL_END_BIT;

	if (observer.isDroneFlying()) {
		float distFront = observer.getDistFront();
		float distBack = observer.getDistBack();
		float distLeft = observer.getDistLeft();
		float distRight = observer.getDistRight();
		if (distFront < NEAREST) {
			msg[1] = INTENSITY_NEAREST;
		} else if (distFront < MIDDLE) {
			msg[1] = INTENSITY_MIDDLE;
		} else if (distFront < FARDEST) {
			msg[1] = INTENSITY_FARDEST;
		}

		if (distBack < NEAREST) {
			msg[2] = INTENSITY_NEAREST;
		} else if (distBack < MIDDLE) {
			msg[2] = INTENSITY_MIDDLE;
		} else if (distBack < FARDEST) {
			msg[2] = INTENSITY_FARDEST;
		}

		if (distLeft < NEAREST) {
			msg[3] = INTENSITY_NEAREST;
		} else if (distLeft < MIDDLE) {
			msg[3] = INTENSITY_MIDDLE;
		} else if (distLeft < FARDEST) {
			msg[3] = INTENSITY_FARDEST;
		}

		if (distRight < NEAREST) {
			msg[4] = INTENSITY_NEAREST;
		} else if (distRight < MIDDLE) {
			msg[4] = INTENSITY_MIDDLE;
		} else if (distRight < FARDEST) {
			msg[4] = INTENSITY_FARDEST;
		}
	}

	serial.write6Byte(msg);
}

int direction = 0;

void feedback_function_all_alternating(ObstacleObserver& observer,
		Serial& serial) {
	unsigned char msg[6];
	msg[0] = SERIAL_START_BIT;
	msg[1] = INTENSITY_ZERO;
	msg[2] = INTENSITY_ZERO;
	msg[3] = INTENSITY_ZERO;
	msg[4] = INTENSITY_ZERO;
	msg[5] = SERIAL_END_BIT;

	if (observer.isDroneFlying()) {
		float distFront, distBack, distLeft, distRight;
		bool isNotSet = true;
		int count = 0;

		while (isNotSet && count < 4) {
			switch (direction) {
			case 0:
				distFront = observer.getDistFront();
				if (distFront < NEAREST) {
					msg[1] = INTENSITY_NEAREST;
					isNotSet = false;
				} else if (distFront < MIDDLE) {
					msg[1] = INTENSITY_MIDDLE;
					isNotSet = false;
				} else if (distFront < FARDEST) {
					msg[1] = INTENSITY_FARDEST;
					isNotSet = false;
				}
				direction++;
				break;
			case 1:
				distBack = observer.getDistBack();
				if (distBack < NEAREST) {
					msg[2] = INTENSITY_NEAREST;
					isNotSet = false;
				} else if (distBack < MIDDLE) {
					msg[2] = INTENSITY_MIDDLE;
					isNotSet = false;
				} else if (distBack < FARDEST) {
					msg[2] = INTENSITY_FARDEST;
					isNotSet = false;
				}
				direction++;
				break;
			case 2:
				distLeft = observer.getDistLeft();
				if (distLeft < NEAREST) {
					msg[3] = INTENSITY_NEAREST;
					isNotSet = false;
				} else if (distLeft < MIDDLE) {
					msg[3] = INTENSITY_MIDDLE;
					isNotSet = false;
				} else if (distLeft < FARDEST) {
					msg[3] = INTENSITY_FARDEST;
					isNotSet = false;
				}
				direction++;
				break;
			case 3:
				distRight = observer.getDistRight();
				if (distRight < NEAREST) {
					msg[4] = INTENSITY_NEAREST;
					isNotSet = false;
				} else if (distRight < MIDDLE) {
					msg[4] = INTENSITY_MIDDLE;
					isNotSet = false;
				} else if (distRight < FARDEST) {
					msg[4] = INTENSITY_FARDEST;
					isNotSet = false;
				}
				direction = 0;
				break;
			}
			count++;
		}
	}
	serial.write6Byte(msg);
}

bool callback_pauseFeedback(std_srvs::Empty::Request request,
		std_srvs::Empty::Response& response) {
	PAUSED = true;
	return true;
}

bool callback_unpauseFeedback(std_srvs::Empty::Request request,
		std_srvs::Empty::Response& response) {
	PAUSED = false;
	return true;
}

void render_all_feedback_once(Serial& serial) {
	unsigned char start_msg[6];
	start_msg[0] = SERIAL_START_BIT;
	start_msg[1] = INTENSITY_FARDEST;
	start_msg[2] = INTENSITY_ZERO;
	start_msg[3] = INTENSITY_ZERO;
	start_msg[4] = INTENSITY_ZERO;
	start_msg[5] = SERIAL_END_BIT;

	serial.write6Byte(start_msg);
	sleep(1);
	start_msg[1] = 0x00;
	serial.write6Byte(start_msg);
	sleep(1);
	start_msg[1] = INTENSITY_MIDDLE;
	serial.write6Byte(start_msg);
	sleep(1);
	start_msg[1] = 0x00;
	serial.write6Byte(start_msg);
	sleep(1);
	start_msg[1] = INTENSITY_NEAREST;
	serial.write6Byte(start_msg);
	sleep(1);
	start_msg[1] = 0x00;
	serial.write6Byte(start_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tactile_feedback_node");
	ros::NodeHandle node;
	ros::NodeHandle local_node("~");

	int feedback_method = 0;
	std::string sensor_topic_front = "/sonar_front";
	std::string sensor_topic_back = "/sonar_back";
	std::string sensor_topic_right = "/sonar_right";
	std::string sensor_topic_left = "/sonar_left";
	std::string serial_port = "/dev/ttyUSB0";
	std::string service_name_pause = "/tactile_feedback/pauseFeedback";
	std::string service_name_unpause = "/tactile_feedback/unpauseFeedback";

	if (local_node.getParam("feedback_method", feedback_method)) {
		std::cout << "Set parameter feedback_method to " << feedback_method
				<< std::endl;
	}
	if (local_node.getParam("sensor_topic_front", sensor_topic_front)) {
		std::cout << "Set parameter sensor_topic_front to "
				<< sensor_topic_front << std::endl;
	}
	if (local_node.getParam("sensor_topic_back", sensor_topic_back)) {
		std::cout << "Set parameter sensor_topic_back to " << sensor_topic_back
				<< std::endl;
	}
	if (local_node.getParam("sensor_topic_right", sensor_topic_right)) {
		std::cout << "Set parameter sensor_topic_left to " << sensor_topic_left
				<< std::endl;
	}
	if (local_node.getParam("sensor_topic_left", sensor_topic_left)) {
		std::cout << "Set parameter sensor_topic_right to "
				<< sensor_topic_right << std::endl;
	}
	if (local_node.getParam("serial_port", serial_port)) {
		std::cout << "Set parameter serial_port to " << serial_port
				<< std::endl;
	}
	if (local_node.getParam("distance_threshold_fardest", FARDEST)) {
		std::cout << "Set parameter distance_threshold_fardest to " << FARDEST
				<< std::endl;
	}
	if (local_node.getParam("distance_threshold_middle", MIDDLE)) {
		std::cout << "Set parameter distance_threshold_middle to " << MIDDLE
				<< std::endl;
	}
	if (local_node.getParam("distance_threshold_nearest", NEAREST)) {
		std::cout << "Set parameter distance_threshold_nearest to " << NEAREST
				<< std::endl;
	}
	if (local_node.getParam("service_name_pause", service_name_pause)) {
		std::cout << "Set parameter service_name_pause to "
				<< service_name_pause << std::endl;
	}
	if (local_node.getParam("service_name_unpause", service_name_unpause)) {
		std::cout << "Set parameter service_name_unpause to "
				<< service_name_unpause << std::endl;
	}

	int tmp;
	if (local_node.getParam("intensity_fardest", tmp)) {
		if (tmp >= 0 && tmp <= 255) {
			INTENSITY_FARDEST = tmp;
			std::cout << "Set INTENSITY_FARDEST to "
					<< static_cast<int>(INTENSITY_FARDEST) << std::endl;
		} else {
			std::cerr
					<< "Error setting parameter INTENSITY_FARDEST. Value must be between 0 and 255."
					<< std::endl;
		}
	}
	if (local_node.getParam("intensity_middle", tmp)) {
		if (tmp >= 0 && tmp <= 255) {
			INTENSITY_MIDDLE = tmp;
			std::cout << "Set INTENSITY_MIDDLE to "
					<< static_cast<int>(INTENSITY_MIDDLE) << std::endl;
		} else {
			std::cerr
					<< "Error setting parameter INTENSITY_MIDDLE. Value must be between 0 and 255."
					<< std::endl;
		}
	}
	if (local_node.getParam("intensity_nearest", tmp)) {
		if (tmp >= 0 && tmp <= 255) {
			INTENSITY_NEAREST = tmp;
			std::cout << "Set INTENSITY_NEAREST to "
					<< static_cast<int>(INTENSITY_NEAREST) << std::endl;
		} else {
			std::cerr
					<< "Error setting parameter INTENSITY_NEAREST. Value must be between 0 and 255."
					<< std::endl;
		}
	}

	std::vector<float> distance_thresholds(3);

	distance_thresholds[0] = FARDEST;
	distance_thresholds[1] = MIDDLE;
	distance_thresholds[2] = NEAREST;

	ObstacleObserver obstacle_observer(distance_thresholds);

	Serial serial(serial_port);
	if (serial.setupParameters() == false) {
		std::cerr << "Couldn't setup connection to tactile feedback controller."
				<< std::endl;
		return EXIT_FAILURE;
	}

//Render all feedbacks once for recognition
	render_all_feedback_once(serial);

	ros::Subscriber sub_front = node.subscribe<sensor_msgs::Range>(
			sensor_topic_front, 1, &ObstacleObserver::setDistFront,
			&obstacle_observer);
	ros::Subscriber sub_back = node.subscribe<sensor_msgs::Range>(
			sensor_topic_back, 1, &ObstacleObserver::setDistBack,
			&obstacle_observer);
	ros::Subscriber sub_left = node.subscribe<sensor_msgs::Range>(
			sensor_topic_left, 1, &ObstacleObserver::setDistLeft,
			&obstacle_observer);
	ros::Subscriber sub_right = node.subscribe<sensor_msgs::Range>(
			sensor_topic_right, 1, &ObstacleObserver::setDistRight,
			&obstacle_observer);

	ros::Subscriber sub_is_flying = node.subscribe("/ardrone/navdata", 1,
			&ObstacleObserver::droneInfoCallback, &obstacle_observer);

	ros::ServiceServer pauseFeedback_srv = node.advertiseService<
			std_srvs::Empty::Request, std_srvs::Empty::Response>(
			service_name_pause, callback_pauseFeedback);

	ros::ServiceServer unpauseFeedback_srv = node.advertiseService<
			std_srvs::Empty::Request, std_srvs::Empty::Response>(
			service_name_unpause, callback_unpauseFeedback);

	void (*feedback_function)(ObstacleObserver&, Serial&);

	switch (feedback_method) {
	case 0:
		feedback_function = &feedback_function_nearest;
		break;
	case 1:
		feedback_function = &feedback_function_all_simultaneous;
		break;
	case 2:
		feedback_function = &feedback_function_all_alternating;
		break;
	}

	ros::Rate rate(10.0);

	unsigned char zero_msg[6];
	zero_msg[0] = SERIAL_START_BIT;
	zero_msg[1] = INTENSITY_ZERO;
	zero_msg[2] = INTENSITY_ZERO;
	zero_msg[3] = INTENSITY_ZERO;
	zero_msg[4] = INTENSITY_ZERO;
	zero_msg[5] = SERIAL_END_BIT;

	int isOn = 0;

	while (ros::ok()) {
		ros::spinOnce();
		if (!PAUSED) {
			if (isOn % 3 == 2) {
				serial.write6Byte(zero_msg);
			} else if (isOn % 3 == 0) {
				feedback_function(obstacle_observer, serial);
			}
			isOn++;
		} else {
			serial.write6Byte(zero_msg);
		}
		rate.sleep();
	}

}
