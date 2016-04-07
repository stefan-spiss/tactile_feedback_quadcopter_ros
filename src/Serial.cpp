// Code from http://stackoverflow.com/questions/18108932/linux-c-serial-port-reading-writing

#include <cstdio>      // standard input / output functions
#include <iostream>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions

#include <tactile_feedback_quadcopter_ros/Serial.h>

Serial::Serial(std::string portName) :
		portName(portName) {
	device_file_descriptor = open(portName.c_str(), O_RDWR | O_NOCTTY);
}

Serial::~Serial() {
	unsigned char msg[6];
	msg[0] = 0x01;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x02;

	write6Byte(msg);
	/* Flush Port, then applies tty_old attributes */
	tcflush(device_file_descriptor, TCIFLUSH);
	tcsetattr(device_file_descriptor, TCSANOW, &tty_old);

	close(device_file_descriptor);
}

bool Serial::setupParameters() {
	struct termios tty;
	struct termios tty_old;
	memset(&tty, 0, sizeof tty);

	if (!isatty(device_file_descriptor)) {
		std::cout << "Error " << errno << " no tty device descriptor chosen "
				<< strerror(errno) << std::endl;
		return false;
	}

	/* Error Handling */
	if (tcgetattr(device_file_descriptor, &tty) != 0) {
		std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno)
				<< std::endl;
		return false;
	}

	/* Save old tty parameters */
	tty_old = tty;

	/* Set Baud Rate */
	cfsetospeed(&tty, (speed_t) B9600);
	cfsetispeed(&tty, (speed_t) B9600);

	/* Setting other Port Stuff */
	tty.c_cflag &= ~PARENB;            	// clear parity generation on output and parity checking for input
	tty.c_cflag &= ~CSTOPB;				// clear two stop bits
	tty.c_cflag &= ~CSIZE;				// clear current char size mask
	tty.c_cflag |= CS8;					// force 8 bit input

	tty.c_cflag &= ~CRTSCTS;           	// no hardware flow control
	tty.c_cc[VMIN] = 1;                	// read doesn't block
	//tty.c_cc[VTIME] = 5;				// 0.5 seconds read timeout
	tty.c_cflag |= CREAD | CLOCAL;     	// turn on READ & ignore ctrl lines
	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush(device_file_descriptor, TCIFLUSH);
	if (tcsetattr(device_file_descriptor, TCSANOW, &tty) != 0) {
		std::cout << "Error " << errno << " from tcsetattr" << std::endl;
		return false;
	}

	return true;
}

bool Serial::writeMsg(std::string &msg) {
	msg.append("\r");
	unsigned char* cmd = (unsigned char*) msg.c_str();
	int n_written = 0, spot = 0;

	do {
		n_written = write(device_file_descriptor, &cmd[spot], 1);
		spot += n_written;
	} while (cmd[spot - 1] != '\r' && n_written > 0);
	return true;
}

bool Serial::write6Byte(unsigned char msg[6]) {

	for(int i=0; i<6; i++) {
		if(write(device_file_descriptor, &msg[i], 1)!=1) {
			return false;
		}
		usleep(1000);
	}
	return true;
}

bool Serial::writeByte(unsigned char msg) {
	if(write(device_file_descriptor, &msg, 1)!=1) {
		return false;
	}
	return true;
}


bool Serial::readMsg(std::string &msg) {
	int n = 0, spot = 0;
	char buf = '\0';

	/*Whole response*/
	char response[1024];

	memset(response, '\0', sizeof response);

	do {
		n = read(device_file_descriptor, &buf, 1);
		sprintf(&response[spot], "%c", buf);
		spot += n;
	} while (buf != '\r' && n > 0);

    if (n < 0) {
		std::cout << "Error reading: " << strerror(errno) << std::endl;
		return false;
	} else if (n == 0) {
		std::cout << "Read nothing!" << std::endl;
	} else {
		std::cout << "Response: " << response << std::endl;
	}

	msg = std::string(response);


	return true;
}

unsigned char Serial::readByte() {
	unsigned char buf;

	int n = read(device_file_descriptor, &buf, 1);

	if(n != 1) {
		std::cout << "Error reading "<< n << std::endl;
		buf = n;
	}

	return buf;
}
