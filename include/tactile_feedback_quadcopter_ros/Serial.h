#pragma once
#include <string>
#include <termios.h>

class Serial {
public:
	Serial(std::string portName);
	~Serial();
	bool setupParameters();
	bool writeMsg(std::string &msg);
	bool write6Byte(unsigned char msg[6]);
	bool writeByte(unsigned char msg);
	bool readMsg(std::string &msg);
	unsigned char readByte();
private:
	std::string portName;
	int device_file_descriptor;
	struct termios tty;
	struct termios tty_old;
};
