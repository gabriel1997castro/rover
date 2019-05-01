#include "SSC.h"
#include <stdio.h>
#include <string>

int main()
{
	std::string command;
	command = "#0 P1500 #1 P1500";
	sendCommand(command.c_str());
	return 0;
}
