#include "rover/SSC.h"
#include <stdio.h>
#include <string>

int main()
{
	std::string command;
	command = "#6 P1500 #7 P1500";
	sendCommand(command.c_str());
	return 0;
}
